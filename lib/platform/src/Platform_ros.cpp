#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>



void Platform::communicateToRos()
{
  // Publish foot output
   pubFootOutput();
   _nh.spinOnce(); //Publishes and Retrieves Messages
   // For Retrieving and Publishing to ROS. We can put it separate in the main in  case we want to put it in an interruption     
}


//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********
//! 1
void Platform::updateFootInput(const custom_msgs::FootInputMsg_v2 &msg)
{
  for (uint c = 0 ; c < NB_FI_CATEGORY; c++) {
    me->_flagInputReceived[c] = true; //! To be used specially for the telemanipulation state
  }

  for (uint k=0; k<NB_AXIS; k++)
  {
      me->_ros_position[k]=msg.ros_position[k];
      me->_ros_speed[k]=msg.ros_speed[k];
      me->_ros_effort[k] = msg.ros_effort[k];
      // me->_effortD_ADD[NORMAL][k]=msg.ros_effort[k];
  }
}


//*****************ROS-SERVICES-CALLBACKS*************************/

//! 2
void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  Platform::State newState = (Platform::State) req.ros_machineState;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc
  for (uint j=0; j<NB_EFFORT_COMPONENTS; j++){
      me->_ros_effortComp[j]=req.ros_effortComp[j];
  }  
  
  if (!(newState==me->_state)) // If I want to go to a new state
  { 
    resp.platform_newState=true;
    me->_flagClearLastState=true;
    me->_state = newState;
  } 
  else{ resp.platform_newState=false; } //! You are already in the desired state 
}

//! 3
void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  me->_ros_controllerType=(Platform::Controller) req.ros_controllerType; 
  me->_ros_flagDefaultControl=req.ros_defaultControl;
  me->_ros_ControlledAxis=req.ros_controlledAxis; 
  
  if ((me->_state!=TELEOPERATION) && (me->_state!=ROBOT_STATE_CONTROL))
    { resp.platform_controlOk=false; }
  else
  {
    resp.platform_controlOk=true;
    float scale=0.0f; 
    for (uint k=0; k<NB_AXIS; k++)
    { 
       if(k<PITCH) {scale=SCALE_GAINS_LINEAR_POSITION;}
       else{scale=SCALE_GAINS_ANGULAR_POSITION;}

       me->_kpPosition[k]=req.ros_posP[k] * scale;
       me->_kiPosition[k]=req.ros_posI[k] * scale;
       me->_kdPosition[k]=req.ros_posD[k] * scale; 

       if(k<PITCH) {scale=SCALE_GAINS_LINEAR_SPEED;}
       else{scale=SCALE_GAINS_ANGULAR_SPEED;}

       me->_kpSpeed[k]=req.ros_speedP[k] * scale;
       me->_kiSpeed[k]=req.ros_speedI[k] * scale;
       me->_kdSpeed[k]=req.ros_speedD[k] * scale; 
    }
  }
}


//! 4
void Platform::pubFootOutput()
{
  _msgFootOutput.platform_stamp = _nh.now();
  _msgFootOutput.platform_id = PLATFORM_ID;
  for (uint k=0; k<NB_AXIS; k++)
  {
    _msgFootOutput.platform_position[k] = _position[k];
    _msgFootOutput.platform_speed[k]= _speed[k];
    _msgFootOutput.platform_effortD[k] =_effortD[k];
    _msgFootOutput.platform_effortM[k] =_effortM[k];
  }
    _msgFootOutput.platform_controllerType= (uint8_t)_ros_controllerType; 
    _msgFootOutput.platform_machineState=(uint8_t)_state;
  _pubFootOutput->publish(&_msgFootOutput);
}
