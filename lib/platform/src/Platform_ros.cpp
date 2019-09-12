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
  me->_rosControlledAxis=msg.ros_controlledAxis; 
  for (int k=0; k<NB_AXIS; k++)
  {
      me->_effortD_ADD[NORMAL][k]=msg.ros_effort[k];
      me->_rosPosition[k]=msg.ros_position[k];
      me->_rosSpeed[k]=msg.ros_speed[k];
  }
}


//*****************ROS-SERVICES-CALLBACKS*************************/

//! 2
void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  Platform::State newState = (Platform::State) req.ros_machineState;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc
  for (int j=0; j<NB_WRENCH_COMPONENTS; j++){
      me->_rosEffortComponents[j]=req.ros_effortComp[j];
  }  
  
  if (!(newState==me->_state)) // If I want to go to a new state
  { 
    resp.platform_newState=true;
    me->_flagClearLastState=true;
    me->_newState = newState;
  } 
  else{ resp.platform_newState=false; } //! You are already in the desired state 
}

//! 3
void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  me->_controllerType=(Platform::Controller) req.ros_controllerType; 
  me->_flagDefaultControl=req.ros_defaultControl;
  me->_rosControlledAxis=req.ros_controlledAxis; 
  
  if ((me->_state!=TELEOPERATION) && (me->_state!=ROBOT_STATE_CONTROL))
    { resp.platform_controlOk=false; }
  else
  {
    resp.platform_controlOk=true;
    for (int k=0; k<NB_AXIS; k++)
    {
       me->_kpPosition[k]=req.ros_posP[k];
       me->_kiPosition[k]=req.ros_posI[k];
       me->_kdPosition[k]=req.ros_posD[k]; 
       me->_kpSpeed[k]=req.ros_speedP[k];
       me->_kiSpeed[k]=req.ros_speedI[k];
       me->_kdSpeed[k]=req.ros_speedD[k]; 
    }
  }
}


//! 4
void Platform::pubFootOutput()
{
  _msgFootOutput.platform_stamp = _nh.now();
  _msgFootOutput.platform_id = PLATFORM_ID;
  for (int k=0; k<NB_AXIS; k++)
  {
    _msgFootOutput.platform_position[k] = _position[k];
    _msgFootOutput.platform_speed[k]= _speed[k];
    _msgFootOutput.platform_effortsD[k] =_effortD[k];
    _msgFootOutput.platform_effortsM[k] =_effortM[k];
  }
    _msgFootOutput.platform_controllerType= (uint8_t)_controllerType; 
    _msgFootOutput.platform_machineState=(uint8_t)_state;
  _pubFootOutput->publish(&_msgFootOutput);
}
