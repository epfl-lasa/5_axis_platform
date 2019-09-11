#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! #1
void Platform::communicateToRos()
{
  // Publish foot output
   pubFootOutput();
   _nh.spinOnce(); //Publishes and Retrieves Messages
   // For Retrieving and Publishing to ROS. We can put it separate in the main in  case we want to put it in an interruption     
}

//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********
//! #2
void Platform::updateFootInput(const custom_msgs::FootInputMsg_v2 &msg)
{
  me->_commControlledAxis=msg.set_axis; 
  for (int k=0; k<NB_AXIS; k++)
  {
    me->_wrenchD_ADD[NORMAL][k]=msg.set_effort[k];
    me->_commPoseSet[k]=msg.set_position[k];
    me->_commTwistSet[k]=msg.set_twist[k];
  }
}

//*****************ROS-SERVICES-CALLBACKS*************************/
//! #3
void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  me->_newState = (Platform::State) req.machine_state;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc
  for (int j=0; j<NB_WRENCH_COMPONENTS; j++){
      me->_desWrenchComponents[j]=req.wrench_comp[j];
  } 
  if (!(me->_newState==me->_state)) // If I want to go to a new state
  { 
    me->_flagClearLastState=true;
    resp.mS_new=true; 
  } 
  else{ 
    resp.mS_new=false; 
    } //! You are already in the desired state 
}

//! #4
void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  me->_controllerType=(Platform::Controller)req.set_ctrlType; 
  me->_flagDefaultControl=req.default_ctrl;
  me->_commControlledAxis=req.set_axis; 
  
  if ((me->_state!=TELEOPERATION) && (me->_state!=ROBOT_STATE_CONTROL))
    { resp.sC_ok=false;}
  else
  {
    resp.sC_ok=true;
    for (int k=0; k<NB_AXIS; k++){
            me->_kpPose[k]=req.pos_p[k];
            me->_kiPose[k]=req.pos_i[k];
            me->_kdPose[k]=req.pos_d[k]; 
            me->_kpTwist[k]=req.speed_p[k];
            me->_kiTwist[k]=req.speed_i[k];
            me->_kdTwist[k]=req.speed_d[k]; 
    }
  }
}

//*****************ROS-PUBLISHER*************************/

//! #5
void Platform::pubFootOutput()
{
  _msgFootOutput.id = PLATFORM_ID;
  _msgFootOutput.stamp = _nh.now();
  for (int k=0; k<NB_AXIS; k++)
  {
    _msgFootOutput.position[k] = _pose[k];
    _msgFootOutput.speed[k]= _twist[k];
    _msgFootOutput.ctrl_efforts[k] =_wrenchD[k];
    _msgFootOutput.meas_efforts[k] =_wrenchM[k];
  }
  _msgFootOutput.machine_state=_state;
  _msgFootOutput.controller_type=_controllerType;
  _pubFootOutput->publish(&_msgFootOutput);
}