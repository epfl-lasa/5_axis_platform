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
  me->_commControlledAxis=msg.set_axis; 
  if (msg.set_axis<=-1){ //! Take info of all axis
    for (int k=0; k<NB_AXIS; k++){
      me->_wrenchD_ADD[NORMAL][k]=msg.set_effort[k];
      if ((me->_controllerType!=TWIST_ONLY) && (me->_controllerType!=TORQUE_ONLY))
        {
            me->_commPoseSet[k]=msg.set_position[k];
        }
      if ((me->_controllerType!=POSE_ONLY) && (me->_controllerType!=TORQUE_ONLY))
        {
            me->_commTwistSet[k]=msg.set_twist[k];
        }
    }
  }

  else{ //Take info of only the axis of interest
      me->_wrenchD_ADD[NORMAL][msg.set_axis]=msg.set_effort[msg.set_axis];
        if ((me->_controllerType!=TWIST_ONLY) && (me->_controllerType!=TORQUE_ONLY))
        {
            me->_commPoseSet[msg.set_axis]=msg.set_position[msg.set_axis];
        }
        if ((me->_controllerType!=POSE_ONLY) && (me->_controllerType!=TORQUE_ONLY))
        {
            me->_commTwistSet[msg.set_axis]=msg.set_twist[msg.set_axis];
        }
    }  
}

//*****************ROS-SERVICES-CALLBACKS*************************/

//! 2
void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  Platform::State newState = (Platform::State) req.machine_state;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc
  for (int j=0; j<NB_WRENCH_COMPONENTS; j++){
      me->_desWrenchComponents[j]=req.wrench_comp[j];
  }  
  
  if (!(newState==me->_state)) // If I want to go to a new state
  { 
    resp.mS_new=true;
    me->_flagClearLastState=true;
    me->_newState = newState;
  } 
  else{ resp.mS_new=false; } //! You are already in the desired state 
}

//! 3
void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  me->_controllerType=(Platform::Controller) req.set_ctrlType; 
  me->_flagDefaultControl=req.default_ctrl;
  me->_commControlledAxis=req.set_axis; 
  
  if ((me->_state!=TELEOPERATION) && (me->_state!=ROBOT_STATE_CONTROL))
    { resp.sC_ok=false; }
  else
  {
    resp.sC_ok=true;
    if (req.set_axis<=-1){ //! Every Axis

      for (int k=0; k<NB_AXIS; k++){
          if ((req.set_ctrlType!=TWIST_ONLY) && (req.set_ctrlType!=TORQUE_ONLY))
          {
            me->_kpPose[k]=req.pos_p[k];
            me->_kiPose[k]=req.pos_i[k];
            me->_kdPose[k]=req.pos_d[k]; 
          }
          if ((req.set_ctrlType!=POSE_ONLY) && (req.set_ctrlType!=TORQUE_ONLY))
          {
            me->_kpTwist[k]=req.speed_p[k];
            me->_kiTwist[k]=req.speed_i[k];
            me->_kdTwist[k]=req.speed_d[k]; 
          }
      }
    }
    else //! One Axis
    {
          if ((req.set_ctrlType!=TWIST_ONLY) && (req.set_ctrlType!=TORQUE_ONLY))
          {
            me->_kpPose[req.set_axis]=req.pos_p[req.set_axis];
            me->_kiPose[req.set_axis]=req.pos_i[req.set_axis];
            me->_kdPose[req.set_axis]=req.pos_d[req.set_axis]; 
          }
          if ((req.set_ctrlType!=POSE_ONLY) && (req.set_ctrlType!=TORQUE_ONLY))
          {
            me->_kpTwist[req.set_axis]=req.speed_p[req.set_axis];
            me->_kiTwist[req.set_axis]=req.speed_i[req.set_axis];
            me->_kdTwist[req.set_axis]=req.speed_d[req.set_axis]; 
          }
      }
    }
}


//! 4
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
    _msgFootOutput.controller_type= (uint8_t)_controllerType; 
    _msgFootOutput.machine_state=(uint8_t)_state;
  _pubFootOutput->publish(&_msgFootOutput);
}
