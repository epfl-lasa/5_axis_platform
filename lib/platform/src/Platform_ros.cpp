#include "Platform.h"
#include "macros_force_measurement.h"

void Platform::communicateToRos()
{
  // Publish foot output
  _platformMutex.lock();
   pubFootOutput();
   _nh.spinOnce(); //Publishes and Retrieves Messages
   // For Retrieving and Publishing to ROS. We can put it separate in the main in  case we want to put it in an interruption
   _platformMutex.unlock();
}


//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********
//! 1
void Platform::updateFootInput(const custom_msgs::FootInputMsg_v3 &msg)
{
  me->_platformMutex.lock();
  for (uint c = 0 ; c < NB_FI_CATEGORY; c++) {
    me->_flagInputReceived[c] = true; //! To be used specially for the telemanipulation state
  }

  for (uint k=0; k<PITCH; k++)
  {
      me->_ros_position[k]=msg.ros_position[rosAxis[k]];
      me->_ros_speed[k]=msg.ros_speed[rosAxis[k]];
      me->_ros_effort[k] = msg.ros_effort[rosAxis[k]];
  }
  
  for (uint k = PITCH; k < NB_AXIS; k++) {
    me->_ros_position[k] = msg.ros_position[rosAxis[k]]*DEG_TO_RAD;
    me->_ros_speed[k] = msg.ros_speed[rosAxis[k]] * DEG_TO_RAD;
    me->_ros_effort[k] = msg.ros_effort[rosAxis[k]];
  }
  
  for (uint c=0; c<NB_AXIS_WRENCH; c++)
  {
    me->_ros_forceSensor[c] = msg.ros_forceSensor[c]; //! In this one the rosAxis doesn't apply
  }
  me->_platformMutex.unlock();
}

//*****************ROS-SERVICES-CALLBACKS*************************/

//! 2
void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  me->_platformMutex.lock();
  State newState = (State) req.ros_machineState;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc
  for (uint j=0; j<NB_EFFORT_COMPONENTS; j++){
      me->_ros_effortComp[j]=req.ros_effortComp[j];
  }  
  
  if (!(newState==me->_platform_state)) // If I want to go to a new state
  { 
    resp.platform_newState=true;
    me->_flagClearLastState=true;
    me->_ros_state = newState;
  } 
  else{ resp.platform_newState=false; } //! You are already in the desired state
  me->_platformMutex.unlock();
}

//! 3
void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  me->_platformMutex.lock();
  if ((me->_platform_state==TELEOPERATION) || (me->_platform_state==ROBOT_STATE_CONTROL))
  {
    Controller newController = (Controller) req.ros_controllerType;
    if (me->_platform_controllerType != newController)
    {
      me->_flagControllerTypeChanged = true;
    }
      me->_ros_controllerType = newController;
   me->_ros_flagDefaultControl = req.ros_defaultControl;
   me->_ros_controlledAxis=req.ros_controlledAxis ==-1 ? -1:rosAxis[req.ros_controlledAxis]; 
   if (!me->_ros_flagDefaultControl){
    float scale=0.0f; 
    for (uint k=0; k<NB_AXIS; k++)
    { 
       if(k<PITCH) {scale=SCALE_GAINS_LINEAR_POSITION;}
       else{scale=SCALE_GAINS_ANGULAR_POSITION;}

       me->_ros_kpPosition[k]=req.ros_posP[rosAxis[k]] * scale;
       me->_ros_kiPosition[k]=req.ros_posI[rosAxis[k]] * scale;
       me->_ros_kdPosition[k]=req.ros_posD[rosAxis[k]] * scale; 

       if(k<PITCH) {scale=SCALE_GAINS_LINEAR_SPEED;}
       else{scale=SCALE_GAINS_ANGULAR_SPEED;}

       me->_ros_kpSpeed[k]=req.ros_speedP[rosAxis[k]] * scale;
       me->_ros_kiSpeed[k]=req.ros_speedI[rosAxis[k]] * scale;
       me->_ros_kdSpeed[k]=req.ros_speedD[rosAxis[k]] * scale; 
    }
    me->_flagCtrlGainsNew=true;
   }
    resp.platform_controlOk=true;
  }
  else
  {
    resp.platform_controlOk=false; 
  }
  me->_platformMutex.unlock();
}


//! 4
void Platform::pubFootOutput()
{
  _msgFootOutput.platform_stamp = _nh.now();
  _msgFootOutput.platform_id = PLATFORM_ID;
  
  for (uint k=0; k<PITCH; k++)
  {
    _msgFootOutput.platform_position[rosAxis[k]] = _position(k);
    _msgFootOutput.platform_speed[rosAxis[k]]= _speed(k);
    _msgFootOutput.platform_effortD[rosAxis[k]] =_effortD(k);
    _msgFootOutput.platform_effortM[rosAxis[k]] =_effortM(k);
  }

  for (uint k = PITCH; k < NB_AXIS; k++) {
    _msgFootOutput.platform_position[rosAxis[k]] = _position(k) * RAD_TO_DEG;
    _msgFootOutput.platform_speed[rosAxis[k]] = _speed(k) * RAD_TO_DEG;
    _msgFootOutput.platform_effortD[rosAxis[k]] = _effortD(k);
    _msgFootOutput.platform_effortM[rosAxis[k]] = _effortM(k);
  }

  //_msgFootOutput.platform_effortM[0] = _timestep;
  _msgFootOutput.platform_controllerType = (uint8_t)_platform_controllerType;
  _msgFootOutput.platform_machineState = (uint8_t)_platform_state;
  _pubFootOutput->publish(&_msgFootOutput);
}

void Platform::updatePlatformFromRos() {

  calculateMeasTorques();
  if (_flagClearLastState) {
    clearLastState();
    _platform_state = _ros_state;
    _flagClearLastState = false;
  }

  switch (_platform_state)

  {
    case TELEOPERATION: case ROBOT_STATE_CONTROL:
    {
        for (uint j = NORMAL; j < NB_EFFORT_COMPONENTS;j++) // {NORMAL*, CONSTRAINS*, COMPENSATION, FEEDFORWARD}
        {
          _platform_effortComp[j] = _ros_effortComp[j];
        }

        if (_flagControllerTypeChanged) {
          resetControllers();
          _flagControllerTypeChanged = false;
        }
        _platform_controllerType = _ros_controllerType;

        if (_ros_flagDefaultControl && !_platform_flagDefaultControl) {
          _flagDefaultCtrlNew = true;
        }
        _platform_flagDefaultControl = _ros_flagDefaultControl;

        _platform_controlledAxis = _ros_controlledAxis;

        if (_flagDefaultCtrlNew) {
          loadDefaultPIDGains();
          if (_platform_state == TELEOPERATION) {
            _virtualWall =
                (Eigen::Map<const Eigen::MatrixXf>(C_WS_LIMITS, NB_AXIS, 1));
          }
          _flagDefaultCtrlNew = false;
        }

        else if (_flagCtrlGainsNew) {
          loadROSPIDGains();
          _flagCtrlGainsNew = false;
        }
    
    }
  }
   
}

void Platform::calculateMeasTorques() {

// Eigen::MatrixXf forceSensorWRTBASE(NB_AXIS_WRENCH,1),
// forceSensorRAW(NB_AXIS_WRENCH,1);
// Eigen::MatrixXf rotationFS(NB_CART_AXIS,NB_CART_AXIS) =
// rotationMatrix(FRAME_FS);
// for (int c=0; c<NB_AXIS_WRENCH; c++)
//  {
//    forceSensorRAW(c)= _ros_forceSensor[c];
//  }
// forceSensorWRTBASE.head(3) = rotationFS*forceSensorRAW.head(3);
// forceSensorWRTBASE.tail(3) = rotationFS*forceSensorRAW.tail(3);
//_effortM=geometricJacobian(FRAME_FS).transpose()*forceSensorWRTBASE;

  _effortM(Y) = effortM_Y;
  _effortM(X) = effortM_X;
  _effortM(PITCH) = effortM_PITCH;
  _effortM(ROLL) = effortM_ROLL;
  _effortM(YAW) = effortM_YAW;
}