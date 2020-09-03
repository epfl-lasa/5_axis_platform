#include "Platform.h"
#include "macros_force_measurement.h"

void Platform::communicateToRos()
{
  // Publish foot output
  //_platformMutex.lock();
    pubFootOutput();
    _nh.spinOnce(); //Publishes and Retrieves Messages
  //_platformMutex.unlock();
   // For Retrieving and Publishing to ROS. We can put it separate in the main in  case we want to put it in an interruption
}


//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********
//! 1
void Platform::updateFootInput(const custom_msgs::FootInputMsg_v3 &msg)
{
  //me->_platformMutex.lock();
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
  //me->_platformMutex.unlock();
}

//*****************ROS-SERVICES-CALLBACKS*************************/

//! 2
void Platform::updateState(const custom_msgs::setStateSrv_v2::Request &req, custom_msgs::setStateSrv_v2Response &resp )
{
  //me->_platformMutex.lock();
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
  //me->_platformMutex.unlock();
}

//! 3
void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  //me->_platformMutex.lock();
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
  //me->_platformMutex.unlock();
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

  _msgFootOutput.platform_effortM[0] = _timestep;
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
          resetControllers(_platform_controllerType);
          _flagControllerTypeChanged = false;
        }
        _platform_controllerType = _ros_controllerType;

        if (_ros_flagDefaultControl && !_platform_flagDefaultControl) {
          _flagDefaultCtrlNew = true;
        }
        _platform_flagDefaultControl = _ros_flagDefaultControl;

        _platform_controlledAxis = _ros_controlledAxis;

        if (_flagDefaultCtrlNew) {
          loadParamPIDGains();
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
  _effortMNEG=-_effortM;
}

void Platform::retrieveParams(Param_Category category_)
{  ///! These parameters require further application of the scale. See LoadParamPIDGains
  if (category_==ALL)
  {
    if (!_flagLoadParams)
    {
      for (int c_ = 0; c_<ALL; c_++)
      {
        retrieveParams((Param_Category) c_);
      }
      _flagLoadParams = true;
    }
  }
  
  else
  {
    //_platformMutex.lock();
    switch (category_)
    {
      case PID_POS_C:
      {
        if(_platform_state==TELEOPERATION)
        { 
          if (!_nh.getParam(PARAM_P_WALL_NAME, _rosParam_kpPosition, NB_AXIS)) {
            for (int k = 0; k < NB_AXIS; k++) {
              _rosParam_kpPosition[k] = C_WS_PID_GAINS[KP][k];
            }
          }
          if (!_nh.getParam(PARAM_I_WALL_NAME, _rosParam_kiPosition, NB_AXIS)) {
            for (int k = 0; k < NB_AXIS; k++) {
              _rosParam_kiPosition[k] = C_WS_PID_GAINS[KI][k];
            }
          }
          if (!_nh.getParam(PARAM_D_WALL_NAME, _rosParam_kdPosition, NB_AXIS)) {
            for (int k = 0; k < NB_AXIS; k++) {
              _rosParam_kdPosition[k] = C_WS_PID_GAINS[KD][k];
            }
          }
        }
        else 
        {
          if (!_nh.getParam(PARAM_P_POS_NAME, _rosParam_kpPosition, NB_AXIS)) {
            for (int k = 0; k < NB_AXIS; k++) {
              _rosParam_kpPosition[k] = POS_PID_GAINS_DEFAULT[KP][k];
            }
          }
          if (!_nh.getParam(PARAM_I_POS_NAME, _rosParam_kiPosition, NB_AXIS)) {
            for (int k = 0; k < NB_AXIS; k++) {
              _rosParam_kiPosition[k] = POS_PID_GAINS_DEFAULT[KI][k];
            }
          }
          if (!_nh.getParam(PARAM_D_POS_NAME, _rosParam_kdPosition, NB_AXIS)) {
            for (int k = 0; k < NB_AXIS; k++) {
              _rosParam_kdPosition[k] = POS_PID_GAINS_DEFAULT[KD][k];
            }
          }
        }
        break;
      }
      case PID_VEL_C:
      {
        if (!_nh.getParam(PARAM_P_SPEED_NAME, _rosParam_kpSpeed, NB_AXIS)) {
          for (int k = 0; k < NB_AXIS; k++) {
            _rosParam_kpSpeed[k] = SPEED_PID_GAINS_DEFAULT[KP][k];
          }
        }
        if (!_nh.getParam(PARAM_I_SPEED_NAME, _rosParam_kiSpeed, NB_AXIS)) {
          for (int k = 0; k < NB_AXIS; k++) {
            _rosParam_kiSpeed[k] = SPEED_PID_GAINS_DEFAULT[KI][k];
          }
        }
        if (!_nh.getParam(PARAM_D_SPEED_NAME, _rosParam_kdSpeed, NB_AXIS)) {
          for (int k = 0; k < NB_AXIS; k++) {
            _rosParam_kdSpeed[k] = SPEED_PID_GAINS_DEFAULT[KD][k];
          }
        }
        break;
      }
      case PID_FS_C:
      {
        if (!_nh.getParam(PARAM_P_FS_NAME, _rosParam_kpFS, NB_AXIS)) {
          for (int k = 0; k < NB_AXIS; k++) {
            _rosParam_kpFS[k] = FS_PID_GAINS_DEFAULT[KP][k];
          }
        }
        if (!_nh.getParam(PARAM_I_FS_NAME, _rosParam_kiFS, NB_AXIS)) {
          for (int k = 0; k < NB_AXIS; k++) {
            _rosParam_kiFS[k] = FS_PID_GAINS_DEFAULT[KI][k];
          }
        }
        if (!_nh.getParam(PARAM_D_FS_NAME, _rosParam_kdFS, NB_AXIS)) {
          for (int k = 0; k < NB_AXIS; k++) {
            _rosParam_kdFS[k] = FS_PID_GAINS_DEFAULT[KD][k];
          }
        }
        break;  
      }
      case PID_RCM_C: {
        if (!_nh.getParam(PARAM_P_RCM_NAME, &_rosParam_kpRCM)) {
            _rosParam_kpRCM = RCM_PID_GAINS_DEFAULT[KP];
        }
        if (!_nh.getParam(PARAM_I_RCM_NAME, &_rosParam_kiRCM)) {
            _rosParam_kiRCM = RCM_PID_GAINS_DEFAULT[KI];
        }
        if (!_nh.getParam(PARAM_D_RCM_NAME, &_rosParam_kdRCM)) {
            _rosParam_kdRCM = RCM_PID_GAINS_DEFAULT[KD];
        }
        if (!_nh.getParam(PARAM_POS_RCM_NAME, _rosParam_posRCM,NB_CART_AXIS)) {
          for (int a=0; a<NB_CART_AXIS; a++)
          {
            _rosParam_posRCM[a] = RCM_POS_DEFAULT[a]; // temporary
          }
        }
        break;
      }
      case COMPENSATION_C:
      {
        if (_platform_state == TELEOPERATION)
        {
          _nh.getParam(PARAM_COMPENSATION_NAME, _rosParam_compensation,NB_COMPENSATION_COMP); 
        }
        else
        {
          for (int c = 0; c < NB_COMPENSATION_COMP; c++) {
            _rosParam_compensation[c] = COMPENSATION_COMP[c];
          }
        }
        break;
      }
    }
    _nh.spinOnce();
    //_platformMutex.unlock();
  }
}

bool Platform::waitUntilRosConnect()
{
  if (!_nh.connected()) {
      //_platformMutex.lock();
      _nh.spinOnce();
      //_platformMutex.unlock();
      return false;
  }
  else
  {
    return true;
  }  
}