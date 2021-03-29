#include "Platform.h"
#include "macros_force_measurement.h"

void Platform::communicateToRos()
{
    pubFootOutput();
    _nh.spinOnce();
}

void Platform::initROS()
{ 
  _nh.initNode();
  rtos::ThisThread::sleep_for(10);
  _nh.advertise(*_pubFootOutput);
  _nh.advertiseService(*_servChangeState);
  _nh.advertiseService(*_servChangeCtrl);
  _nh.subscribe(*_subFootInput);
  _nh.negotiateTopics();
  rtos::ThisThread::sleep_for(10);
}


//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********
//! 1
void Platform::updateFootInput(const custom_msgs::FootInputMsg &msg)
{
  me->_msgFootInput = msg;
  me->_flagRosInputReceived=true;
}
//*****************ROS-SERVICES-CALLBACKS*************************/
void Platform::updateStateInfo()
{
  for (size_t j = NORMAL; j < NB_EFFORT_COMPONENTS;j++) 
  {
    _platform_effortComp[j] = _reqSrvState.ros_effortComp[j];
  }
  if (!(_platform_state == (State)_reqSrvState.ros_machineState)) // If I want to go to a new state
  { 
    clearLastState();
    _platform_state = (State)_reqSrvState.ros_machineState;
  }

}
//! 2
void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  me->_reqSrvState =  req;
  resp.platform_newState = req.ros_machineState != (uint8_t) me->_platform_state;
  me->_flagStateRequest=true; 
}


//! 3
void Platform::updateControllerRequest(){

  if ((_platform_state==TELEOPERATION) || (_platform_state==ROBOT_STATE_CONTROL))
  {
      if (_platform_controllerType != (Controller) _reqSrvController.ros_controllerType)
      {
        _flagControllerTypeChanged = true;
      }

      if (_flagControllerTypeChanged) {
        resetControllers(_platform_controllerType);
        _flagControllerTypeChanged = false;
      }
      _platform_controllerType = (Controller) _reqSrvController.ros_controllerType;

      if (_reqSrvController.ros_defaultControl && !_platform_flagDefaultControl) {
        _flagDefaultCtrlNew = true;
      }
      _platform_flagDefaultControl = _reqSrvController.ros_defaultControl;

      _platform_controlledAxis = _reqSrvController.ros_controlledAxis ==-1 ? -1:rosAxis[_reqSrvController.ros_controlledAxis]; ;

      if (_flagDefaultCtrlNew) {
        loadParamPIDGains();
        _flagDefaultCtrlNew = false;
      }
  }
}

void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
   me->_reqSrvController = req;
   me->_flagControllerRequest=true;
   resp.platform_controlOk = true;
}


//! 4
void Platform::pubFootOutput()
{
  _msgFootOutput.platform_stamp = _nh.now();
  _msgFootOutput.platform_id = PLATFORM_ID;
  
  for (size_t k=0; k<NB_PLATFORM_AXIS; k++)
  {
    _msgFootOutput.platform_position[rosAxis[k]] = _position(k)  * ConversionAxisPlatformToROS[k];
    _msgFootOutput.platform_speed[rosAxis[k]] = _speed(k) * ConversionAxisPlatformToROS[k];
    _msgFootOutput.platform_effortRef[rosAxis[k]] = _forceSensorD(k);
    _msgFootOutput.platform_effortD[rosAxis[k]] =_effortD(k);
    _msgFootOutput.platform_effortM[rosAxis[k]] =_effortM(k);
  }

  _msgFootOutput.platform_controllerType = (int8_t) _platform_controllerType;
  _msgFootOutput.platform_machineState = (int8_t) _platform_state;
  _pubFootOutput->publish(&_msgFootOutput);
}

void Platform::loadROSPIDGains()
{

  if (!_platform_flagDefaultControl && (_platform_state==TELEOPERATION || _platform_state==ROBOT_STATE_CONTROL))
    {
      for (size_t k=0; k<NB_PLATFORM_AXIS; k++)
      { 
        float scale=0.0f; 
        switch (_platform_controllerType)
        {
          case SPEED_CTRL:
            {
               scale=k<PITCH  ? SCALE_GAINS_LINEAR_SPEED :SCALE_GAINS_ANGULAR_SPEED; 
                _platform_kpSpeed[k]=_msgFootInput.ros_kp[rosAxis[k]] * scale;
                _platform_kiSpeed[k]=_msgFootInput.ros_ki[rosAxis[k]] * scale;
                _platform_kdSpeed[k]=0.0;
              break;
            }
          case POSITION_CTRL:
            {
               scale=k<PITCH  ? SCALE_GAINS_LINEAR_POSITION :SCALE_GAINS_ANGULAR_POSITION; 
                _platform_kpPosition[k]=_msgFootInput.ros_kp[rosAxis[k]] * scale;
                _platform_kiPosition[k]=_msgFootInput.ros_ki[rosAxis[k]] * scale;
                _platform_kdPosition[k]=_msgFootInput.ros_kd[rosAxis[k]] * scale;
                if (_platform_state==TELEOPERATION)
                { _platform_kiPosition[k]=0.0f;} // only SPRING-DAMPER: PD
              break;
            }
          default:
            break;
        }
      }
    }
  setPIDGains();
}

void Platform::updateFootInputFromRos() {

  calculateMeasTorques();
    // for (size_t c=0; c<NB_AXIS_WRENCH; c++)
    // {
    //   _ros_forceSensor[c] = _msgFootInput.ros_forceSensor[c]; //! In this one the rosAxis doesn't apply
    // }

  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
  {
    _platform_filterAxisFS(i) = _msgFootInput.ros_filterAxisForce[rosAxis[i]];
      if (_platform_state==TELEOPERATION || _platform_state==ROBOT_STATE_CONTROL)
      {
        switch (_platform_controllerType)
        {
        case SPEED_CTRL:
          {
            _speedD(i)=_msgFootInput.ros_speed[rosAxis[i]] * ConversionAxisROSToPlatform[i];
            break;
          }
        case POSITION_CTRL:
          {
            _positionD(i)=_msgFootInput.ros_position[rosAxis[i]] * ConversionAxisROSToPlatform[i];
            break;
          }
        default:
          break;
        }
      }
            
      if (_platform_state==TELEOPERATION)
      {
        _effortD_ADD(i,NORMAL) = _msgFootInput.ros_effort[rosAxis[i]];
      }

      loadROSPIDGains();
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

  // _effortM(Y) = effortM_Y;
  // _effortM(X) = effortM_X;
  // _effortM(PITCH) = -effortM_PITCH;
  // _effortM(ROLL) = -effortM_ROLL;
  // _effortM(YAW) = effortM_YAW;
  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
  {
    _effortM(i) = _msgFootInput.ros_effortM[rosAxis[i]];
  }

  _effortMNEG=-_effortM.cwiseProduct(_platform_filterAxisFS);
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
          if (!_nh.getParam(PARAM_P_POS_NAME, _rosParam_kpPosition, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_kpPosition[k] = POS_PID_GAINS_DEFAULT[KP][k];
            }
          }
          if (!_nh.getParam(PARAM_I_POS_NAME, _rosParam_kiPosition, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_kiPosition[k] = POS_PID_GAINS_DEFAULT[KI][k];
            }
          }
          if (!_nh.getParam(PARAM_D_POS_NAME, _rosParam_kdPosition, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_kdPosition[k] = POS_PID_GAINS_DEFAULT[KD][k];
            }
          }
        break;
      }

      case PID_SOFT_LIMITS_C:
      {
        
        if (!_nh.getParam(PARAM_WALL_POINT_MIN, _rosParam_pointSoftLimitsMin, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_pointSoftLimitsMin[k] = -C_WS_LIMITS[k];
            }
        } else
          {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_pointSoftLimitsMin[k]*= ConversionAxisROSToPlatform[k];
            }
          }
        if (!_nh.getParam(PARAM_WALL_POINT_MAX, _rosParam_pointSoftLimitsMax, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_pointSoftLimitsMax[k] = C_WS_LIMITS[k];
            }
        } else
          {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_pointSoftLimitsMax[k]*= ConversionAxisROSToPlatform[k];
            }
          }

        if (!_nh.getParam(PARAM_P_WALL_NAME, _rosParam_kpSoftLimits, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_kpSoftLimits[k] = C_WS_PID_GAINS[KP][k];
            }
          }
          if (!_nh.getParam(PARAM_I_WALL_NAME, _rosParam_kiSoftLimits, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_kiSoftLimits[k] = C_WS_PID_GAINS[KI][k];
            }
          }
          if (!_nh.getParam(PARAM_D_WALL_NAME, _rosParam_kdSoftLimits, NB_PLATFORM_AXIS)) {
            for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
              _rosParam_kdSoftLimits[k] = C_WS_PID_GAINS[KD][k];
            }
          }
        break;
      }

      case PID_VEL_C:
      {
        if (!_nh.getParam(PARAM_P_SPEED_NAME, _rosParam_kpSpeed, NB_PLATFORM_AXIS)) {
          for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
            _rosParam_kpSpeed[k] = SPEED_PID_GAINS_DEFAULT[KP][k];
          }
        }
        if (!_nh.getParam(PARAM_I_SPEED_NAME, _rosParam_kiSpeed, NB_PLATFORM_AXIS)) {
          for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
            _rosParam_kiSpeed[k] = SPEED_PID_GAINS_DEFAULT[KI][k];
          }
        }
        if (!_nh.getParam(PARAM_D_SPEED_NAME, _rosParam_kdSpeed, NB_PLATFORM_AXIS)) {
          for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
            _rosParam_kdSpeed[k] = SPEED_PID_GAINS_DEFAULT[KD][k];
          }
        }
        break;
      }
      case PID_FS_C:
      {
        if (!_nh.getParam(PARAM_P_FS_NAME, _rosParam_kpFS, NB_PLATFORM_AXIS)) {
          for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
            _rosParam_kpFS[k] = FS_PID_GAINS_DEFAULT[KP][k];
          }
        }
        if (!_nh.getParam(PARAM_I_FS_NAME, _rosParam_kiFS, NB_PLATFORM_AXIS)) {
          for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
            _rosParam_kiFS[k] = FS_PID_GAINS_DEFAULT[KI][k];
          }
        }
        if (!_nh.getParam(PARAM_D_FS_NAME, _rosParam_kdFS, NB_PLATFORM_AXIS)) {
          for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
            _rosParam_kdFS[k] = FS_PID_GAINS_DEFAULT[KD][k];
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
        _nh.spinOnce();
        _flagRosConnected = false;
    }
      else
    {
      _flagRosConnected =  true;
    }  
  return _flagRosConnected;
}