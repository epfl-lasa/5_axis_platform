#include "Platform.h"
#include <string>


void Platform::step()
{
  if ((_platform_state == RESET_UC)) // || (_allEsconOk && _recoveringFromError))
  {
    _enableMotors->write(0);
    NVIC_SystemReset();
    _stop = true;
    rtos::ThisThread::sleep_for(5000);
    return;
  }

  getMotion(); //! SPI
  
  
  unsigned int esconCheck = 1;
  
  for (size_t k=0; k<NB_AXIS; k++) { 
    esconCheck=  _esconEnabled[k]->read() * esconCheck;
    }
  
  if ((_allEsconOk==1) && (esconCheck==0))
  {
    _recoveringFromError=true;
  }
  _allEsconOk=esconCheck;

  if (_platform_state != EMERGENCY)
  {
    if (_allEsconOk==0 || _flagEmergencyCalled)
    {
      clearLastState();
      _platform_state = EMERGENCY;
    }
  }
  

  if(!_flagRosConnected)
  {
    clearLastState();
    _platform_state = EMERGENCY;
  }

  switch (_platform_state)
  {

  case STANDBY:{ 
        if (!_enterStateOnceFlag[STANDBY]){
          resetEscons();
          _enterStateOnceFlag[STANDBY]=true;
        }
        totalEffortDClear(-1);
        break;
        }    //Do nothing

    case HOMING:
        {
          // Init
          if(!_enterStateOnceFlag[HOMING])
          { 
            _enableMotors->write(0);          
            _platform_controllerType = SPEED_CTRL;
            resetControllers(SPEED_CTRL);
            _platform_effortComp[NORMAL]=1;
            loadDefaultPIDGains();
            speedCtrlLimitsSet();
            limitSwitchesClear();
            _enterStateOnceFlag[HOMING]=true;

            _speedD[X] = SPEED_D_HOMING_X;         // m/s
            _speedD[Y] = SPEED_D_HOMING_Y;         // m/s
            _speedD[PITCH] = SPEED_D_HOMING_PITCH; // °/s
            _enableMotors->write(1);
          }

          compEffortClear(-1, NORMAL);
          speedAxisControl(NORMAL,-1);

          // Definition of the transition rule to the next state
          if ((_switchesState[X] == 1) && (_switchesState[Y] == 1) && (_switchesState[PITCH] == 1))
          {
            if(!_tic){
              _toc = _innerTimer.read_us();
              _tic=true;
              }
    
            //  After 1.5 second move to next state       
          
            if ((_innerTimer.read_us() - _toc) > 1500000)
            {
                
                positionAllReset(); // has to be before ClearlastState
                clearLastState(); 
                _tic=false;
                _toc=false;
                _platform_state = CENTERING;    
            }
          }

          break;
          }
        

    case CENTERING:
        {
          // Init State
          if (!_enterStateOnceFlag[CENTERING])
          {
            _enableMotors->write(0);
            _platform_controllerType = POSITION_CTRL;
            resetControllers(POSITION_CTRL);
            _platform_effortComp[NORMAL] = 1;
            loadDefaultPIDGains();
            posCtrlLimitsSet();
            _enterStateOnceFlag[CENTERING]=true;
            _enableMotors->write(1);
          }
          // Main State

          compEffortClear(-1, NORMAL);
          _positionD.setZero();
          positionAxisControl(NORMAL,-1);

          if((fabs(_positionD(X)-_position(X)) < 0.003f) && (fabs(_positionD(Y)-_position(Y)) < 0.003f) && (fabs(_positionD(PITCH)-_position(PITCH)) < 3.0f*DEG_TO_RAD))
          {
            if(!_tic){
              _toc = _innerTimer.read_us();
              _tic=true;
              }
          
            // After a second and a half move to next state
            if ((_innerTimer.read_us() - _toc) > 1500000)
            {
              clearLastState();
              _platform_state = TELEOPERATION;
              _tic=false;
              _toc=false;
            }
          }
          break;
        }
      case TELEOPERATION:
            //NB In this state, the controller type (set from ROS) POSITION_CTRL AND SPEED WILL REFER TO WANTING TO HAVE CONSTRAINS 
      {
          // Init State
        if (!_enterStateOnceFlag[TELEOPERATION])
        {
          _enableMotors->write(0);
          _platform_controllerType = TORQUE_CTRL;
          resetControllers(SPEED_CTRL);
          resetControllers(SOFT_LIMITS_CTRL);
          resetControllers(POSITION_CTRL);
          resetControllers(FS_CTRL);
          loadParamPIDGains();
          posCtrlLimitsSet(); // for constrains
          forceSensorCtrlLimitsSet();
          softLimitsCtrlLimitsSet();
          _enterStateOnceFlag[TELEOPERATION]=true;
          _flagOutofCompensation=true;
          _flagOutofSoftLimitsControl = true;
          _enableMotors->write(1);
          rtos::ThisThread::sleep_for(10);
          _enableMotors->write(1);
        }

        //! Clear the vector of efforts
        compEffortClear(-1, CUSTOM_IMPEDANCE);
        compEffortClear(-1, COMPENSATION);
        compEffortClear(-1, FEEDFORWARD);
        compEffortClear(-1, SOFT_LIMITS);
        // Main State
        updatePlatformFromRos();
        if (_platform_effortComp[CUSTOM_IMPEDANCE] == 1) {
          if (flagPositionInControl()) {
              positionAxisControl(CUSTOM_IMPEDANCE,_platform_controlledAxis);
          }
        }

          if (_platform_effortComp[COMPENSATION] == 1) {
            if (_flagOutofCompensation)
              {
              loadParamCompensation();
              _flagOutofCompensation = false;
            }
            dynamicCompensation();
          } else {
            if (!_flagOutofCompensation) {
             resetControllers(FS_CTRL);
             _flagOutofCompensation=true;
            }
          }

          if (_platform_effortComp[SOFT_LIMITS] == 1) {
            if (_flagOutofSoftLimitsControl) {
              loadParamPIDGains();  
              _flagOutofSoftLimitsControl = false;
            }
            wsConstrains(_platform_controlledAxis);
          } 
          else {
            if (!_flagOutofSoftLimitsControl) {
              resetControllers(SOFT_LIMITS_CTRL);
              _flagOutofSoftLimitsControl = true;
            }
          }

          break;
      }


    case ROBOT_STATE_CONTROL: //There is a bug here, please fix
    {
          // Init State
        if (!_enterStateOnceFlag[ROBOT_STATE_CONTROL])
        {
         _enableMotors->write(0);
         _platform_controllerType=POSITION_CTRL;
         resetControllers(POSITION_CTRL);
         resetControllers(SPEED_CTRL);
         loadParamPIDGains();
         posCtrlLimitsSet(); // for constrains
         speedCtrlLimitsSet(); // for constrains
         _enterStateOnceFlag[ROBOT_STATE_CONTROL] = true;
         _flagOutofCompensation=true;
         _enableMotors->write(1);
        }

        // Main state
          totalEffortDClear(-1);
          updatePlatformFromRos();
          positionAxisControl(NORMAL,-1);
        break;               
    }
        
    case EMERGENCY:
    {
          if(!_enterStateOnceFlag[EMERGENCY]){
            _enterStateOnceFlag[EMERGENCY]=true;
          }
          clearFlagsOtherStates();
          releasePlatform();
          break;
    }
    case RESET_UC:
    {
        
      //defined upstairs
      break;
    }
  }

  workspaceCheck(_platform_controlledAxis);

   if(_flagStateRequest)
  {
    updateStateInfo();
    _flagStateRequest=false;
  }
  
  if (_allEsconOk) {setEfforts();}// Aply the forces and torques}
  //readActualEffort();             //! Using the ESCON 50/5 Analog Output
  

  _timestep = float(_innerTimer.read_us() - _timestamp);
  _timestamp=_innerTimer.read_us();
  //_platformMutex.unlock();
}

bool Platform::flagTorqueInControl(){
   return _platform_controllerType == TORQUE_CTRL;
 }

bool Platform::flagPositionInControl() {
  return (_platform_controllerType == POSITION_CTRL);
}

bool Platform::flagSpeedInControl() {
  return (_platform_controllerType == SPEED_CTRL);
}

void Platform::updatePlatformFromRos(){
  if(_flagRosInputReceived){
    updateFootInputFromRos();
    _flagRosInputReceived=false;
  }
  
  if(_flagControllerRequest)
  {
    updateControllerRequest();
    _flagControllerRequest=false;
  }
}

void Platform::controlPositionWS(EffortComp Component, int axis)
{
  if (axis == -1) 
  {
    for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
    {
      controlPositionWS(Component,i);
    }
  } else{
      if (_workspaceLimitReached[axis])
      {
        _pidPosition[axis]->reset();   
      } else {
        positionAxisControl(Component, axis);
      }
  }
}