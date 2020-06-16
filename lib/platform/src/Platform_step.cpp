#include "Platform.h"
#include "definitions.h"
#include <string>


void Platform::step()
{
  memset(_logMsg, 0, sizeof(_logMsg)); //! Flush the char

  if ((_ros_state == RESET_UC) || ((_platform_state == RESET_UC)) || (_allEsconOk && _recoveringFromError))
  {
    sprintf(_logMsg, "%s : ABOUT TO RESTART THE PLATFORM CONTROLLER", Platform_Names[PLATFORM_ID]);
    _nh.loginfo(_logMsg);
    NVIC_SystemReset();
    _stop = true;
    wait_us(5000000);
    return;
  }

  getMotion(); //! SPI
  
  _allEsconOk=1;
  //for (uint k=0; k<NB_AXIS; k++) { _allEsconOk=  _esconEnabled[k]->read() * _allEsconOk;}

  if (!_allEsconOk || _flagEmergencyCalled)
  {
    _platform_state = EMERGENCY;
    _recoveringFromError=true;
  }

  switch (_platform_state)
  {

  case STANDBY:{ 
        if (!_enterStateOnceFlag[STANDBY]){
          // TODO
          sprintf(_logMsg, "%s : MOVING TO STATE STANDBY", Platform_Names[PLATFORM_ID]);
          _nh.loginfo(_logMsg);
          _enableMotors->write(0);
          _enterStateOnceFlag[STANDBY]=true;
          sprintf(_logMsg, "Timestep: %f milliseconds", ((float)_timestep * 1e-3));
          _nh.loginfo(_logMsg);
        }
        totalEffortDClear(-1);
        break;
        }    //Do nothing

    case HOMING:
        {
          // Init
          if(!_enterStateOnceFlag[HOMING])
          {           
            _platform_controllerType = SPEED_CTRL;
            for (uint k = 0; k < NB_AXIS; k++) {
              _pidSpeed[k]->reset();
            }
            loadDefaultPIDGains();
            speedCtrlLimitsSet();
            sprintf(_logMsg, "%s : MOVING TO STATE HOMING", Platform_Names[PLATFORM_ID]);
            _nh.loginfo(_logMsg);
            _enableMotors->write(1);
            limitSwitchesClear();
            // Set commanded forces and torques for homing
            _enterStateOnceFlag[HOMING]=true;

            _speedD[X] = SPEED_D_HOMING_X;         // m/s
            _speedD[Y] = SPEED_D_HOMING_Y;         // m/s
            _speedD[PITCH] = SPEED_D_HOMING_PITCH; // °/s
          }

          compEffortClear(-1, NORMAL);
          speedAllControl(NORMAL);

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
                
                positionAllReset();
                _platform_state = CENTERING;    
                _tic=false;
                clearLastState(); 
            }
          }

          break;
          }
        

    case CENTERING:
        {
          // Init State
          if (!_enterStateOnceFlag[CENTERING])
          {
            _platform_controllerType = POSITION_CTRL;
            for (uint k = 0; k < NB_AXIS; k++) {
              _pidPosition[k]->reset();  _posDesiredFilters[k].reset();   
            }
            loadDefaultPIDGains();
            posCtrlLimitsSet();
            sprintf(_logMsg, "%s : MOVING TO STATE CENTERING", Platform_Names[PLATFORM_ID]);
            _nh.loginfo(_logMsg);
            _enableMotors->write(1);
            _enterStateOnceFlag[CENTERING]=true;
          }
          // Main State

          compEffortClear(-1, NORMAL);
          gotoPointAll(0.0,0.0,0.0,0.0,0.0); //! Go to the center of the WS

          if((fabs(_positionD(X)-_position(X)) < 0.003f) && (fabs(_positionD(Y)-_position(Y)) < 0.003f) && (fabs(_positionD(PITCH)-_position(PITCH)) < 3.0f*DEG_TO_RAD))
          {
            if(!_tic){
              _toc = _innerTimer.read_us();
              _tic=true;
              }
          
            // After a second and a half move to next state
            if ((_innerTimer.read_us() - _toc) > 1500000)
            {
              _platform_state = TELEOPERATION;
              _tic=false;
              clearLastState();
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
          //
          _platform_controllerType = TORQUE_CTRL;
          for (uint k = 0; k < NB_AXIS; k++) {
            _pidPosition[k]->reset();  _posDesiredFilters[k].reset();   
            _pidSpeed[k]->reset();
          }
          loadDefaultPIDGains();
          posCtrlLimitsSet(); // for constrains
          sprintf(_logMsg, "%s : MOVING TO STATE TELEOPERATION", Platform_Names[PLATFORM_ID]);
          _nh.loginfo(_logMsg);
          _enterStateOnceFlag[TELEOPERATION]=true;
          _enableMotors->write(1);
        }

        //! Clear the vector of efforts
        compEffortClear(-1, CONSTRAINS);
        compEffortClear(-1, COMPENSATION);
        compEffortClear(-1, FEEDFORWARD);
        // Main State
        if (_flagInputReceived[MSG_TORQUE]) {
            for (uint k=0; k<NB_AXIS; k++) {
              _effortD_ADD(k,NORMAL) = _ros_effort[k];
            }
            _flagInputReceived[MSG_TORQUE] = false;
        }

        if (flagPositionInControl()) { //! Constrains / Virtual Walls
          if (_flagInputReceived[MSG_POSITION]) {
            for (uint k = 0; k < NB_AXIS; k++) {
              _positionD(k) = _ros_position[k];
            }
            _flagInputReceived[MSG_POSITION] = false;
          }
        }
      
          if (_platform_effortComp[CONSTRAINS] == 1) {
            if (flagPositionInControl()) {
              wsConstrains(_platform_controlledAxis); //! workspace constraints : soft
                                        //! limits, or joystick effect, etc
            }
          }

          if (_platform_effortComp[COMPENSATION] == 1) {
            _flagCalculateSinCos = true;
            int comp_[] = {1, 1, 1, 1,
                           1}; // gravity, viscous, inertia, coriolis, dry
            dynamicCompensation(comp_);
          } else {
            _flagCalculateSinCos = false;
          }

          break;
      }


    case ROBOT_STATE_CONTROL: //There is a bug here, please fix
    {
          // Init State
        
        if (!_enterStateOnceFlag[ROBOT_STATE_CONTROL])
        {
          _platform_controllerType=TORQUE_CTRL;
          for (uint k = 0; k<NB_AXIS; k++)
          {
              _pidPosition[k]->reset();  _posDesiredFilters[k].reset();   
              _pidSpeed[k]->reset();
          }
          loadDefaultPIDGains();
          posCtrlLimitsSet(); // for constrains
          speedCtrlLimitsSet(); // for constrains
          sprintf(_logMsg, "%s : MOVING TO STATE ROBOT_STATE_CONTROL",
                  Platform_Names[PLATFORM_ID]);
          _nh.loginfo(_logMsg);
          _enterStateOnceFlag[ROBOT_STATE_CONTROL] = true;
          _enableMotors->write(1);
        }

        // Main state
          totalEffortDClear(-1);

          if (_platform_effortComp[COMPENSATION] == 1) {
            _flagCalculateSinCos = true;
            int comp_[] = {1,1,1,1,0}; //gravity, viscous, inertia, coriolis, dry
            dynamicCompensation(comp_);
          } 
          else 
          {
            _flagCalculateSinCos = false;
          }

          if (flagPositionInControl()) {
            if (_flagInputReceived[MSG_POSITION]) {
              for (uint k = 0; k < NB_AXIS; k++) 
              {
                  _positionD(k)=_ros_position[k];
              }
                _flagInputReceived[MSG_POSITION] = false;
            }
          
            if (_platform_controlledAxis == -1) 
            {
              for (uint k=0; k<NB_AXIS; k++)
              {
                if (_workspaceLimitReached[k])
                {
                  _pidPosition[k]->reset();   
                }
                else
                {
                  gotoPointAxis(k, _positionD(k));
                }
              }
            } 
            
            else 
            {
              if (_workspaceLimitReached[_platform_controlledAxis])
              {
                _pidPosition[_platform_controlledAxis]->reset();
              }
              else
              {
                gotoPointAxis(_platform_controlledAxis, _positionD[_platform_controlledAxis]);
              }
              
            }
            
          }
          else if (flagSpeedInControl())
          {
            if (_flagInputReceived[MSG_SPEED]) {
              if (_platform_controlledAxis==-1)
              {
                for (uint k = 0; k < NB_AXIS; k++) {
                  _speedD(k) = _ros_speed[k];
                }
              }
              else {
                _speedD[_platform_controlledAxis] = _ros_speed[_platform_controlledAxis];
              }
                  _flagInputReceived[MSG_SPEED] = false;
            } 
            //! Add speed controller here!  
          }
          break;
    }
        
    case EMERGENCY:
    {
          _enableMotors->write(0);
          if(!_enterStateOnceFlag[EMERGENCY]){
            for (uint k = 0; k < NB_AXIS; k++) {
              _pidPosition[k]->reset();  _posDesiredFilters[k].reset();   
              _pidSpeed[k]->reset();
            }
            if(!_allEsconOk) {_nh.logerror("The servoamplifiers are not doing fine. Try restarting the microcontroller or rebooting the power supply");}
            sprintf(_logMsg,"%s MOVING TO STATE EMERGENCY",Platform_Names[PLATFORM_ID]);
            _nh.loginfo(_logMsg);
            _enterStateOnceFlag[EMERGENCY]=true;
          }
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
  
  if (_allEsconOk) {setEfforts();}// Aply the forces and torques}
  //readActualEffort();             //! Using the ESCON 50/5 Analog Output
  
  //! Keep track of variables

  //Platform State
  if (_flagClearLastState) {
    clearLastState();
    _platform_state = _ros_state;
    _flagClearLastState = false;
  }

  for (uint j = 0; j < NB_EFFORT_COMPONENTS;
       j++) // {NORMAL*, CONSTRAINS*, COMPENSATION, FEEDFORWARD}
  {
    _platform_effortComp[j] = _ros_effortComp[j];
    }
  
  //Control variables
  if (_flagControllerTypeChanged) {
    resetControllers();
    _flagControllerTypeChanged = false;
  }  
    _platform_controllerType = _ros_controllerType;
  if (_ros_flagDefaultControl && !_platform_flagDefaultControl) 
  {
      _flagDefaultCtrlNew = true;
  }
    _platform_flagDefaultControl = _ros_flagDefaultControl;

    _platform_controlledAxis = _ros_controlledAxis;
  
    if (_flagDefaultCtrlNew)
  {
        loadDefaultPIDGains();
        if (_platform_state == TELEOPERATION) {
          _virtualWall =
              (Eigen::Map<const Eigen::MatrixXf>(C_WS_LIMITS, NB_AXIS, 1));
        }
        _flagDefaultCtrlNew = false;
  }

  else if (_flagCtrlGainsNew)
  {
    loadROSPIDGains();
    _flagCtrlGainsNew=false;
  }

  _timestep = float(_innerTimer.read_us() - _timestamp);
  _timestamp=_innerTimer.read_us();
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
