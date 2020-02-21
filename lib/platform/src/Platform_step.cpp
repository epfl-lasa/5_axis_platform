#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>
#include <string>

void Platform::step()
{
  memset(_logMsg, 0, sizeof(_logMsg)); //! Flush the char

  if ((_ros_state == RESET) || (_allEsconOk && _recoveringFromError))
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
  for (uint k=0; k<NB_AXIS; k++) { _allEsconOk=  _esconEnabled[k]->read() * _allEsconOk;}

  if (!_allEsconOk || _flagEmergencyCalled)
  {
    _ros_state = EMERGENCY;
    _recoveringFromError=true;
  }

  //
  if(_flagClearLastState)
  {
    clearLastState();
    _flagClearLastState=false;
  }

  if (_flagControllerTypeChanged)
  {
    resetControllers();
    _flagControllerTypeChanged=false;
  }

  switch (_ros_state)
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
        for (uint k = 0; k < NB_AXIS; k++) {
          _pidSpeed[k]->reset();
        }
        sprintf(_logMsg, "%s : MOVING TO STATE HOMING", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        _enableMotors->write(1);
        limitSwitchesClear();
        // Set commanded forces and torques for homing
        _enterStateOnceFlag[HOMING]=true;
      }

      _ros_controllerType=SPEED_ONLY;

      _speedD[X] = SPEED_D_HOMING_X; // m/s
      _speedD[Y] = SPEED_D_HOMING_Y; // m/s
      _speedD[PITCH] = SPEED_D_HOMING_PITCH; // °/s

      _kpSpeed[X] = KP_HOMING_SPEED_X;
      _kiSpeed[X] = KI_HOMING_SPEED_X; 
      _kpSpeed[Y] = KP_HOMING_SPEED_Y;
      _kiSpeed[Y] = KI_HOMING_SPEED_Y; //
      _kpSpeed[PITCH] = KP_HOMING_SPEED_PITCH; //
      _kiSpeed[PITCH] = KI_HOMING_SPEED_PITCH; // 

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
            _ros_state = CENTERING;    
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
        for (uint k = 0; k < NB_AXIS; k++) {
           _pidPosition[k]->reset();  _posDesiredFilters[k]->reset();   
        }
        sprintf(_logMsg, "%s : MOVING TO STATE CENTERING", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        _enableMotors->write(1);
        gotoPointGainsDefault(-1);
        _enterStateOnceFlag[CENTERING]=true;
      }
      // Main State
      _ros_controllerType = POSITION_ONLY;

      compEffortClear(-1, NORMAL);
      gotoPointAll(0.0,0.0,0.0,0.0,0.0); //! Go to the center of the WS

      if((fabs(_positionD[X]-_position[X]) < 0.003f) && (fabs(_positionD[Y]-_position[Y]) < 0.003f) && (fabs(_positionD[PITCH]-_position[PITCH]) < 3.0f))
      {
        if(!_tic){
          _toc = _innerTimer.read_us();
          _tic=true;
          }
      
        // After a second and a half move to next state
        if ((_innerTimer.read_us() - _toc) > 1500000)
        {
          _ros_state = TELEOPERATION;
          _tic=false;
          clearLastState();
        }
      }
      break;
    }
    case TELEOPERATION:
        //NB In this state, the controller type (set from ROS) POSITION AND SPEED WILL REFER TO WANTING TO HAVE CONSTRAINS 
    {
      // Init State
     if (!_enterStateOnceFlag[TELEOPERATION])
     {
      //
      for (uint k = 0; k < NB_AXIS; k++) {
        _pidPosition[k]->reset();  _posDesiredFilters[k]->reset();   
        _pidSpeed[k]->reset();
      }
      _ros_controllerType = TORQUE_ONLY;
      sprintf(_logMsg, "%s : MOVING TO STATE TELEOPERATION", Platform_Names[PLATFORM_ID]);
      _nh.loginfo(_logMsg);
      _enableMotors->write(1);
      _enterStateOnceFlag[TELEOPERATION]=true;
     }

    //! Clear the vector of efforts
    //  totalEffortDClear(-1);

     compEffortClear(-1, CONSTRAINS);
     compEffortClear(-1, COMPENSATION);
     compEffortClear(-1, FEEDFORWARD);

     if (_flagInputReceived[MSG_TORQUE]) {
         for (uint k=0; k<NB_AXIS; k++) {
           _effortD_ADD[NORMAL][k] = _ros_effort[k];
          //  _ros_effort_prev[k] = _ros_effort[k];
         }
         _flagInputReceived[MSG_TORQUE] = false;
     }

     // Main State
     //# In this context: e.g. CtrlType=POSITION_ONLY-> "I should listen" to
     //set_positions[k],

     if(_ros_effortComp[COMPENSATION] == 1)
     {
      gravityCompensation();
      frictionCompensation();
      //  _nh.loginfo("here");
     }

     if (_ros_effortComp[CONSTRAINS] == 1) {
      if (flagPositionInControl()) {
         wsConstrains(_ros_ControlledAxis); //! workspace constraints : soft
                                            //! limits, or joystick effect, etc
       }
      else if (flagSpeedInControl()) 
      {
         motionDamping(_ros_ControlledAxis); //! Motion damping, to make it
                                             //! easier to control the platform
       }
      }
      break;
    }
    
    case ROBOT_STATE_CONTROL:
    {
       // Init State
     
     if (!_enterStateOnceFlag[ROBOT_STATE_CONTROL])
     {
       _enableMotors->write(1);
       for (uint k = 0; k<NB_AXIS; k++)
       {
          _pidPosition[k]->reset();  _posDesiredFilters[k]->reset();   
         _pidSpeed[k]->reset();
       }
       _ros_controllerType=TORQUE_ONLY;
       //
        sprintf(_logMsg, "%s : MOVING TO STATE ROBOT_STATE_CONTROL", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        _enterStateOnceFlag[ROBOT_STATE_CONTROL] = true;
     }

     // Main state
      totalEffortDClear(-1);

      if (flagPositionInControl()) {
        if (_flagInputReceived[MSG_POSITION]) {
          if (_ros_ControlledAxis == -1)
          {   
            for (uint k = 0; k < NB_AXIS; k++) {
              _positionD[k]=_ros_position[k];
            }
          }
          else 
          {
            _positionD[_ros_ControlledAxis] = _ros_position[_ros_ControlledAxis];
          }
            _flagInputReceived[MSG_POSITION] = false;
        }
      
        if (_ros_ControlledAxis == -1) 
        {
          gotoPointAll(_positionD[X], _positionD[Y], _positionD[PITCH],
                       _positionD[ROLL], _positionD[YAW]);
          for (uint k=0; k<NB_AXIS; k++)
          {
             if (_workspaceLimitReached[k])
             {
               _pidPosition[k]->reset();  _posDesiredFilters[k]->reset();   
             }
            
          }
        } 
        
        else 
        {
          gotoPointAxis(_ros_ControlledAxis, _positionD[_ros_ControlledAxis]);
          if (_workspaceLimitReached[_ros_ControlledAxis])
          {
          _pidPosition[_ros_ControlledAxis]->reset();
          }
        }
        
      }
      else if (flagSpeedInControl())
      {
        if (_flagInputReceived[MSG_SPEED]) {
          if (_ros_ControlledAxis==-1)
          {
            for (uint k = 0; k < NB_AXIS; k++) {
              _speedD[k] = _ros_speed[k];
            }
          }
          else {
            _speedD[_ros_ControlledAxis] = _ros_speed[_ros_ControlledAxis];
          }
              _flagInputReceived[MSG_SPEED] = false;
        }
          
      }

      break;
    }
    
    case EMERGENCY:
    {
      _enableMotors->write(0);
      if(!_enterStateOnceFlag[EMERGENCY]){
        for (uint k = 0; k < NB_AXIS; k++) {
           _pidPosition[k]->reset();  _posDesiredFilters[k]->reset();   
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
    case RESET:
    {
      //defined upstairs
      break;
    }
  }

  workspaceCheck(_ros_ControlledAxis);
  
  if (_allEsconOk) {setEfforts();}// Aply the forces and torques}
  readActualEffort();             //! Using the ESCON 50/5 Analog Output
  
  //! Keep track of variables
  _platform_state = _ros_state;
  _platform_controllerType=_ros_controllerType;
  _timestep=_innerTimer.read_us()-_timestamp;
  _timestamp=_innerTimer.read_us();
}

bool Platform::flagTorqueInControl()
 {
   return _ros_controllerType == TORQUE_ONLY;
 }

bool Platform::flagPositionInControl() {
  return (_ros_controllerType != SPEED_ONLY) && !flagTorqueInControl();
}

bool Platform::flagSpeedInControl() {
  return (_ros_controllerType != POSITION_ONLY) && !flagTorqueInControl();
}
