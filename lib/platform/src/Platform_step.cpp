#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>


void Platform::step()
{
  getMotion(); //! SPI
  readActualWrench(); //! Using the ESCON 50/5 Analog Output  

  //Security Check  
  if (!_allEsconOk) {_state=EMERGENCY;}
  _allEsconOk=1; //! In principle all the motor servo drives are doing fine until proved otherwise
  for (int k=0; k<NB_AXIS; k++) { _allEsconOk=  _esconEnabled[k]->read() * _allEsconOk;}
  //
  if(_flagClearLastState)
  {
    clearLastState();
    _state= _newState; 
    _flagClearLastState=false;
  }
  switch (_state)
  {

  case STANDBY:{ 
    if (!_enterStateOnceFlag[STANDBY]){
      // TODO
      _nh.loginfo("MOVING TO STATE STANDBY");
      _enterStateOnceFlag[STANDBY]=true;
    }
    totalWrenchDClear(-1);
    _lastState=_state; 
    break;
    }    //Do nothing

  case HOMING:
    {
      // Init
      if(!_enterStateOnceFlag[HOMING])
      {
        _nh.loginfo("MOVING TO STATE HOMING");
        _enableMotors->write(1);
        limitSwitchesClear();
        // Set commanded forces and torques for homing
        _enterStateOnceFlag[HOMING]=true;
      }

      _controllerType=TWIST_ONLY;
      
      _twistD[X] = TWIST_D_HOMING_X; // m/s
      _twistD[Y] = TWIST_D_HOMING_Y; // m/s
      _twistD[PITCH] = TWIST_D_HOMING_PITCH; // °/s

      _kpTwist[X] = KP_HOMING_TWIST_X;
      _kiTwist[X] = KI_HOMING_TWIST_X; 
      _kpTwist[Y] = KP_HOMING_TWIST_Y;
      _kiTwist[Y] = KI_HOMING_TWIST_Y; //
      _kpTwist[PITCH] = KP_HOMING_TWIST_PITCH; //
      _kiTwist[PITCH] = KI_HOMING_TWIST_PITCH; // 

      compWrenchClear(-1, NORMAL);
      twistControl(NORMAL);

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
             
            poseAllReset();
            _state = CENTERING;    
            _tic=false;
            clearLastState(); 
        }
      }

      _lastState=_state;
      break;
      }
    

    case CENTERING:
    {
      // Init State
      if (!_enterStateOnceFlag[CENTERING])
      {
        _nh.loginfo("MOVING TO STATE CENTERING");
        gotoPointGainsDefault(-1);
        _enterStateOnceFlag[CENTERING]=true;
      }
      // Main State
      compWrenchClear(-1, NORMAL);
      gotoPointAll(0.0,0.0,0.0,0.0,0.0); //! Go to the center of the WS

      if((fabs(_poseD[X]-_pose[X]) < 0.003f) && (fabs(_poseD[Y]-_pose[Y]) < 0.003f) && (fabs(_poseD[PITCH]-_pose[PITCH]) < 3.0f))
      {
        if(!_tic){
          _toc = _innerTimer.read_us();
          _tic=true;
          }
      
        // After a second and a half move to next state
        if ((_innerTimer.read_us() - _toc) > 1500000)
        {
          _state = TELEOPERATION;
          _tic=false;
          clearLastState();
        }
      }
      _lastState=_state;
      break;
    }
    case TELEOPERATION:
        //NB In this state, the controller type (set from ROS) POSE AND TWIST WILL REFER TO WANTING TO HAVE CONSTRAINS 
    {
      // Init State
     if (!_enterStateOnceFlag[TELEOPERATION])
     {
      //
      _nh.loginfo("MOVING TO STATE TELEOPERATION");
      _enterStateOnceFlag[TELEOPERATION]=true;
     }
      compWrenchClear(-1,CONSTRAINS);
      compWrenchClear(-1,COMPENSATION);
      compWrenchClear(-1,FEEDFORWARD);
      // Main State
      //# In this context: e.g. CtrlType=POSE_ONLY-> "I should listen" to set_positions[k], 
      if (_desWrenchComponents[CONSTRAINS]==1)
      {
        if ((_controllerType!=TWIST_ONLY)&&(_controllerType!=TORQUE_ONLY))
        {
          wsConstrains(_commControlledAxis); //! workspace constraints : soft limits, or joystick effect, etc
        }
        if ((_controllerType!=POSE_ONLY)&&(_controllerType!=TORQUE_ONLY))
        { 
          motionDamping(_commControlledAxis); //! Motion damping, to make it easier to control the platform
        }
      }
      _lastState=_state;
      break;
    }
    
    case ROBOT_STATE_CONTROL:
    {
       // Init State
     
     if (!_enterStateOnceFlag[ROBOT_STATE_CONTROL])
     {
      //
      _nh.loginfo("MOVING TO STATE ROBOT_STATE_CONTROL");
      _enterStateOnceFlag[ROBOT_STATE_CONTROL]=true;
     }

     // Main state
      compWrenchClear(-1,CONSTRAINS);
      compWrenchClear(-1,COMPENSATION);
      compWrenchClear(-1,FEEDFORWARD);
      for(int k=0; k<NB_AXIS; k++)
        {
          if(_controllerType!=TWIST_ONLY && _controllerType!=TORQUE_ONLY)
          {
            _poseD[k]=_commPoseSet[k];
                    //! Position Control
        
            if (_commControlledAxis==-1)
            {
              gotoPointAll(_poseD[X],_poseD[Y],_poseD[PITCH],_poseD[ROLL],_poseD[YAW]);
            }
            else 
            {
              gotoPointAxis(_commControlledAxis,_poseD[_commControlledAxis]);
            }
          }
          if(_controllerType!=POSE_ONLY && _controllerType!=TORQUE_ONLY)
          {
            _twistD[k]=_commTwistSet[k];
          }
        }

     _lastState=_state;
      break;
    }
    
    case EMERGENCY:
    {
      if(!_enterStateOnceFlag[EMERGENCY]){
        _nh.loginfo("MOVING TO STATE EMERGENCY");
        _enterStateOnceFlag[EMERGENCY]=true;
      }
      releasePlatform();
      _enableMotors->write(0);
      break;
    }
    case RESET:
    {
      _nh.loginfo("ABOUT TO RESTART THE PLATFORM CONTROLLER");
      softReset();
      break;
    }
  }

  if (_allEsconOk) {setWrenches();}// Aply the forces and torques}  
  
  //! Keep track of time
  _timestep=_innerTimer.read_us()-_timestamp;
  _timestamp=_innerTimer.read_us();
}