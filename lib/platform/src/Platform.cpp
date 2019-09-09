#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

Platform *Platform::me = NULL;

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  float mapping = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  return mapping<-out_min ? -out_min : (mapping>out_max ? out_max : mapping);
}


Platform::Platform()
{
  me = this;

    _transmisions[X]=(1.0f/BELT_PULLEY_R)/cos(PI/3.0f); 
    _transmisions[Y]=1.0f/BELT_PULLEY_R; 
    _transmisions[PITCH]=PITCH_REDUCTION_R;
    _transmisions[ROLL]=ROLL_YAW_REDUCTION_R;
    _transmisions[YAW]=ROLL_YAW_REDUCTION_R;

    _maxCurrent[X]=MAX_CURRENT_X;
    _maxCurrent[Y]=MAX_CURRENT_Y;
    _maxCurrent[PITCH]=MAX_CURRENT_PITCH_ROLL_YAW;
    _maxCurrent[ROLL]=MAX_CURRENT_PITCH_ROLL_YAW;
    _maxCurrent[YAW]=MAX_CURRENT_PITCH_ROLL_YAW;

    _torqueConstants[X]=TORQUE_CONSTANT_X/1000; 
    _torqueConstants[Y]=TORQUE_CONSTANT_Y/1000;
    _torqueConstants[PITCH]=TORQUE_CONSTANT_PITCH_ROLL_YAW/1000;
    _torqueConstants[ROLL]=TORQUE_CONSTANT_PITCH_ROLL_YAW/1000;//[Nm/A]
    _torqueConstants[YAW]=TORQUE_CONSTANT_PITCH_ROLL_YAW/1000;
    
    for (int k=0; k<NB_AXIS; k++)
    {
      _maxWrench[k]=_torqueConstants[k]*_maxCurrent[k]*_transmisions[k];
    }

  _encoderScale[X] =ENCODERSCALE_X;
  _encoderScale[Y] =ENCODERSCALE_Y;
  _encoderScale[PITCH] =ENCODERSCALE_PITCH;
  _encoderScale[ROLL] =ENCODERSCALE_ROLL;
  _encoderScale[YAW] =ENCODERSCALE_YAW;

  _encoderSign[X] =ENCODERSIGN_X;
  _encoderSign[Y] =ENCODERSIGN_Y;
  _encoderSign[PITCH] =ENCODERSIGN_PITCH;
  _encoderSign[ROLL] =ENCODERSIGN_ROLL;
  _encoderSign[YAW] =ENCODERSIGN_YAW;

  _motorSign[X] =MOTORSIGN_X;
  _motorSign[Y] =MOTORSIGN_Y;
  _motorSign[PITCH] =MOTORSIGN_PITCH;
  _motorSign[ROLL] =MOTORSIGN_ROLL;
  _motorSign[YAW] =MOTORSIGN_YAW;

  _wsRange[X]=X_RANGE;
  _wsRange[Y]=Y_RANGE;
  _wsRange[PITCH]=PITCH_RANGE;
  _wsRange[ROLL]=ROLL_RANGE;
  _wsRange[YAW]=YAW_RANGE;

  for(int k = 0; k < NB_AXIS; k++)
  {
    _pose[k] = 0.0f;
    _poseOffsets[k] = 0.0f;
    _posePrev[k] = 0.0f;
    
    poseCtrlClear(k);
    twistCtrlClear(k);

    _wrench[k] = 0.0f;
    totalWrenchDClear(k);
    _wrenchM[k] = 0.0f;
    _poseFilters[k] = new LP_Filter(0.6);
    _twistFilters[k] = new LP_Filter(0.95);
    _wrenchMFilters[k] = new LP_Filter(0.95);
    limitSwitchesClear();

    _commPoseSet[k]=0.0f;
    _commTwistSet[k]=0.0f;
    
    _pidPose[k] = new PID(&_innerTimer, &_pose[k], &_poseCtrlOut[k], &_poseD[k], _kpPose[k], _kiPose[k], _kdPose[k],DIRECT);
    _pidPose[k]->setMode(AUTOMATIC);
    _pidTwist[k] = new PID(&_innerTimer, &_twist[k], &_twistCtrlOut[k], &_twistD[k], _kpTwist[k], _kiTwist[k], _kdTwist[k],DIRECT);
    _pidTwist[k]->setMode(AUTOMATIC);
  }  
  _innerCounter=0;
  
  _commControlledAxis=-1; //! all of them
  _controllerType=TORQUE_ONLY;
  _flagInWsConstrains=false;
  _flagDefaultControl=true;
   wsConstrainsDefault(-1);
  
  for (int j=0; j<NB_WRENCH_COMPONENTS; j++) // {NORMAL*, CONSTRAINS*, COMPENSATION, FEEDFORWARD}
  {
    if (j<=1){ _desWrenchComponents[j]=1;}
    else{_desWrenchComponents[j]=0;}
  }

  _tic=false;
  _state = HOMING;
  _lastState=_state;
    
  // Reset the flags that acknowledge when the state is entered for the first time 
  _enterStateOnceFlag[HOMING]=false;
  _enterStateOnceFlag[CENTERING]=false;
  _enterStateOnceFlag[TELEOPERATION]=false;
  _enterStateOnceFlag[STANDBY]=false;
  _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;

  _controllerType= TORQUE_ONLY;

#if (BOARD==NUCLEO64)
  /*******DESIGNATIONS OF PINS IN THE MICROCONTROLLER NUCLEO L476RG */

  _csPins[X] = PA_10;  //! CS1 -> Lateral NOT as PWMX/XN
  _csPins[Y] = PB_10;  //! CS2  -> Dorsi/Plantar Flexion NOT as PWMX/XN
  _csPins[PITCH] = PA_9; //! CS3 -> Flexion/Extension of the Leg NOT as PWM/XN
  _csPins[ROLL] = PB_6;  //! CS4 -> roll and yaw encoder 1 NOT as PWMX/XN
  _csPins[YAW] = PB_2;  //! CS5 -> roll and yaw encoder 2  NOT as PWMX/X
  
  _motorPins[X] = PC_7_ALT0; // D9 PWM8/2
  _motorPins[Y] = PB_3; // D3  PWM2/2
  _motorPins[PITCH] = PB_5; // D4 PWM3/2 
  _motorPins[ROLL] = PA_8; // D7 PWM1/1
  _motorPins[YAW] = PB_4; // D5  PWM3/1

  _limitSwitchesPins[X] = PC_8; //  NOT as PWMX/XN
  _limitSwitchesPins[Y] = PC_5; // NOT as PWMX/XN
  _limitSwitchesPins[PITCH] = PC_6; // NOT as PWMX/XN
  
  _esconEnabledPins[X] = PC_3;
  _esconEnabledPins[PITCH] = PC_2;
  _esconEnabledPins[Y] = PH_1;
  _esconEnabledPins[ROLL] = PH_0;
  _esconEnabledPins[YAW] = PC_13;


  _motorCurrentsPins[X]=PC_0;
  _motorCurrentsPins[PITCH]=PC_1;
  _motorCurrentsPins[Y]=PB_0;
  _motorCurrentsPins[ROLL]=PA_4;
  _motorCurrentsPins[YAW]=PA_0;

  _enableMotors= new DigitalOut(PB_1);  

  _spi = new SPI(PA_7, PA_6, PA_5); // mosi, miso, sclk https://os.mbed.com/platforms/ST-Nucleo-L476RG/
  // _spi->format(8,0); // Default
  // _spi->frequency(1000000); // Default
  /************************************************************* */
#endif

  wait_ms(10); //! Wait a bit after the SPI starts

  for(int k = 0; k < NB_AXIS; k++)
  {
    _encoders[k] = new QEC_1X(_csPins[k]);
    _motors[k] = new PwmOut(_motorPins[k]);  {
    _esconEnabled[k]=new InterruptIn(_esconEnabledPins[k]);
    _motorCurrents[k]= new AnalogIn(_motorCurrentsPins[k]);

  }
    if (k<NB_SWITCHES)
      {
        _limitSwitches[k] = new InterruptIn(_limitSwitchesPins[k]);
        _limitSwitches[k]->mode(PullUp);
      }
    else
    {
      _limitSwitches[k]=NULL;
    }
      
   }
_timestamp=0; // We don't read the timer until the platform is initialized
}


Platform::~Platform()
{
  _innerTimer.~Timer();
  for(int k = 0; k <NB_AXIS; k++)
  {
    delete (_poseFilters[k]);
    delete (_wrenchMFilters[k]);
    delete (_twistFilters[k]);
    _pidPose[k]->~PID();
    delete (_pidPose[k]);
    _pidTwist[k]->~PID();
    delete (_pidTwist[k]);
    _encoders[k]->~QEC_1X();
    delete (_encoders[k]);
    delete (_motors[k]);
    delete (_limitSwitches[k]);
    delete (_motorCurrents[k]);
    delete (_esconEnabled[k]);
  }
  delete(_enableMotors);
  delete (_spi); 
  delete (_pubFootOutput);
}


void Platform::init()
{
    
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
    for(int k = 0; k <NB_AXIS; k++)
    {

     _motors[k]->period_us(200); // PWM (to ESCON) PERIOD 200 us-> 1kHz    
     _esconEnabled[k]->fall(&emergencyCallback);

    if (k<2){
     _pidPose[k]->setOutputLimits(-25.0, 25.0);
     _pidTwist[k]->setOutputLimits(-25.0, 25.0);  
    }
     else {
     _pidPose[k]->setOutputLimits(-3.0, 3.0); //! For the moment 
     _pidTwist[k]->setOutputLimits(-3.0, 3.0);    
      }

    //}
    _pidPose[k]->setSampleTime(POSE_PID_SAMPLE_P*1e-6); //! [us]
    _pidTwist[k]->setSampleTime(VELOCITY_PID_SAMPLE_P*1e-6);
    }

  //! Attach interruptions to callbacks on falling edge 
  _limitSwitches[X]->fall(&switchCallbackX);
  _limitSwitches[Y]->fall(&switchCallbackY);
  _limitSwitches[PITCH]->fall(&switchCallbackPitch);
  
  _spi->lock();
  for (int k = 0; k<NB_AXIS; k++)
  {
     _encoders[k]->QEC_init((int)k, _encoderScale[k], _encoderSign[k],_spi);
  }
  _spi->unlock(); 

  _subFootInput = new ros::Subscriber<custom_msgs::FootInputMsg_v2>(SUBSCRIBER_NAME, updateFootInput);
  _servChangeState = new ros::ServiceServer<custom_msgs::setStateSrv::Request,custom_msgs::setStateSrv::Response>(SERVICE_CHANGE_STATE_NAME, updateState);
  _servChangeCtrl = new ros::ServiceServer<custom_msgs::setControllerSrv::Request,custom_msgs::setControllerSrv::Response>(SERVICE_CHANGE_CTRL_NAME, updateController);
  
  _pubFootOutput = new ros::Publisher(PUBLISHER_NAME, &_msgFootOutput);


  _nh.getHardware()->setBaud(BAUDRATE);
  _nh.initNode();
   wait_ms(10);
  _nh.advertise(*_pubFootOutput);
  _nh.advertiseService(*_servChangeState);
  _nh.advertiseService(*_servChangeCtrl);
  _nh.subscribe(*_subFootInput);
  wait_ms(10);
  _innerTimer.start(); // Start Running the Timer -> I moved it to the constructor
  _timestamp = _innerTimer.read_us();
  _speedSamplingStamp=_timestamp;
  _analogReadStamp=_timestamp;
  _enableMotors->write(0);
  _allEsconOk=0;
  resetEscons();
}

void Platform::step()
{
  getMotion(); //! SPI
  readActualWrench(); //! Using the ESCON 50/5 Analog Output  

  //Security Check  
  if (!_allEsconOk) {_state=EMERGENCY;}
  _allEsconOk=1; //! In principle all the motor servo drives are doing fine until proved otherwise
  for (int k=0; k<NB_AXIS; k++) { _allEsconOk=  _esconEnabled[k]->read() * _allEsconOk;}
  //

  switch (_state)
  {

  case STANDBY:{ 
    if (!_enterStateOnceFlag[STANDBY]){
      // TODO
      _enterStateOnceFlag[STANDBY]=true;
    }
    _lastState=_state; 
    break;
    }    //Do nothing

  case HOMING:
    {
      // Init
      if(!_enterStateOnceFlag[HOMING])
      {
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
        gotoPointGainsDefault(-1);
        _enterStateOnceFlag[CENTERING]=true;
      }
      // Main State
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
      _enterStateOnceFlag[TELEOPERATION]=true;
     }

      // Main State
      //# In this context: e.g. CtrlType=POSE_ONLY-> "I should listen" to set_positions[k], 

      if (_desWrenchComponents[CONSTRAINS]==1){ //! Check which components of the wrench I wanna activate
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
        _enterStateOnceFlag[ROBOT_STATE_CONTROL]=true;
     }

     // Main state

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
        _enterStateOnceFlag[EMERGENCY]=true;
      }
      releasePlatform();
      _enableMotors->write(0);
      break;
    }
    case RESET:
    {
      softReset();
      break;
    }
  }

  if (_allEsconOk) {setWrenches();}// Aply the forces and torques}  
  
  //! Keep track of time
  _timestep=_innerTimer.read_us()-_timestamp;
  _timestamp=_innerTimer.read_us();
}

void Platform::getMotion()
{
  getPose();
  getTwist();
}

void Platform::readActualWrench() //! ADC
{
  if(_innerCounter<NB_AXIS && (_timestamp-_analogReadStamp)>=ANALOG_SAMPLING_TIME)
  {
    if (_innerCounter>=ROLL){
      _wrenchM[_innerCounter+2]=map(_motorCurrents[_innerCounter]->read()*_motorSign[_innerCounter],0.1,0.9,-_maxWrench[_innerCounter],_maxWrench[_innerCounter]);
    }
    else
    {
      _wrenchM[_innerCounter]=map(_motorCurrents[_innerCounter]->read()*_motorSign[_innerCounter],0.1,0.9,-_maxWrench[_innerCounter],_maxWrench[_innerCounter]);
      _wrenchM[_innerCounter]=_wrenchMFilters[_innerCounter]->update(_wrenchM[_innerCounter]);
    }
    
  if(_innerCounter==YAW){
    // Adapt roll and yaw angles due to differential mechanism
    _wrenchM[ROLL]= (_wrenchM[ROLL+2]-_wrenchM[YAW+2])/2.0f;
    _wrenchM[YAW] = (_wrenchM[ROLL+2]+_wrenchM[YAW+2])/2.0f;
    _wrenchM[ROLL]=_wrenchMFilters[ROLL]->update(_wrenchM[ROLL]);
    _wrenchM[YAW]=_wrenchMFilters[YAW]->update(_wrenchM[YAW]);
    _innerCounter=0;
  }
  
  _innerCounter++;
  }

}

void Platform::communicateToRos()
{
  // Publish foot output
   pubFootOutput();
   _nh.spinOnce(); //Publishes and Retrieves Messages
   // For Retrieving and Publishing to ROS. We can put it separate in the main in  case we want to put it in an interruption     
}

void Platform::getPose()
{
  _spi->lock(); 
  for (int k = 0; k < NB_AXIS; k++)
  {
    _encoders[k]->QEC_getPose(_spi);
    _pose[k] = _encoders[k]->outDimension + _poseOffsets[k];
    _pose[k] = _poseFilters[k]->update(_pose[k]);
  }
  _spi->unlock(); 
  // Adapt roll and yaw angles due to differential mechanism
  float enc1 = _pose[ROLL];
  float enc2 = _pose[YAW];
  _pose[ROLL]= (enc1-enc2)/2.0f;
  _pose[YAW] = (enc1+enc2)/2.0f;
}


void Platform::getTwist()
{
  if ((_timestamp-_speedSamplingStamp)>=VELOCITY_PID_SAMPLE_P)
  {
    for (int k = 0; k < NB_AXIS; k++)
    {
      _twist[k] = (_pose[k] - _posePrev[k]) / (VELOCITY_PID_SAMPLE_P * 1e-6f);
      _twist[k] = _twistFilters[k]->update(_twist[k]);
      _posePrev[k] = _pose[k];
      _speedSamplingStamp=_timestamp;
    }
  }
}

  void Platform::posAxisControl(WrenchComp Component, int axis)
{

    if ((axis<2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      _pidPose[axis]->setOutputLimits(-25.0,25.0); //!N
    }

    if ((axis>=2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      if (axis==PITCH) {_pidPose[axis]->setOutputLimits(-5.0,5.0);} //!Nm
      if (axis==ROLL || axis==YAW){_pidPose[axis]->setOutputLimits(-5.0,5.0);} //!Nm
    }

    if (_controllerType==TWIST_POSE_CASCADE){
      _poseD[axis]=_twistCtrlOut[axis];
    }

     _pidPose[axis]->setTunings(_kpPose[axis], _kiPose[axis], _kdPose[axis]);
     _pidPose[axis]->compute();

    if ((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE)){
      _wrenchD_ADD[Component][axis]=_poseCtrlOut[axis];
    }
}

void Platform::poseControl(WrenchComp Component)
{
  //Compute the PID

  for (int i = 0; i < NB_AXIS; i++)
  { 
    posAxisControl(Component,i);
  }
 
}

void Platform::speedAxisControl(WrenchComp Component, int axis)
{
  if ((axis<2)&&((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE))){
      _pidTwist[axis]->setOutputLimits(-25,25); //!N
    }

    if ((axis>=2)&&((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE))){
      _pidTwist[axis]->setOutputLimits(-3,3); //!Nm
    }

    if (_controllerType==POSE_TWIST_CASCADE){
      _poseD[axis]=_twistCtrlOut[axis];
    }

     _pidTwist[axis]->setTunings(_kpTwist[axis], _kiTwist[axis], _kdTwist[axis]);
     _pidTwist[axis]->compute();

    if ((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE)){
      _wrenchD_ADD[NORMAL][axis]=_twistCtrlOut[axis];
    }
}

void Platform::twistControl(WrenchComp Component)
{

  for (int i = 0; i < NB_AXIS; i++)
  {  
    speedAxisControl(Component,i);
  }
}


void Platform::setWrenches()
{
  float wrenchSum;

  for(int k = 0; k < NB_AXIS; k++)
  { 
     wrenchSum = 0.0f;
      for(int j = 0; j < NB_WRENCH_COMPONENTS; j++)
      {
        if (_desWrenchComponents[j]==1) {wrenchSum+= _wrenchD_ADD[j][k];}
      }
    _wrenchD[k]=wrenchSum;
  } 
  

  setWrenchAxis(_wrenchD[X],_motors[X],_motorSign[X],(int)X);
  setWrenchAxis(_wrenchD[Y],_motors[Y],_motorSign[Y],(int)Y);
  setWrenchAxis(_wrenchD[PITCH],_motors[PITCH],_motorSign[PITCH],(int)PITCH);
  // Adapt roll and yaw commands due to differential mechanism
  setWrenchAxis((_wrenchD[YAW]+_wrenchD[ROLL])/2.0f, _motors[ROLL], _motorSign[ROLL], (int)ROLL);
  setWrenchAxis((_wrenchD[YAW]-_wrenchD[ROLL])/2.0f, _motors[YAW], _motorSign[YAW], (int)YAW);

}


void Platform::setWrenchAxis(float wrench, PwmOut *pin, int sign, int axis)
{
  float escon_torque = wrench/_transmisions[axis];
  setCurrentAxis(escon_torque, pin, sign, axis);
}

void Platform::setCurrentAxis(float torque, PwmOut *pin, int sign, int axis)
{
  float escon_current = torque/_torqueConstants[axis];
  float escon_current_PWM = map(sign*escon_current, -_maxCurrent[axis], _maxCurrent[axis], 0.1f, 0.9f); //! from 10 to 90% of Duty Cycle to acknowledge connection
  pin->write(escon_current_PWM);
}


void Platform::switchCallbackX()
{
  if ((me->_switchesState[X] == 0) )
  {
    me->_switchesState[X] = 1;
  }
}


void Platform::switchCallbackY()
{
  if ((me->_switchesState[Y] == 0) )
  {
    me->_switchesState[Y] = 1;
  }
}


void Platform::switchCallbackPitch()
{
  if ((me->_switchesState[PITCH] == 0) )
  {
    me->_switchesState[PITCH] = 1;
  }
}


void Platform::poseAllReset()
{
  if (_switchesState[X] == 1 && _switchesState[Y] == 1 && _switchesState[PITCH] == 1)
  {
    _poseOffsets[X] = HOMING_OFFSET_X;
    _poseOffsets[Y] = HOMING_OFFSET_Y;
    _poseOffsets[PITCH] = HOMING_OFFSET_PITCH;
    _spi->lock();
      for(int k = 0; k <NB_AXIS; k++)
      {
      _encoders[k]->QEC_clear(_spi);
      }
    _spi->unlock();
  }
}

//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********

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

void Platform::updateState(const custom_msgs::setStateSrv::Request &req, custom_msgs::setStateSrvResponse &resp )
{
  Platform::State newState = (Platform::State) req.machine_state;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc
  for (int j=0; j<NB_WRENCH_COMPONENTS; j++){
      me->_desWrenchComponents[j]=req.wrench_comp[j];
  }  
  
  if (!(newState==me->_state)) // If I want to go to a new state
  { resp.mS_new=true;
    me->clearLastState();
    me->_state = newState;
  } 
  else{ resp.mS_new=false; } //! You are already in the desired state 
}

void Platform::updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp )
{
  me->_controllerType=(Platform::Controller) req.set_ctrlType; 
  me->_flagDefaultControl=req.default_ctrl;
  
  if ((me->_state!=TELEOPERATION) && (me->_state!=ROBOT_STATE_CONTROL))
    { resp.sC_ok=false; }
  else
  {
    resp.sC_ok=true;
    if (me->_commControlledAxis<=-1){ //! Every Axis

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
            me->_kpPose[me->_commControlledAxis]=req.pos_p[me->_commControlledAxis];
            me->_kiPose[me->_commControlledAxis]=req.pos_i[me->_commControlledAxis];
            me->_kdPose[me->_commControlledAxis]=req.pos_d[me->_commControlledAxis]; 
          }
          if ((req.set_ctrlType!=POSE_ONLY) && (req.set_ctrlType!=TORQUE_ONLY))
          {
            me->_kpTwist[me->_commControlledAxis]=req.speed_p[me->_commControlledAxis];
            me->_kiTwist[me->_commControlledAxis]=req.speed_i[me->_commControlledAxis];
            me->_kdTwist[me->_commControlledAxis]=req.speed_d[me->_commControlledAxis]; 
          }
      }
    }
}


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
  _pubFootOutput->publish(&_msgFootOutput);
}

void Platform::gotoPointAxis(int axis_, float point)
{
  if (_flagDefaultControl){ gotoPointGainsDefault(axis_);}
  _poseD[axis_]=point;
  posAxisControl(NORMAL,axis_);
}

void Platform::gotoPointAll(float pointX, float pointY, float pointPITCH, float pointROLL, float pointYAW)
{
  _controllerType=POSE_ONLY;
  _poseD[X]=pointX;
  _poseD[Y]=pointY;
  _poseD[PITCH]=pointPITCH;
  _poseD[ROLL]=pointROLL;
  _poseD[YAW]=pointYAW;
  gotoPointGainsDefault(-1);
  poseControl(NORMAL);
}

void Platform::wsConstrains(int axis_)
{
  if (axis_==-1) //! All axis constrained
  {
    for (int k = 0; k<NB_AXIS; k++){
      wsConstrains(k);
    }
  }
  else
  {
    if (_flagDefaultControl) {wsConstrainsDefault(axis_);}
    else
    {
       _c_wsLimits[axis_]=fabs(_commPoseSet[axis_]); //! A symmetric wall will be built on the set position
       _kiPose[axis_]=0.0f;
    }   
    _poseD[axis_] = _pose[axis_] >= _c_wsLimits[axis_] ? _c_wsLimits[axis_] : (_pose[axis_] <= -_c_wsLimits[axis_] ? -_c_wsLimits[axis_]: 0.0f);
    if ( _pose[axis_] >= _c_wsLimits[axis_] || _pose[axis_] <= -_c_wsLimits[axis_] )
      {
        _flagInWsConstrains=true;
        if ( ((_pose[axis_] <= 0.0) && (_twist[axis_] <= 0.0)) || ((_pose[axis_] >= 0.0) && (_twist[axis_] >= 0.0)) ) 
        {
          posAxisControl(CONSTRAINS,axis_);
        }
      }
      else    
      {
        _flagInWsConstrains=false;
        _wrenchD_ADD[CONSTRAINS][axis_]=0.0f;
      }
  }
}

void Platform::motionDamping(int axis_)
{ 
  if (axis_==-1) //! All axis constrained
  {
    for (int k = 0; k<NB_AXIS; k++){
      motionDamping(k);
    }
    return;
  }
  else
  {

    if (!_flagInWsConstrains) //! The motion damping only applies outside the walls (constrains)
    {
      if (_flagDefaultControl) {motionDampingGainsDefault(axis_);}
      else
      {
        _kiTwist[axis_]=0.0f; //! overwrite the i and d constants sent from ROS (because it is teleoperation mode)
        _kdTwist[axis_]=0.0f;
      }
      _twistD[axis_]=0.0f; //! the target is zero speed
      speedAxisControl(CONSTRAINS, axis_);
    }   
  }
}

void Platform::emergencyCallback()
{
  me->_allEsconOk=0;
  me->_state=EMERGENCY; 
}

void Platform::releasePlatform()
{
  for(int k = 0; k < NB_AXIS; k++)
  {
    // _pidPose[k]->reset();
    // _pidTwist[k]->reset();
    _poseFilters[k]->reset();
    _twistFilters[k]->reset();
    _wrenchMFilters[k]->reset();
    
     for(int j = 0; j < NB_WRENCH_COMPONENTS; j++)
    {
      _wrenchD_ADD[j][k] = 0.0f;
    }
  }
}

void Platform::limitSwitchesClear()
{
  for (int k=0; k<NB_SWITCHES; k++) 
  {
    _switchesState[k] = 0;
  }
}
void Platform::clearLastState()
{
  switch(_lastState)
    {
      case(HOMING):

      {
        _enterStateOnceFlag[HOMING]=false;   
        limitSwitchesClear();       
        //! Finally resets the wrench commands given by this controller. 
        compWrenchClear(-1,NORMAL); //! Clear the normal dimension of the 
        twistCtrlClear(-1);
        break; 
      }

      case(CENTERING):
      {
        _enterStateOnceFlag[CENTERING]=false;
        compWrenchClear(-1,NORMAL);
        poseCtrlClear(-1);
        break;
      }
      case(TELEOPERATION):
      {
        _enterStateOnceFlag[TELEOPERATION]=false;
        totalWrenchDClear(-1);
        poseCtrlClear(-1);
        twistCtrlClear(-1);
        break;
      }

      case(ROBOT_STATE_CONTROL):
      {
        _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;
        totalWrenchDClear(-1);
        poseCtrlClear(-1);
        twistCtrlClear(-1);
        break;
      }


      case(EMERGENCY):{break;}
      case(STANDBY): {_enterStateOnceFlag[STANDBY]=false; break;} 
      
      case(RESET):{break;}
    }
}

void Platform::gotoPointGainsDefault(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      gotoPointGainsDefault(k);
    }
  }

  else{
    switch(axis_){
        case(X): {_kpPose[X]=GT_KP_POSE_X;_kiPose[X]=GT_KI_POSE_X;_kdPose[X]=GT_KD_POSE_X; break;}
        case(Y): {_kpPose[Y]=GT_KP_POSE_Y;_kiPose[Y]=GT_KI_POSE_Y;_kdPose[Y]=GT_KD_POSE_Y; break;}
        case(PITCH): {_kpPose[PITCH]=GT_KP_POSE_PITCH;_kiPose[PITCH]=GT_KI_POSE_PITCH;_kdPose[PITCH]=GT_KD_POSE_PITCH;break;}
        case(ROLL): {_kpPose[ROLL]=GT_KP_POSE_ROLL;_kiPose[ROLL]=GT_KI_POSE_ROLL;_kdPose[ROLL]=GT_KD_POSE_ROLL;break;}
        case(YAW): {_kpPose[YAW]=GT_KP_POSE_YAW;_kiPose[YAW]=GT_KI_POSE_YAW;_kdPose[YAW]=GT_KD_POSE_YAW;break;}
      }
  }
}

void Platform::wsConstrainsDefault(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      wsConstrainsDefault(k);
    }
  }
  else{
    switch(axis_){
        case(X): {_c_wsLimits[X] = C_WS_RANGE_X;_kpPose[X]=C_WS_KP_POSE_X;_kiPose[X]=C_WS_KI_POSE_X;_kdPose[X]=C_WS_KD_POSE_X; break;}
        case(Y): {_c_wsLimits[Y] = C_WS_RANGE_Y;_kpPose[Y]=C_WS_KP_POSE_Y;_kiPose[Y]=C_WS_KI_POSE_Y;_kdPose[Y]=C_WS_KD_POSE_Y; break;}
        case(PITCH): {_c_wsLimits[PITCH] = C_WS_RANGE_PITCH;_kpPose[PITCH]=C_WS_KP_POSE_PITCH;_kiPose[PITCH]=C_WS_KI_POSE_PITCH;_kdPose[PITCH]=C_WS_KD_POSE_PITCH;break;}
        case(ROLL): {_c_wsLimits[ROLL] = C_WS_RANGE_ROLL;_kpPose[ROLL]=C_WS_KP_POSE_ROLL;_kiPose[ROLL]=C_WS_KI_POSE_ROLL;_kdPose[ROLL]=C_WS_KD_POSE_ROLL;break;}
        case(YAW): {_c_wsLimits[YAW] = C_WS_RANGE_YAW;_kpPose[YAW]=C_WS_KP_POSE_YAW;_kiPose[YAW]=C_WS_KI_POSE_YAW;_kdPose[YAW]=C_WS_KD_POSE_YAW;break;}
      }
  }
}

void Platform:: motionDampingGainsDefault(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      motionDampingGainsDefault(k);
    }
    return;
  }

  else{
    switch(axis_){
        case(X): {_kpTwist[X]=MOTION_DAMPING_KP_TWIST_X;_kiTwist[X]=MOTION_DAMPING_KI_TWIST_X;_kdTwist[X]=MOTION_DAMPING_KD_TWIST_X; break;}
        case(Y): {_kpTwist[Y]=MOTION_DAMPING_KP_TWIST_Y;_kiTwist[Y]=MOTION_DAMPING_KI_TWIST_Y;_kdTwist[Y]=MOTION_DAMPING_KD_TWIST_Y; break;}
        case(PITCH): {_kpTwist[PITCH]=MOTION_DAMPING_KP_TWIST_PITCH;_kiTwist[PITCH]=MOTION_DAMPING_KI_TWIST_PITCH;_kdTwist[PITCH]=MOTION_DAMPING_KD_TWIST_PITCH;break;}
        case(ROLL): {_kpTwist[ROLL]=MOTION_DAMPING_KP_TWIST_ROLL;_kiTwist[ROLL]=MOTION_DAMPING_KI_TWIST_ROLL;_kdTwist[ROLL]=MOTION_DAMPING_KD_TWIST_ROLL;break;}
        case(YAW): {_kpTwist[YAW]=MOTION_DAMPING_KP_TWIST_YAW;_kiTwist[YAW]=MOTION_DAMPING_KI_TWIST_YAW;_kdTwist[YAW]=MOTION_DAMPING_KD_TWIST_YAW;break;}
      }
  }
}

void Platform::poseCtrlClear(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      poseCtrlClear(k);
    }
  }
  else
  {
    _poseD[axis_]=0.0f;
    _kpPose[axis_]=0.0f;
    _kiPose[axis_]=0.0f;
    _kdPose[axis_]=0.0f;
    _poseCtrlOut[axis_]=0.0f;
  }
  
}

void Platform::twistCtrlClear(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      twistCtrlClear(k);
    }
  }
  else
  {
    _twistD[axis_]=0.0f;
    _kpTwist[axis_]=0.0f;
    _kiTwist[axis_]=0.0f;
    _kdTwist[axis_]=0.0f;
    _twistCtrlOut[axis_]=0.0f;
  }
  
}

void Platform::compWrenchClear(int axis_, Platform::WrenchComp component_)
{
  if (axis_==-1)
  {
    for (int k=0; k<NB_AXIS; k++) 
    { 
      compWrenchClear(k, component_); 
    }
    return;
  }
  else
  {
    _wrenchD_ADD[component_][axis_]=0.0f;
  }
}

void Platform::totalWrenchDClear(int axis_)
{
  if (axis_==-1)
    {
      for (int k=0; k<NB_AXIS; k++) 
      { 
        totalWrenchDClear(k); 
        _wrenchD[k] = 0.0f;
      }
    }
  else
  {  
    for (int j=0; j<NB_WRENCH_COMPONENTS; j++)
    {
      compWrenchClear(axis_, (Platform::WrenchComp) j);
    }
  }
}

void Platform::resetEscons(){

  totalWrenchDClear(-1);
  setWrenches();
  wait_ms(150);
  _enableMotors->write(1);
  wait_ms(750);
  _enableMotors->write(0);
}

void Platform::softReset(){
   NVIC_SystemReset(); 
}