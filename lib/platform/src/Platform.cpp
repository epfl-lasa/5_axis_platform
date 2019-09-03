#include "Platform.h"
#include "definitions.h"
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

  _c_wsLimits[X] = C_WS_RANGE_X;
  _c_wsLimits[Y] = C_WS_RANGE_Y;
  _c_wsLimits[PITCH] = C_WS_RANGE_PITCH;
  _c_wsLimits[ROLL] = C_WS_RANGE_ROLL;
  _c_wsLimits[YAW] = C_WS_RANGE_YAW;

  _wsRange[X]=X_RANGE;
  _wsRange[Y]=Y_RANGE;
  _wsRange[PITCH]=PITCH_RANGE;
  _wsRange[ROLL]=ROLL_RANGE;
  _wsRange[YAW]=YAW_RANGE;

for(int k = 0; k < NB_AXIS; k++)
{
  for(int j = 0; j < WRENCH_COMPONENTS; j++)
  {
    _wrenchD_ADD[j][k] = 0.0f;
  }
}

  for(int k = 0; k < NB_AXIS; k++)
  {
    _pose[k] = 0.0f;
    _poseOffsets[k] = 0.0f;
    _posePrev[k] = 0.0f;
    _poseD[k] = 0.0f;
    _poseCtrlOut[k]= 0.0f;
    _twist[k] = 0.0f;
    _twistD[k] = 0.0f;
    _twistCtrlOut[k]= 0.0f;
    _wrench[k] = 0.0f;
    _wrenchD[k] = 0.0f;
    _wrenchM[k] = 0.0f;
    _poseFilters[k] = new LP_Filter(0.6);
    _twistFilters[k] = new LP_Filter(0.95);
    _wrenchMFilters[k] = new LP_Filter(0.95);
    _switchesState[k] = 0;
    _kpPose[k] = 0.0f;
    _kiPose[k] = 0.0f; 
    _kdPose[k] = 0.0f;
    _kpTwist[k] = 0.0f;
    _kiTwist[k] = 0.0f;
    _kdTwist[k] = 0.0f;
    _innerCounter=0;
    
    //Define the common PID constants for the GoTo function

     #if (PLATFORM_ID==LEFT_PLATFORM)
          
          _gtKpPose[X] = 2000.0f;
          _gtKdPose[X] = 1.0f;
          _gtKiPose[X] = 2000.0f; //Ki->0.0f;
          _gtKpPose[Y] = 2500.0f;
          _gtKdPose[Y] = 0.5f;
          _gtKiPose[Y] = 2500.0f; //Ki->0.0f;
          _gtKpPose[PITCH] = 3000.0f * PI / 180.0f * 0.01f; //2000.0
          _gtKdPose[PITCH] = 5.0f * PI / 180.0f * 0.01f; // 5.0
          _gtKiPose[PITCH] = 5000.0f * PI / 180.0f * 0.01f; // 1000.0 
          _gtKpPose[ROLL] = 2500.0f * PI / 180.0f * 0.01f;
          _gtKdPose[ROLL] = 10.0f * PI / 180.0f * 0.01f;
          _gtKiPose[ROLL] = 1000.0f * PI / 180.0f * 0.01f; 
          _gtKpPose[YAW] = 2500.0f * PI / 180.0f * 0.01f;
          _gtKdPose[YAW] = 10.0f * PI / 180.0f * 0.01f;
          _gtKiPose[YAW] = 1000.0f * PI / 180.0f * 0.01f; 
        #else  //! TODO TUNE FOR RIGHT_PLATFORM
          
          _gtKpPose[X] = 1000.0f;
          _gtKdPose[X] = 1.0f;
          _gtKiPose[X] = 1000.0f; //Ki->0.0f;
          _gtKpPose[Y] = 2500.0f;
          _gtKdPose[Y] = 0.5f;
          _gtKiPose[Y] = 1000.0f; //Ki->0.0f;
          _gtKpPose[PITCH] = 2500.0f * PI / 180.0f * 0.01f; //2000.0
          _gtKdPose[PITCH] = 5.0f * PI / 180.0f * 0.01f; // 5.0
          _gtKiPose[PITCH] = 2500.0f * PI / 180.0f * 0.01f; // 1000.0 
          _gtKpPose[ROLL] = 2500.0f * PI / 180.0f * 0.01f;
          _gtKdPose[ROLL] = 10.0f * PI / 180.0f * 0.01f;
          _gtKiPose[ROLL] = 1000.0f * PI / 180.0f * 0.01f; 
          _gtKpPose[YAW] = 2500.0f * PI / 180.0f * 0.01f;
          _gtKdPose[YAW] = 10.0f * PI / 180.0f * 0.01f;
          _gtKiPose[YAW] = 1000.0f * PI / 180.0f * 0.01f; 
        #endif


     _pidPose[k] = new PID(&_innerTimer, &_pose[k], &_poseCtrlOut[k], &_poseD[k], _kpPose[k], _kiPose[k], _kdPose[k],DIRECT);
     _pidPose[k]->setMode(AUTOMATIC);
     _pidTwist[k] = new PID(&_innerTimer, &_twist[k], &_twistCtrlOut[k], &_twistD[k], _kpTwist[k], _kiTwist[k], _kdTwist[k],DIRECT);
     _pidTwist[k]->setMode(AUTOMATIC);
  }


  _tic=false;
  _state = HOMING;
  _lastState=_state;
  _frictionIDFlag=false;
  
  // Reset the flags that acknowledge when the state is entered for the first time 
  _stateOnceFlag[HOMING]=false;
  _stateOnceFlag[CENTERING]=false;
  _stateOnceFlag[TELEOPERATION]=false;


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

#if (PLATFORM_ID == LEFT_PLATFORM)
  _subFootInput = new ros::Subscriber<custom_msgs::FootInputMsg>("/FI_Input/Left", updateFootInput);
  _pubFootOutput = new ros::Publisher("/FI_Output/Left", &_msgFootOutput);
#else
  _subFootInput = new ros::Subscriber<custom_msgs::FootInputMsg>("/FI_Input/Right", updateFootInput);
  _pubFootOutput = new ros::Publisher("/FI_Output/Right", &_msgFootOutput);
#endif
  _nh.getHardware()->setBaud(BAUDRATE);
  _nh.initNode();
   wait_ms(10);
  _nh.advertise(*_pubFootOutput);
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

  case STANDBY:{ _lastState=_state; break;}    //Do nothing

  case HOMING:
    {
      // Init
      if(!_stateOnceFlag[HOMING])
      {
        _enableMotors->write(1);
        for(int k = 0; k < NB_AXIS; k++)
          {
            _switchesState[k] = 0;
          }
        // Set commanded forces and torques for homing
        _stateOnceFlag[HOMING]=true;
      }

      _controllerType=TWIST_ONLY;
      
      

      #if (PLATFORM_ID==LEFT_PLATFORM) //! TODO: Set for the left platform
          _twistD[X] = -2.5; // m/s
          _twistD[Y] = 2.5; // m/s
          _twistD[PITCH] = 300; // °/s
          
          _kpTwist[X] = 2500.0f*0.01f;
          _kiTwist[X] = 2500.0f*0.01f; 
          _kpTwist[Y] = 2000.0f*0.01f;
          _kiTwist[Y] = 2000.0f*0.01f; //
          _kpTwist[PITCH] = 10000.0f * PI / 180.0f * 1e-4f; //
          _kiTwist[PITCH] = 5000.0f * PI / 180.0f * 1e-4f; //   
        #else 
          _twistD[X] = 2.5; // m/s
          _twistD[Y] = 2.5; // m/s
          _twistD[PITCH] = -300; // °/s

          _kpTwist[X] = 2500.0f*0.01f;
          _kiTwist[X] = 2500.0f*0.01f; 
          _kpTwist[Y] = 1500.0f*0.01f;
          _kiTwist[Y] = 1000.0f*0.01f; //
          _kpTwist[PITCH] = 10000.0f * PI / 180.0f * 1e-4f; //
          _kiTwist[PITCH] = 5000.0f * PI / 180.0f * 1e-4f; // 
          
        #endif

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
      if (!_stateOnceFlag[CENTERING])
      {
          _stateOnceFlag[CENTERING]=true;
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
   
    {
      // Init State
     
     if (!_stateOnceFlag[TELEOPERATION])
     {
       for(int k=0; k<NB_AXIS; k++)
       {
        _switchesState[k] = 0;
        _poseD[k]=0.0f;
        _kpPose[k]=0.0f;
        _kdPose[k]=0.0f;
        _kiPose[k]=0.0f;
      }
      _stateOnceFlag[TELEOPERATION]=true;
     }

      // Main State
      wsConstrains();
      //frictionID(X,FW_COMP);
      _lastState=_state;
      break;
    }
    case EMERGENCY:
    {
      if(!_stateOnceFlag[EMERGENCY]){
        _stateOnceFlag[EMERGENCY]=true;
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

void Platform::posAxisControl(WrenchComp Component, int axis){

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
    for(int j = 0; j < WRENCH_COMPONENTS; j++)

    {
      wrenchSum+= _wrenchD_ADD[j][k];
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


void Platform::updateFootInput(const custom_msgs::FootInputMsg &msg)
{
  me->_wrenchD_ADD[NORMAL][X] = msg.FxDes;
  me->_wrenchD_ADD[NORMAL][Y] = msg.FyDes;
  me->_wrenchD_ADD[NORMAL][PITCH] = msg.TphiDes;
  me->_wrenchD_ADD[NORMAL][ROLL] = msg.TthetaDes;
  me->_wrenchD_ADD[NORMAL][YAW] = msg.TpsiDes;
  Platform::State newState = (Platform::State) msg.stateDes;
  if (!(newState==me->_state)) // If I want to go to a new state
  {
    me->clearLastState();
    me->_state = newState;
  } 
}


void Platform::pubFootOutput()
{
  _msgFootOutput.id = PLATFORM_ID;
  _msgFootOutput.stamp = _nh.now();
  _msgFootOutput.x = _pose[X];
  //_msgFootOutput.x = (float)_switchesState[0];
  _msgFootOutput.y = _pose[Y];
  //_msgFootOutput.y = (float)_switchesState[1];
  _msgFootOutput.phi = _pose[PITCH];
  //_msgFootOutput.phi = (float)_switchesState[2];
  _msgFootOutput.theta = _pose[ROLL];
  _msgFootOutput.psi = _pose[YAW];
  _msgFootOutput.Fx_d = _wrenchD[X];
  _msgFootOutput.Fy_d = _wrenchD[Y];
  _msgFootOutput.Tphi_d = _wrenchD[PITCH];
  _msgFootOutput.Ttheta_d = _wrenchD[ROLL];
  _msgFootOutput.Tpsi_d = _wrenchD[YAW];
  _msgFootOutput.Fx_m = _wrenchM[X];
  _msgFootOutput.Fy_m = _wrenchM[Y];
  _msgFootOutput.Tphi_m = _wrenchM[PITCH];
  _msgFootOutput.Ttheta_m = _wrenchM[ROLL];
  _msgFootOutput.Tpsi_m = _wrenchM[YAW];
  _msgFootOutput.vx = _twist[X];
  _msgFootOutput.vy = _twist[Y];
  _msgFootOutput.wphi = _twist[PITCH];
  _msgFootOutput.wtheta = _twist[ROLL];
  _msgFootOutput.wpsi = _twist[YAW];
  _msgFootOutput.state = _state;
  _pubFootOutput->publish(&_msgFootOutput);
}

void Platform::gotoPointAxis(int axis_, float point)
{
  _controllerType=POSE_ONLY;
  _kpPose[axis_]=_gtKpPose[axis_];
  _kdPose[axis_]=_gtKdPose[axis_];
  _kiPose[axis_]=_gtKiPose[axis_];
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
  for (int k=0; k<NB_AXIS; k++)
    {
    _kpPose[k]=_gtKpPose[k];
    _kdPose[k]=_gtKdPose[k];
    _kiPose[k]=_gtKiPose[k];
    }
    poseControl(NORMAL);
}

void Platform::wsConstrains()
{


  _controllerType=POSE_ONLY;
  for (int k=0; k<NB_AXIS; k++)
    {
    _kpPose[k]=_gtKpPose[k];
    _kdPose[k]=_gtKdPose[k];
    _kiPose[k]=0.0f;
    //
    if (k>=PITCH){
      _kpPose[k]=2000.0f * PI / 180.0f * 0.01f;
    }
    }
  for (int k = 0; k<NB_AXIS; k++){
  _poseD[k] = _pose[k] >= _c_wsLimits[k] ? _c_wsLimits[k] : (_pose[k] <= -_c_wsLimits[k] ? -_c_wsLimits[k]: 0.0f);
    if ( _pose[k] >= _c_wsLimits[k] || _pose[k] <= -_c_wsLimits[k] )
    {
     if ( ((_pose[k] <= 0.0) && (_twist[k] <= 0.0)) || ((_pose[k] >= 0.0) && (_twist[k] >= 0.0)) ) {
       posAxisControl(CONSTRAINS,k);
      }
    }
    else    
    {
      _wrenchD_ADD[CONSTRAINS][k]=0.0f;
    }
  }
}

void Platform::frictionID(int axis_, int direction_)
{
  if (!_frictionIDFlag){
      _controllerType=POSE_ONLY;
      static float step_ = _wsRange[axis_]/NB_COMPENSATION_STEPS;
      static float tolerance = 0.003;
      _poseD[axis_] = direction_*step_; // Go to the limit

      _kpPose[axis_]=1.0f;
      _kiPose[axis_]=500.0f;
      _kdPose[axis_]=1.0f;

      if (axis_>2)
      {
        _kpPose[axis_]*=PI / 180.0f * 0.01f;
        _kdPose[axis_]*=PI / 180.0f * 0.01f;
        _kiPose[axis_]*=PI / 180.0f * 0.01f;
      }  

      posAxisControl(COMPENSATION,axis_);
      
      if ((fabs(_poseD[axis_]-_pose[axis_])<=tolerance)) {
        _poseD[axis_]+=direction_*step_;
      }

      // If almost in the opposite limit, then finish the friction ID
      if (_poseD[axis_]-(direction_*(_wsRange[axis_]/2))>=2*tolerance){
        _frictionIDFlag=true;
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
    
     for(int j = 0; j < WRENCH_COMPONENTS; j++)
    {
      _wrenchD_ADD[j][k] = 0.0f;
    }
  }
}

void Platform::clearLastState()
{
  switch(_lastState)
    {
      case(HOMING):

      {
        _stateOnceFlag[HOMING]=false;          
            //! Finally resets the wrench commands given by this controller. 
            for (int k = 0; k<NB_AXIS; k++)
              {
              _twistD[k] = 0; // m/s
              _wrenchD_ADD[NORMAL][k] = 0.0f;
              // _pidTwist[k]->reset();
              }
        break; 
      }

      case(CENTERING):
      {
        _stateOnceFlag[CENTERING]=false;
          for (int k = 0; k<NB_AXIS; k++)
            {
              _wrenchD_ADD[NORMAL][k] = 0.0f;
              _poseD[k]=0.0f;
            }
        break;
      }
      case(TELEOPERATION):
      {
        _stateOnceFlag[TELEOPERATION]=false;
        for (int k = 0; k<NB_AXIS; k++)
        {
          for (int j=0; j<WRENCH_COMPONENTS; j++)
            {
              _wrenchD_ADD[j][k] = 0.0f;
            }
          _poseD[k]=0.0f;
        }
        break;
      }
      case(EMERGENCY):{break;}
      case(STANDBY): {break;}
      case(RESET):{break;}
    }
}

void Platform::resetEscons(){

  for(int k = 0; k < NB_AXIS; k++)
  {
    for(int j = 0; j < WRENCH_COMPONENTS; j++)
    {
      _wrenchD_ADD[j][k] = 0.0f;
    }
  }
  setWrenches();
  wait_ms(150);
  _enableMotors->write(1);
  wait_ms(750);
  _enableMotors->write(0);
}

void Platform::softReset(){
   NVIC_SystemReset(); 
}