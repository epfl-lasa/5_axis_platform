#include "Platform.h"
#include "definitions.h"
Platform *Platform::me = NULL;

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Platform::Platform()
{
  me = this;

  _encoderScale[X] =ENCODERSCALE1;
  _encoderScale[Y] =ENCODERSCALE2;
  _encoderScale[PITCH] =ENCODERSCALE3;
  _encoderScale[ROLL] =ENCODERSCALE4;
  _encoderScale[YAW] =ENCODERSCALE5;

  _encoderSign[X] =ENCODERSIGN1;
  _encoderSign[Y] =ENCODERSIGN2;
  _encoderSign[PITCH] =ENCODERSIGN3;
  _encoderSign[ROLL] =ENCODERSIGN4;
  _encoderSign[YAW] =ENCODERSIGN5;

  _motorSign[X] =MOTORSIGN1;
  _motorSign[Y] =MOTORSIGN2;
  _motorSign[PITCH] =MOTORSIGN3;
  _motorSign[ROLL] =MOTORSIGN4;
  _motorSign[YAW] =MOTORSIGN5;

  _wsLimits[X] = WS_LIMIT_X;
  _wsLimits[Y] = WS_LIMIT_Y;
  _wsLimits[PITCH] = WS_LIMIT_PITCH;
  _wsLimits[ROLL] = WS_LIMIT_ROLL;
  _wsLimits[YAW] = WS_LIMIT_YAW;


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
    _poseFilters[k] = new LP_Filter(0.65);
    _twistFilters[k] = new LP_Filter(0.9);
    _switchesState[k] = 0;
    _kpPose[k] = 0.0f;
    _kiPose[k] = 0.0f; //! Ki->0
    _kdPose[k] = 0.0f;
    _kpTwist[k] = 0.0f;
    _kiTwist[k] = 0.0f;
    _kdTwist[k] = 0.0f;

     _pidPose[k] = new PID(&_innerTimer, &_pose[k], &_poseCtrlOut[k], &_poseD[k], _kpPose[k], _kiPose[k], _kdPose[k],DIRECT);
     _pidPose[k]->setMode(AUTOMATIC);
     _pidTwist[k] = new PID(&_innerTimer, &_twist[k], &_twistCtrlOut[k], &_twistD[k], _kpTwist[k], _kiTwist[k], _kdTwist[k],DIRECT);
     _pidTwist[k]->setMode(AUTOMATIC);
  }

  _state = HOMING;
  
  // Reset the flags that acknowledge when the state is entered for the first time 
  _stateOnceFlag[HOMING]=false;
  _stateOnceFlag[CENTERING]=false;
  _stateOnceFlag[TELEOPERATION]=false;


  _controllerType= TORQUE_ONLY;

#if (BOARD==NUCLEO64)
  /*******DESIGNATIONS OF PINS IN THE MICROCONTROLLER NUCLEO L476RG */

  _csPins[X] = D2;  //! CS1 -> Lateral NOT as PWMX/XN
  _csPins[Y] = D6;  //! CS2  -> Dorsi/Plantar Flexion NOT as PWMX/XN
  _csPins[PITCH] = D8; //! CS3 -> Flexion/Extension of the Leg NOT as PWM/XN
  _csPins[ROLL] = D10;  //! CS4 -> roll and yaw encoder 1 NOT as PWMX/XN
  _csPins[YAW] = PB_2;  //! CS5 -> roll and yaw encoder 2  NOT as PWMX/X
  
  _motorPins[X] = PC_7_ALT0; // D9 PWM8/2
  _motorPins[Y] = PB_3; // D3  PWM2/2
  _motorPins[PITCH] = PB_5; // D4 PWM3/2 
  _motorPins[ROLL] = PA_8; // D7 PWM1/1
  _motorPins[YAW] = PB_4; // D5  PWM3/1

  _limitSwitchesPins[X] = PC_8; //  NOT as PWMX/XN
  _limitSwitchesPins[Y] = PC_5; // NOT as PWMX/XN
  _limitSwitchesPins[PITCH] = PC_6; // NOT as PWMX/XN


  _spi = new SPI(PA_7, PA_6, PA_5); // mosi, miso, sclk https://os.mbed.com/platforms/ST-Nucleo-L476RG/
  // _spi->format(8,0); // Default
  // _spi->frequency(1000000); // Default
  /************************************************************* */
#elif (BOARD==NUCLEO32)

  /*******DESIGNATIONS OF PINS IN THE MICROCONTROLLER NUCLEO F303K8 */
  _csPins[X] = D6;  //! CS1 -> Lateral NOT as PWMX/XN
  _csPins[Y] = D5;  //! CS2  -> Dorsi/Plantar Flexion NOT as PWMX/XN
  _csPins[PITCH] = D4; //! CS3 -> Flexion/Extension of the Leg NOT as PWM/XN
  _csPins[ROLL] = A1;  //! CS4 -> roll and yaw encoder 1 NOT as PWMX/XN
  _csPins[YAW] = A2;  //! CS5 -> roll and yaw encoder 2  NOT as PWMX/X

  _motorPins[X] = PA_8; // D9 PA_8 PWM1/1
  _motorPins[Y] = PA_11_ALT0; // D10 PA_11_ALT0 PWM1/4   ***UNCOMMENT THIS IF USING NUCLEO32***
  _motorPins[PITCH] = PA_9; // D1 PA_9 PWM1/2 
  _motorPins[ROLL] = PA_10; // D0 PA_10 PWM1/3
  _motorPins[YAW] = PA_12_ALT0; // D2 PA_12_ALT0 PWM16/1 ***UNCOMMENT THIS IF USING NUCLEO32***

  _limitSwitchesPins[X] = D8; // NO PWM
  _limitSwitchesPins[Y] = D7; // NOT as PWMX/XN
  _limitSwitchesPins[PITCH] = D3; // NOT as PWMX/XN

  _spi = new SPI(PB_5, PB_4, PB_3); // mosi, miso, sclk https://os.mbed.com/platforms/ST-Nucleo-F303K8/ and PeripheralPins.c from mbed
  
#endif

  wait_ms(10); //! Wait a bit after the SPI starts

  for(int k = 0; k < NB_AXIS; k++)
  {
    _encoders[k] = new QEC_1X(_csPins[k]);
    _motors[k] = new PwmOut(_motorPins[k]);
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
_timestamp=0;
}


Platform::~Platform()
{
  _innerTimer.~Timer();
  for(int k = 0; k <NB_AXIS; k++)
  {
    delete (_poseFilters[k]);
    delete (_twistFilters[k]);
    delete (_pidPose[k]);
    delete (_pidTwist[k]);
    delete (_encoders[k]);
    delete (_motors[k]);
    delete (_limitSwitches[k]);
  }
  delete (_spi);  
}


void Platform::init()
{
    
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
    for(int k = 0; k <NB_AXIS; k++)
    {

     _motors[k]->period_us(200); // PWM (to ESCON) PERIOD 200 us-> 1kHz    
    
    if (k<2){
     _pidPose[k]->setOutputLimits(-25.0, 25.0);
     _pidTwist[k]->setOutputLimits(-25.0, 25.0);  
    }
     else {
     _pidPose[k]->setOutputLimits(-12.0, 12.0);
     _pidTwist[k]->setOutputLimits(-12.0, 12.0);    
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
}

void Platform::step()
{
  
  switch (_state)
  {

  #if (HOMING_TECHNIQUE==TORQUE_CONTROLLED_HOMMING)    
    case HOMING:
    {
      // Init
      if(!_stateOnceFlag[HOMING])
      {
        for(int k = 0; k < NB_AXIS; k++)
          {
            _switchesState[k] = 0;
          }
        // Set commanded forces and torques for homing
        _stateOnceFlag[HOMING]=true;
      }
      
      _wrenchD_ADD[NORMAL][X] = HOMING_FORCE_X;   //
      _wrenchD_ADD[NORMAL][Y] = HOMING_FORCE_Y;   // [N]
      _wrenchD_ADD[NORMAL][PITCH] = HOMING_TORQUE_P;  // [Nm]
      _controllerType=TORQUE_ONLY;

      // Definition of the transition rule to the next state
      if ((_switchesState[X] == 1) && (_switchesState[Y] == 1) && (_switchesState[PITCH] == 1))
      {
        poseAllReset();
        static uint32_t idle = _innerTimer.read_us();
        //  After 1.5 second move to next state
        if ((_innerTimer.read_us() - idle) > 150000)
        {
          _state = CENTERING;
        }
      }
      break;
    }
  #else

    case HOMING:
    {
      // Init
      if(!_stateOnceFlag[HOMING])
      {
        for(int k = 0; k < NB_AXIS; k++)
          {
            _switchesState[k] = 0;
          }
        // Set commanded forces and torques for homing
        _stateOnceFlag[HOMING]=true;
      }

      _controllerType=TWIST_ONLY;
      
      for(int k = 0; k < NB_AXIS; k++)
          {
            _twistD[k] = 2;
          }

      #if (PLATFORM_ID==LEFT_PLATFORM)
          // Set the PID tunings for the centering
          _kpTwist[X] = 0.0f;
          _kdTwist[X] = 0.0f;
          _kiTwist[X] = 0.0f; //Ki->0.0f;
          _kpTwist[Y] = 0.0f;
          _kdTwist[Y] = 0.0f;
          _kiTwist[Y] = 0.0f; //Ki->0.0f;
          _kpTwist[PITCH] = 0.0f * PI / 180.0f * 0.01f;
          _kdTwist[PITCH] = 0.0f * PI / 180.0f * 0.01f;HOMING
          _kiTwist[PITCH] = 0.0f * PI / 180.0f * 0.01fHOMING; 
          _kpTwist[ROLL] = 0.0f * PI / 180.0f * 0.01f;HOMING
          _kdTwist[ROLL] = 0.0f * PI / 180.0f * 0.01f;HOMING
          _kiTwist[ROLL] = 0.0f * PI / 180.0f * 0.01f; 
          _kpTwist[YAW] = 0.0f * PI / 180.0f * 0.01f;
          _kdTwist[YAW] = 0.0f * PI / 180.0f * 0.01f;
          _kiTwist[YAW] = 0.0f * PI / 180.0f * 0.01f; 
        #else  //! TODO TUNE FOR RIGHT_PLATFORM
          // Set the PID tunings for the centering
          _kpTwist[X] = 1500.0f*0.01;
          _kdTwist[X] = 0.0f*0.01f;
          _kiTwist[X] = 1000.0f*0.01f; //Ki->0.0f;
          _kpTwist[Y] = 1500.0f*0.01f;
          _kdTwist[Y] = 0.0f*0.01f;
          _kiTwist[Y] = 1000.0f*0.01f; //Ki->0.0f;
          _kpTwist[PITCH] = 0.0f * PI / 180.0f * 0.01f; //2000.0
          _kdTwist[PITCH] = 0.0f * PI / 180.0f * 0.01f; // 5.0
          _kiTwist[PITCH] = 0.0f * PI / 180.0f * 0.01f; // 1000.0 
          _kpTwist[ROLL] = 0.0f * PI / 180.0f * 0.01f;
          _kdTwist[ROLL] = 0.0f * PI / 180.0f * 0.01f;
          _kiTwist[ROLL] = 0.0f * PI / 180.0f * 0.01f; 
          _kpTwist[YAW] = 0.0f * PI / 180.0f * 0.01f;
          _kdTwist[YAW] = 0.0f * PI / 180.0f * 0.01f;
          _kiTwist[YAW] = 0.0f * PI / 180.0f * 0.01f; 
        #endif

      twistControl(NORMAL);

      // Definition of the transition rule to the next state
      if ((_switchesState[X] == 1) && (_switchesState[Y] == 1) && (_switchesState[PITCH] == 1))
      {
        poseAllReset();
        static uint32_t idle = _innerTimer.read_us();
        //  After 1.5 second move to next state
        if ((_innerTimer.read_us() - idle) > 150000)
        {
          _state = CENTERING;
        }
      }
      break;
    }

  #endif

    case CENTERING:
    {
      // Init State
      _stateOnceFlag[HOMING]=false;
      if (!_stateOnceFlag[CENTERING])
      {
        for(int k = 0; k < NB_AXIS; k++)
        {
          _poseD[k] = 0.0f;
        }

        #if (PLATFORM_ID==LEFT_PLATFORM)
          // Set the PID tunings for the centering
          _kpPose[X] = 1000.0f;
          _kdPose[X] = 1.0f;
          _kiPose[X] = 1000.0f; //Ki->0.0f;
          _kpPose[Y] = 1500.0f;
          _kdPose[Y] = 1.0f;
          _kiPose[Y] = 1000.0f; //Ki->0.0f;
          _kpPose[PITCH] = 2000.0f * PI / 180.0f * 0.01f;
          _kdPose[PITCH] = 5.0f * PI / 180.0f * 0.01f;HOMING
          _kiPose[PITCH] = 1000.0f * PI / 180.0f * 0.01fHOMING; 
          _kpPose[ROLL] = 1000.0f * PI / 180.0f * 0.01f;HOMING
          _kdPose[ROLL] = 5.0f * PI / 180.0f * 0.01f;HOMING
          _kiPose[ROLL] = 1000.0f * PI / 180.0f * 0.01f; 
          _kpPose[YAW] = 1000.0f * PI / 180.0f * 0.01f;
          _kdPose[YAW] = 5.0f * PI / 180.0f * 0.01f;
          _kiPose[YAW] = 1000.0f * PI / 180.0f * 0.01f; 
        #else  //! TODO TUNE FOR RIGHT_PLATFORM
          // Set the PID tunings for the centering
          _kpPose[X] = 1000.0f;
          _kdPose[X] = 1.0f;
          _kiPose[X] = 1000.0f; //Ki->0.0f;
          _kpPose[Y] = 2500.0f;
          _kdPose[Y] = 0.5f;
          _kiPose[Y] = 1000.0f; //Ki->0.0f;
          _kpPose[PITCH] = 2000.0f * PI / 180.0f * 0.01f; //2000.0
          _kdPose[PITCH] = 5.0f * PI / 180.0f * 0.01f; // 5.0
          _kiPose[PITCH] = 1000.0f * PI / 180.0f * 0.01f; // 1000.0 
          _kpPose[ROLL] = 0.0f * PI / 180.0f * 0.01f;
          _kdPose[ROLL] = 0.0f * PI / 180.0f * 0.01f;
          _kiPose[ROLL] = 0.0f * PI / 180.0f * 0.01f; 
          _kpPose[YAW] = 0.0f * PI / 180.0f * 0.01f;
          _kdPose[YAW] = 0.0f * PI / 180.0f * 0.01f;
          _kiPose[YAW] = 0.0f * PI / 180.0f * 0.01f; 
        #endif
          // Set the controller to position control
          _controllerType=POSE_ONLY;
          _stateOnceFlag[CENTERING]=true;
      }
      // Main State
      poseControl(NORMAL);

      if((fabs(_poseD[X]-_pose[X]) < 0.003f) && (fabs(_poseD[Y]-_pose[Y]) < 0.003f) && (fabs(_poseD[PITCH]-_pose[PITCH]) < 3.0f))
      {
        static uint32_t idle = _innerTimer.read_us();
        // After a second and a half move to next state
        if ((_innerTimer.read_us() - idle) > 1500000)
        {
          _state = TELEOPERATION;
          //! Finally resets the wrench commands given by this controller. 
          for (int k = 0; k<NB_AXIS; k++)
            {
          _wrenchD_ADD[NORMAL][k] = 0.0f;
            }
        };
      }
      break;
    }
    case TELEOPERATION:
   
    {

      // Init State
     _stateOnceFlag[CENTERING]=false;
     if (!_stateOnceFlag[TELEOPERATION])
     {
       for(int k=0; k<NB_AXIS; k++)
       {
        _switchesState[k] = 0;
        _poseD[k]=0.0f;
        _kdPose[k]=0.0f;
        _kiPose[k]=0.0f;
        switch(k)
          {
            case(X): { _kpPose[X] = 1000.0f; break;}
            case(Y): { _kpPose[Y] = 1500.0f; break;}
            case(PITCH): { _kpPose[PITCH] = 0.0f * PI / 180.0f * 0.01f; break;}
            case(ROLL): { _kpPose[ROLL] = 0.0f * PI / 180.0f * 0.01f; break;}
            case(YAW): { _kpPose[YAW] = 0.0f * PI / 180.0f * 0.01f; break;}
          }
      }
      _stateOnceFlag[TELEOPERATION]=true;
     }

      // Main State

     //wsConstrains();

     _controllerType=TORQUE_ONLY; //! Reset the controller type to torque only in case another function needs it

      break;
    }
  }

  // Apply forces and torques
 setWrenches();
}

void Platform::getMotion()
{
  getPose();
  getTwist();
}

void Platform::communicateToRos()
{
  // Publish foot output
   pubFootOutput();
  // _nh.spinOnce(); //Publishes and Retrieves Messages
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
  if (fabs(_innerTimer.read_us() - _timestamp) >= VELOCITY_PID_SAMPLE_P)
  {
    for (int k = 0; k < NB_AXIS; k++)
    {
      _twist[k] = (_pose[k] - _posePrev[k]) / (VELOCITY_PID_SAMPLE_P * 1e-6f);
      _twist[k] = _twistFilters[k]->update(_twist[k]);
      _posePrev[k] = _pose[k];
    }
    _timestamp = _innerTimer.read_us();
  }
}

void Platform::posAxisControl(WrenchComp Component, int axis){

    if ((axis<2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      _pidPose[axis]->setOutputLimits(-25,25); //!N
    }

    if ((axis>=2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      _pidPose[axis]->setOutputLimits(-12,12); //!Nm
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
      _pidTwist[axis]->setOutputLimits(-12,12); //!Nm
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
  

  for(int k = 0; k <2; k++)
  {
    if(k<2)
    {
      setForce(_wrenchD[k], _motors[k], _motorSign[k], (int)k);
    }  
  }

  setTorque(_wrenchD[PITCH],_motors[PITCH],_motorSign[PITCH],(int)PITCH);
  // Adapt roll and yaw commands due to differential mechanism
  setTorque((_wrenchD[YAW]+_wrenchD[ROLL])/2.0f, _motors[ROLL], _motorSign[ROLL], (int)ROLL);
  setTorque((_wrenchD[YAW]-_wrenchD[ROLL])/2.0f, _motors[YAW], _motorSign[YAW], (int)YAW);

}


void Platform::setForce(float force, PwmOut *pin, int sign, int axis)
{
  if(axis==(int)X)
  {
    force /= cos(PI/3.0f);
  }
  float escon_torque = force * BELT_PULLEY_R; //! Convert from torque to force
  setTorque(escon_torque, pin, sign, axis);
}


void Platform::setTorque(float torque, PwmOut *pin, int sign, int axis)
{
  float kTau = 0.0f, iMax = 0.0f, reduction = 0.0;

  switch (axis)
  {
    case X:
    {
      kTau = TORQUE_CONSTANT_X;
      iMax = MAX_CURRENT_X;
      reduction = 1.0f;
      break;
    }
    case Y:
    {
      kTau = TORQUE_CONSTANT_Y;
      iMax = MAX_CURRENT_Y;
      reduction = 1.0f;
      break;
    }
    case PITCH:
    {
      kTau = TORQUE_CONSTANT_PITCH_ROLL_YAW;
      iMax = MAX_CURRENT_PITCH_ROLL_YAW;
      reduction = PITCH_REDUCTION_R;
      break;
    }
    case ROLL:
    {
      kTau = TORQUE_CONSTANT_PITCH_ROLL_YAW;
      iMax = MAX_CURRENT_PITCH_ROLL_YAW;
      reduction = ROLL_YAW_REDUCTION_R;
      break;
    }
    case YAW:
    {
      kTau = TORQUE_CONSTANT_PITCH_ROLL_YAW;
      iMax = MAX_CURRENT_PITCH_ROLL_YAW;
      reduction = ROLL_YAW_REDUCTION_R;
      break;
    }
  }

  double escon_current = sign*(torque * 1000 / kTau) / reduction;
  escon_current = (escon_current > iMax ? iMax : (escon_current < -iMax ? -iMax : escon_current)); 
  float escon_current_PWM = map(escon_current, -iMax, iMax, 0.1f, 0.9f); //! from 10 to 90% of Duty Cycle to acknowledge connection
  pin->write(escon_current_PWM);
  
}


void Platform::switchCallbackX()
{
  if ((me->_switchesState[X] == 0) && (me->_limitSwitches[X]->read()==1) )
  {
    me->_switchesState[X] = 1;
  }
}


void Platform::switchCallbackY()
{
  if ((me->_switchesState[Y] == 0) && (me->_limitSwitches[Y]->read()==1))
  {
    me->_switchesState[Y] = 1;
  }
}


void Platform::switchCallbackPitch()
{
  if ((me->_switchesState[PITCH] == 0) && (me->_limitSwitches[PITCH]->read()==1))
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
  //TODO: Check if it should be the commands or the measured forces and torques
  me->_wrenchD_ADD[NORMAL][X] = msg.FxDes;
  me->_wrenchD_ADD[NORMAL][Y] = msg.FyDes;
  me->_wrenchD_ADD[NORMAL][PITCH] = msg.TphiDes;
  me->_wrenchD_ADD[NORMAL][ROLL] = msg.TthetaDes;
  me->_wrenchD_ADD[NORMAL][YAW] = msg.TpsiDes;
  Platform::State newState = (Platform::State) msg.stateDes;
  if (!(newState==me->_state)) // If I want to go to a new state
  {
    me->_stateOnceFlag[me->_state]=false; //! Unless it is in teleoperation (continuous messages)
    me->_stateOnceFlag[newState]=false; 
    me->_state = newState;
  }
  
}


void Platform::pubFootOutput()
{
  _msgFootOutput.id = PLATFORM_ID;
  _msgFootOutput.stamp = _nh.now();
  //_msgFootOutput.x = _pose[X];
  _msgFootOutput.x = (float)_switchesState[0];
  //_msgFootOutput.y = _pose[Y];
  _msgFootOutput.y = (float)_switchesState[1];
  //_msgFootOutput.phi = _pose[PITCH];
  _msgFootOutput.phi = (float)_switchesState[2];
  _msgFootOutput.theta = _pose[ROLL];
  _msgFootOutput.psi = _pose[YAW];
  _msgFootOutput.Fx = _wrenchD[X];
  _msgFootOutput.Fy = _wrenchD[Y];
  _msgFootOutput.Tphi = _wrenchD[PITCH];
  _msgFootOutput.Ttheta = _wrenchD[ROLL];
  _msgFootOutput.Tpsi = _wrenchD[YAW];
  _msgFootOutput.vx = _twist[X];
  _msgFootOutput.vy = _twist[Y];
  _msgFootOutput.wphi = _twist[PITCH];
  _msgFootOutput.wtheta = _twist[ROLL];
  _msgFootOutput.wpsi = _twist[YAW];
  _msgFootOutput.state = _state;
  _pubFootOutput->publish(&_msgFootOutput);
}

void Platform::wsConstrains()
{
  _controllerType=POSE_ONLY;

  for (int k = 0; k<NB_AXIS; k++){
  _poseD[k] = _pose[k] >= fabs(_wsLimits[k]) ? fabs(_wsLimits[k]) : (_pose[k] <= -fabs(_wsLimits[k]) ? -fabs(_wsLimits[k]): _poseD[k]);
    if ( _pose[k] >= fabs(_wsLimits[k]) || _pose[k] <= -fabs(_wsLimits[k]) )
    {
      posAxisControl(CONSTRAINS,k);
    }
    else    
    {
      _wrenchD_ADD[CONSTRAINS][k]=0.0f;
    }
  }
}

void Platform::frictionID()
{
  
  
}