#include "Platform.h"
Platform *Platform::me = NULL;

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Platform::Platform()
{
  me = this;
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
    _poseFilters[k] = new LP_Filter(0.05);
    _twistFilters[k] = new LP_Filter(0.01);
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
  _motorPins[ROLL] = PB_4; // D5  PWM3/1
  _motorPins[YAW] = PA_8; // D7 PWM1/1

  _limitSwitchesPins[X] = PC_8; // PC_13 NO PWM
  _limitSwitchesPins[Y] = PC_6; // NOT as PWMX/XN
  _limitSwitchesPins[PITCH] = PC_5; // NOT as PWMX/XN


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
      {_limitSwitches[k] = new InterruptIn(_limitSwitchesPins[k]);}
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

  //! Attach interruptions to callbacks on rising edge 
  _limitSwitches[X]->fall(&switchCallbackX);
  _limitSwitches[Y]->fall(&switchCallbackY);
  _limitSwitches[PITCH]->fall(&switchCallbackPitch);
  
  _spi->lock();
  _encoders[X]->QEC_init(X, ENCODERSCALE1, ENCODERSIGN1,_spi);
    _encoders[Y]->QEC_init(Y, ENCODERSCALE2, ENCODERSIGN2,_spi);
    _encoders[PITCH]->QEC_init(PITCH, ENCODERSCALE3, ENCODERSIGN3,_spi);
    _encoders[ROLL]->QEC_init(ROLL, ENCODERSCALE4, ENCODERSIGN4,_spi);
    _encoders[YAW]->QEC_init(YAW, ENCODERSCALE5, ENCODERSIGN5,_spi);
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
  // Get the pose of the platform.
  //getMotion(); // Since it uses SPI I will make it public and run it out of the interruption

  switch (_state)
  {
    case HOMING:
    {
      // Set commanded forces and torques for homing
      _wrenchD[X] = HOMING_FORCE_X;   //
      _wrenchD[Y] = HOMING_FORCE_Y;   // [N]
      _wrenchD[PITCH] = HOMING_TORQUE_P;  // [Nm]
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
    case CENTERING:
    {
      for(int k = 0; k < NB_AXIS; k++)
      {
        _switchesState[k] = 0;
        _poseD[k] = 0.0f;
      }
      _kpPose[X] = 100.0f;
      _kdPose[X] = 0.0f;
      _kiPose[X] = 0.0f; //Ki->0.0f;
      _kpPose[Y] = 200.0f;
      _kdPose[Y] = 0.0f;
      _kiPose[Y] = 0.0f; //Ki->0.0f;
      _kpPose[PITCH] = 1000.0f * PI / 180.0f * 0.01f;
      _kdPose[PITCH] = 0.0f * PI / 180.0f * 0.01f;
      _kiPose[PITCH] = 0.0f * PI / 180.0f * 0.01f; // For the moment set to zero the roll and yaw

      _kpPose[4] = 0.0f;
      _kpPose[3] = 0.0f;
      _controllerType=POSE_ONLY;
      poseControl();

      if((fabs(_poseD[X]-_pose[X]) < 0.003f) && (fabs(_poseD[Y]-_pose[Y]) < 0.003f) && (fabs(_poseD[PITCH]-_pose[PITCH]) < 3.0f))
      {
        static uint32_t idle = _innerTimer.read_us();
        // After a second and a half move to next state
        if ((_innerTimer.read_us() - idle) > 1500000)
        {
          _state = NORMAL;
        };
      }
      break;
    }
    case NORMAL:
    {
      _controllerType=TORQUE_ONLY;
      for (int k = 0; k < NB_AXIS; k++)
      {
        _switchesState[k] = 0;
      }
      break;
    }
    case COMPENSATION:
    {
      break;
    }
    case FEEDFORWARD:
    {
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


void Platform::poseControl()
{
  //Compute the PID

  for (int k = 0; k < NB_AXIS; k++)
  {  
    if ((k<2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      _pidPose[k]->setOutputLimits(-25,25); //!N
    }

    if ((k>=2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      _pidPose[k]->setOutputLimits(-12,12); //!Nm
    }

    if (_controllerType==TWIST_POSE_CASCADE){
      _poseD[k]=_twistCtrlOut[k];
    }

     _pidPose[k]->setTunings(_kpPose[k], _kiPose[k], _kdPose[k]);
     //_pidPose[k]->setProcessValue(_pose[k]);
     //_pidPose[k]->setSetPoint(_poseD[k]);
     _pidPose[k]->compute();

    if ((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE)){
      _wrenchD[k]=_poseCtrlOut[k];
    }
  }
}


void Platform::twistControl()
{

  for (int k = 0; k < NB_AXIS; k++)
  {  
    if ((k<2)&&((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE))){
      _pidTwist[k]->setOutputLimits(-25,25); //!N
    }

    if ((k>=2)&&((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE))){
      _pidTwist[k]->setOutputLimits(-12,12); //!Nm
    }

    if (_controllerType==POSE_TWIST_CASCADE){
      _poseD[k]=_twistCtrlOut[k];
    }

     _pidTwist[k]->setTunings(_kpTwist[k], _kiTwist[k], _kdTwist[k]);
     //_pidTwist[k]->setProcessValue(_pose[k]);
     //_pidTwist[k]->setSetPoint(_poseD[k]);
     _pidTwist[k]->compute();

    if ((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE)){
      _wrenchD[k]=_twistCtrlOut[k];
    }
  }
}


void Platform::setWrenches()
{
  for(int k = 0; k <2; k++)
  {
    if(k<2)
    {
      setForce(_wrenchD[k], _motors[k], 1, k);
    }  
  }
  setTorque(_wrenchD[PITCH],_motors[PITCH],1,(int)PITCH);
  // Adapt roll and yaw commands due to differential mechanism
  setTorque((_wrenchD[YAW]+_wrenchD[ROLL])/2.0f, _motors[ROLL], 1, (int)ROLL);
  setTorque((_wrenchD[YAW]-_wrenchD[ROLL])/2.0f, _motors[YAW], 1, (int)YAW);
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

  double escon_current = (torque * 1000 / kTau) / reduction;
  escon_current = (escon_current > iMax ? iMax : (escon_current < -iMax ? -iMax : escon_current)); 
  float escon_current_PWM = map(escon_current, -iMax, iMax, 0.1f, 0.9f); //! from 10 to 90% of Duty Cycle to acknowledge connection
  escon_current_PWM *= sign;
  pin->write(escon_current_PWM);
  
}


void Platform::switchCallbackX()
{
  if (me->_switchesState[X] == 0)
  {
    me->_switchesState[X] = 1;
  }
}


void Platform::switchCallbackY()
{
  if (me->_switchesState[Y] == 0)
  {
    me->_switchesState[Y] = 1;
  }
}


void Platform::switchCallbackPitch()
{
  if (me->_switchesState[PITCH] == 0)
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
      _encoders[k]->QEC_offset(_spi);
      }
    _spi->unlock();

    // TODO add other offsets !!!!!
  }
}


void Platform::updateFootInput(const custom_msgs::FootInputMsg &msg)
{
  //TODO: Check if it should be the commands or the measured forces and torques
  me->_wrenchD[X] = msg.FxDes;
  me->_wrenchD[Y] = msg.FyDes;
  me->_wrenchD[PITCH] = msg.TphiDes;
  /* s_torqueR = s_wrench_msg.torque.y;
    s_torqueY = s_wrench_msg.torque.z */
  me->_state = (Platform::State) msg.stateDes;
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