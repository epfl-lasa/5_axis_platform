#include "Platform.h"

Platform *Platform::me = NULL;

Platform::Platform()
{
  me = this;
  for(int k = 0; k < NB_AXIS; k++)
  {
    _pose[k] = 0.0f;
    _poseOffsets[k] = 0.0f;
    _posePrev[k] = 0.0f;
    _poseD[k] = 0.0f;
    _twist[k] = 0.0f;
    _twistD[k] = 0.0f;
    _wrench[k] = 0.0f;
    _commands[k] = 0.0f;
    _poseFilters[k] = new LP_Filter(0.05);
    _twistFilters[k] = new LP_Filter(0.01);
    _switchesState[k] = 0;
    _kpPose[k] = 0.0f;
    _kiPose[k] = 0.0f;
    _kdPose[k] = 0.0f;
    _kpTwist[k] = 0.0f;
    _kiTwist[k] = 0.0f;
    _kdTwist[k] = 0.0f;
    _pidPose[k] = new PID(&_pose[k], &_commands[k], &_poseD[k], _kpPose[k], _kiPose[k], _kdPose[k], DIRECT);
    _pidTwist[k] = new PID(&_twist[k], &_commands[k], &_twistD[k], _kpTwist[k], _kiTwist[k], _kdTwist[k], DIRECT);
  }

  _cs[X] = D6;  //! CS1 -> Lateral
  _cs[Y] = D9;  //! CS2  -> Dorsi/Plantar Flexion
  _cs[PITCH] = D10; //! CS3 -> Flexion/Extension of the Leg
  _cs[ROLL] = A1;  //! CS4 -> roll and yaw encoder 1
  _cs[YAW] = A2;  //! CS5 -> roll and yaw encoder 2
  for(int k = 0; k < NB_AXIS; k++)
  {
    _encoders[k] = new QEC_1X(_cs[k]);
  }
  _motorsPins[X] = D3;
  _motorsPins[Y] = D4;
  _motorsPins[PITCH] = D5;
  _motorsPins[ROLL] = D0;
  _motorsPins[YAW] = D1;
  _limitSwitchesPins[X] = D8;
  _limitSwitchesPins[Y] = D7;
  _limitSwitchesPins[PITCH] = D2;

  _timestamp = micros();
}


Platform::~Platform()
{
  for(int k = 0; k <NB_AXIS; k++)
  {
    delete (_poseFilters[k]);
    delete (_twistFilters[k]);
    delete (_pidPose[k]);
    delete (_pidTwist[k]);
    delete (_encoders[k]);
  }
}


void Platform::init()
{
  for(int k = 0; k <NB_AXIS; k++)
  {
    pinMode(_limitSwitchesPins[k],INPUT);
    if(k<2)
    {
      _pidPose[k]->SetOutputLimits(-25.0, 25.0);
      _pidTwist[k]->SetOutputLimits(-25.0, 25.0); //! N
    }
    else
    {
      _pidPose[k]->SetOutputLimits(-12.0, 12.0);
      _pidTwist[k]->SetOutputLimits(-12.1, 12.1); //! N
    }
    _pidPose[k]->SetSampleTime(POSE_PID_SAMPLE_P); //! [us]
    _pidTwist[k]->SetSampleTime(VELOCITY_PID_SAMPLE_P);
    _pidPose[k]->SetMode(AUTOMATIC); //! [us]
    _pidTwist[k]->SetMode(AUTOMATIC);
  }

  attachInterrupt(_limitSwitchesPins[X], switchCallbackX, RISING);
  attachInterrupt(_limitSwitchesPins[Y], switchCallbackY, RISING);
  attachInterrupt(_limitSwitchesPins[PITCH], switchCallbackPitch, RISING);

  _encoders[X]->QEC_init(X, ENCODERSCALE1, ENCODERSIGN1);
  _encoders[Y]->QEC_init(Y, ENCODERSCALE2, ENCODERSIGN2);
  _encoders[PITCH]->QEC_init(PITCH, ENCODERSCALE3, ENCODERSIGN3);


#if (PLATFORM_ID == LEFT_PLATFORM)
  _subFootInput = new ros::Subscriber<custom_msgs::FootInputMsg>("/FI_Input/Left", updateFootInput);
  _pubFootOutput = new ros::Publisher("/FI_Output/Left", &_msgFootOutput);
#else
  _subFootInput = new ros::Subscriber<custom_msgs::FootInputMsg>("/FI_Input/Right", updateFootInput);
  _pubFootOutput = new ros::Publisher("/FI_Output/Right", &_msgFootOutput);
#endif

  _nh.initNode();
  _nh.advertise(*_pubFootOutput);
  _nh.subscribe(*_subFootInput);

  analogWriteResolution(12);
  analogReadResolution(12);
}


void Platform::step()
{
  // Get the pose of the platform.
  getMotion();

  switch (_state)
  {
    case HOMING:
    {
      // Set commanded forces and torques for homing
      _commands[X] = HOMING_FORCE_X;   //
      _commands[Y] = HOMING_FORCE_Y;   // [N]
      _commands[PITCH] = HOMING_TORQUE_P;  // [Nm]
      
      // Definition of the transition rule to the next state
      if ((_switchesState[X] == 1) && (_switchesState[Y] == 1) && (_switchesState[PITCH] == 1))
      {
        allReset();
        static uint32_t idle = micros();
        //  After 1.5 second move to next state
        if ((micros() - idle) > 1500000)
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
        if(k<2)
        {
          _kpPose[k] = 1000.0f;
        }
        else
        {
          _kpPose[k] = 5000 * PI / 180.0f * 0.001f;
        }
        _kdPose[k] = 0.0f;
        _kiPose[k] = 0.0f;
      }
      poseControl();

      if((fabs(_poseD[X]-_pose[X]) < 0.003f) && (fabs(_poseD[Y]-_pose[Y]) < 0.003f) && (fabs(_poseD[PITCH]-_pose[PITCH]) < 3.0f))
      {
        static uint32_t idle = micros();
        // After a second and a half move to next state
        if ((micros() - idle) > 1500000)
        {
          _state = NORMAL;
        };
      }
      break;
    }
    case NORMAL:
    {
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

  // Publish foot output
  pubFootOutput();

  _nh.spinOnce();
  delayMicroseconds(LOOP_P);
}

void Platform::getMotion()
{
  getPose();
  getTwist();
}


void Platform::getPose()
{
  for (int k = 0; k < NB_AXIS; k++)
  {
    _encoders[k]->QEC_getPose();
    _pose[k] = _encoders[k]->outDimension + _poseOffsets[k];
    _pose[k] = _poseFilters[k]->Update(_pose[k]);
  }
}


void Platform::getTwist()
{
  if (fabs(micros() - _timestamp) >= VELOCITY_PID_SAMPLE_P)
  {
    for (int k = 0; k < NB_AXIS; k++)
    {
      _twist[k] = (_pose[k] - _posePrev[k]) / (VELOCITY_PID_SAMPLE_P * 1e-6f);
      _twist[k] = _twistFilters[k]->Update(_twist[k]);
      _posePrev[k] = _pose[k];
    }
    _timestamp = micros();
  }
}


void Platform::poseControl()
{
  for (int k = 0; k < NB_AXIS; k++)
  {
    _pidPose[k]->SetTunings(_kpPose[k], _kiPose[k], _kdPose[k]);
    _pidPose[k]->Compute();
  }
}


void Platform::twistControl()
{
  for (int k = 0; k < NB_AXIS; k++)
  {
    _pidTwist[k]->SetTunings(_kpTwist[k], _kiTwist[k], _kdTwist[k]);
    _pidTwist[k]->Compute();
  }
}


void Platform::setWrenches()
{
  for(int k = 0; k <NB_AXIS; k++)
  {
    if(k<2)
    {
      setForce(_commands[k], _motorsPins[k], 1, k);
    }
    else
    {
      setTorque(_commands[k], _motorsPins[k], 1, k);
    }  
  }
}


void Platform::setForce(float force, int pin, int sign, int axis)
{
  force /= cos(PI/3.0f);
  float escon_torque = force * BELT_PULLEY_R; //! Convert from torque to force
  setTorque(escon_torque, pin, sign, axis);
}


void Platform::setTorque(float torque, int pin, int sign, int axis)
{
  float ki = 0.0f, iMax = 0.0f, reduction = 0.0;

  switch (axis)
  {
    case PITCH:
    {
      ki = CURRENT_K_P;
      iMax = C_CURRENT_MAX_P;
      reduction = PITCH_REDUCTION_R;
      break;
    }
    case X:
    {
      ki = CURRENT_K_XY;
      iMax = C_CURRENT_MAX_XY;
      reduction = 1.0f;
      break;
    }
    case Y:
    {
      ki = CURRENT_K_XY;
      iMax = C_CURRENT_MAX_XY;
      reduction = 1.0f;
      break;
    }
  }

  double escon_current = (torque * 1000 / ki) / reduction;
  escon_current = (escon_current > iMax ? iMax : (escon_current < -iMax ? -iMax : escon_current)); 
  int escon_current_PWM = map(escon_current, -iMax, iMax, 410, 3685);
  escon_current_PWM *= sign;
  analogWrite(pin, escon_current_PWM);
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


void Platform::allReset()
{
  if (_switchesState[X] == 1 && _switchesState[Y] == 1 && _switchesState[PITCH] == 1)
  {
    for(int k = 0; k <NB_AXIS; k++)
    {
      _encoders[k]->QEC_offset();
    }

    _poseOffsets[X] = HOMING_OFFSET_X;
    _poseOffsets[Y] = HOMING_OFFSET_Y;
    _poseOffsets[PITCH] = HOMING_OFFSET_PITCH;
    // TODO add other offsets !!!!!
  }
}


void Platform::updateFootInput(const custom_msgs::FootInputMsg &msg)
{
  //TODO: Check if it should be the commands or the measured forces and torques
  me->_commands[X] = msg.FxDes;
  me->_commands[Y] = msg.FyDes;
  me->_commands[PITCH] = msg.TphiDes;
  /* s_torqueR = s_wrench_msg.torque.y;
    s_torqueY = s_wrench_msg.torque.z */
  me->_state = (Platform::State) msg.stateDes;
}


void Platform::pubFootOutput()
{
  _msgFootOutput.id = PLATFORM_ID;
  _msgFootOutput.stamp = _nh.now();
  _msgFootOutput.x = _pose[X];
  _msgFootOutput.y = _pose[Y];
  _msgFootOutput.phi = _pose[PITCH];
  _msgFootOutput.theta = _pose[ROLL];
  _msgFootOutput.psi = _pose[YAW];
  _msgFootOutput.Fx = _commands[X];
  _msgFootOutput.Fy = _commands[Y];
  _msgFootOutput.Tphi = _commands[PITCH];
  _msgFootOutput.Ttheta = _commands[ROLL];
  _msgFootOutput.Tpsi = _commands[YAW];
  _msgFootOutput.vx = _twist[X];
  _msgFootOutput.vy = _twist[Y];
  _msgFootOutput.wphi = _twist[PITCH];
  _msgFootOutput.wtheta = _twist[ROLL];
  _msgFootOutput.wpsi = _twist[YAW];
  _msgFootOutput.state = _state;
  _pubFootOutput->publish(&_msgFootOutput);
}