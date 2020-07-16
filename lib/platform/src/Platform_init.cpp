#include "Platform.h"

void Platform::init()
{
    
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
    for(int k = 0; k <NB_AXIS; k++)
    {
     _motors[k]->period_us(200); // PWM (to ESCON) PERIOD 200 us-> 5kHz    
     _esconEnabled[k]->fall(&emergencyCallback);

    //}
    _pidPosition[k]->setSampleTime(POSITION_PID_SAMPLE_P); //! [us]
    _pidSpeed[k]->setSampleTime(VELOCITY_PID_SAMPLE_P);
    _pidForceSensor[k]->setSampleTime(ACC_SAMPLE_P);
    }
    _pidRCM->setSampleTime(POSITION_PID_SAMPLE_P);
    loadDefaultPIDGains();
    //! Attach interruptions to callbacks on falling edge
    _limitSwitches[X]->fall(&switchCallbackX);
    _limitSwitches[Y]->fall(&switchCallbackY);
    _limitSwitches[PITCH]->fall(&switchCallbackPitch);

    _spi->lock();
    for (uint k = 0; k < NB_AXIS; k++) {
      _encoders[k]->QEC_init((int)k, ENCODER_SCALE[k], ENCODER_SIGN[k], _spi);
  }
  _spi->unlock(); 

  _subFootInput = new ros::Subscriber<custom_msgs::FootInputMsg_v3>(PLATFORM_SUBSCRIBER_NAME, updateFootInput);
  _servChangeState = new ros::ServiceServer<custom_msgs::setStateSrv_v2::Request,custom_msgs::setStateSrv_v2::Response>(SERVICE_CHANGE_STATE_NAME, updateState);
  _servChangeCtrl = new ros::ServiceServer<custom_msgs::setControllerSrv::Request,custom_msgs::setControllerSrv::Response>(SERVICE_CHANGE_CTRL_NAME, updateController);
  
  _pubFootOutput = new ros::Publisher(PLATFORM_PUBLISHER_NAME, &_msgFootOutput);


  //_nh.getHardware()->setBaud(BAUDRATE);
  _nh.initNode();
   Thread::wait(10);
  _nh.advertise(*_pubFootOutput);
  _nh.advertiseService(*_servChangeState);
  _nh.advertiseService(*_servChangeCtrl);
  _nh.subscribe(*_subFootInput);
   Thread::wait(10);
   _timestamp = _innerTimer.read_us();
   _timestep = CTRL_LOOP;
  //_timestamp = _innerTimer.read_us();
  _posSamplingStamp = _timestamp;
  _speedSamplingStamp=_timestamp;
  _accSamplingStamp=_timestamp;
  _vibGenStamp=_timestamp;
  _analogReadStamp=_timestamp;
  _enableMotors->write(0);
  _allEsconOk=0;
  resetEscons();
}


//******************************LIMIT-SWITCHES-CALLBACKS****************

//! WATCH OUT-> THE DEFINITION OF THE INTERRUPTION CALLBACKS HAVE TO BE IN THE SAME C++

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