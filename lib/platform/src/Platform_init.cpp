#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

void Platform::init()
{
    
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
    for(int k = 0; k <NB_AXIS; k++)
    {

     _motors[k]->period_us(200); // PWM (to ESCON) PERIOD 200 us-> 1kHz    
     _esconEnabled[k]->fall(&emergencyCallback);

    if (k<2){
     _pidPosition[k]->setOutputLimits(-25.0, 25.0);
     _pidSpeed[k]->setOutputLimits(-25.0, 25.0);  
    }
     else {
     _pidPosition[k]->setOutputLimits(-3.0, 3.0); //! For the moment 
     _pidSpeed[k]->setOutputLimits(-3.0, 3.0);    
      }

    //}
    _pidPosition[k]->setSampleTime(POSITION_PID_SAMPLE_P*1e-6); //! [us]
    _pidSpeed[k]->setSampleTime(VELOCITY_PID_SAMPLE_P*1e-6);
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