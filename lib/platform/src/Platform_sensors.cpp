#include "Platform.h"

//! #1


void Platform::getMotion()
{
  getPosition();
  getSpeed();
  getAcceleration();
}

#if (EFFORT_M == ADC)

//! #2
void Platform::readActualEffort() //! ADC
{
  if ((_timestamp - _analogReadStamp) >= ANALOG_SAMPLING_TIME)
  {
    
    float e_meas[5];
    float e_raw[5];

    for (uint k = 0; k < NB_AXIS; k++)
    {
      e_meas[k] = ADC_SIGN[k]*_motorCurrents[k]->read();
      e_raw[k] = (e_meas[k] - 0.1f) * (2.0f * MAX_EFFORT[k]) / (0.8f) - 1.0f * MAX_EFFORT[k];
      e_raw[k] *= ADC_EFFORT_SCALE[k];
    }

    _effortM(X) = _effortMFilters[X].update(e_raw[X]);
    _effortM(Y) = _effortMFilters[Y].update(e_raw[Y]);
    _effortM(PITCH) = _effortMFilters[PITCH].update(e_raw[PITCH]);
    _effortM(ROLL) = _effortMFilters[ROLL].update((e_raw[ROLL] - e_raw[YAW]) / 2.0f);
    _effortM(YAW) = _effortMFilters[YAW].update((e_raw[ROLL] + e_raw[YAW]) / 2.0f);

    if (!_flagBiasADCOk)
      {
       if (_innerCounterADC>200)
       { 
          for (uint k = 0; k < NB_AXIS; k++)
          {
            _adc_sum[k] += _effortM(k);
          }
       }
        _innerCounterADC++;
      }

    if (_innerCounterADC == 500)
    {
      for (uint k = 0; k < NB_AXIS; k++)
        {
          _effortMFilters[k].setBias(_adc_sum[k] / (_innerCounterADC - 200) + ADC_USER_BIAS[k]);
          _adc_sum[k]=0.0f;
        }
        _flagBiasADCOk=true;
        _innerCounterADC=0;
    }



    _analogReadStamp = _timestamp;
  }
}
#endif

//! #3
void Platform::getPosition()
{
  if ((_timestamp - _posSamplingStamp) >= (uint32_t)POSITION_PID_SAMPLE_P)
  {
    float encoders_out[NB_AXIS] = {0.0f,0.0f,0.0f,0.0f,0.0f};
    _spi->lock();
      for (uint k = 0; k < NB_AXIS; k++)
      {
        encoders_out[k] = _encoders[k]->QEC_getPosition(_spi);
      }
    _spi->unlock();
    // Adapt roll and yaw angles due to differential mechanism
      _position(Y) = encoders_out[Y] + _positionOffsets (Y);
      _position(X) = encoders_out[X] + _positionOffsets(X);
      _position(PITCH) = encoders_out[PITCH] + _positionOffsets(PITCH);
      _position(ROLL) = (encoders_out[ROLL] - encoders_out[YAW]) / 2.0f + _positionOffsets (ROLL);
      _position(YAW) = (encoders_out[ROLL] + encoders_out[YAW]) / 2.0f + + _positionOffsets (YAW);
    _posSamplingStamp = _timestamp;
  }
  //
  if (_flagCalculateSinCos)
  {
    _c_theta = cos(_position(PITCH) );
    _c_phi = cos(_position(ROLL) );
    _c_psi = cos(_position(YAW));
    _s_theta = sin(_position(PITCH) );
    _s_phi = sin(_position(ROLL) );
    _s_psi = sin(_position(YAW));
  }
}
//! #4
void Platform::getSpeed()
{
  if ((_timestamp-_speedSamplingStamp)>=(uint32_t)VELOCITY_PID_SAMPLE_P)
  {
    _speed = (_position - _positionPrev) * invSpeedSampT;
    for (uint k = 0; k < NB_AXIS; k++) // to change
    {
      _speed(k) = _speedFilters[k].update(_speed(k));
    }
    _positionPrev = _position;
    _speedSamplingStamp = _timestamp;
  }
}

//! #5
void Platform::getAcceleration()
{
  if ((_timestamp - _accSamplingStamp) >= (uint32_t)ACC_SAMPLE_P)
  {
    _acceleration = (_speed - _speedPrev) * invAccSampT;
    for (uint k = 0; k < NB_AXIS; k++) // to change
    {    
      _acceleration(k) = _accFilters[k].update(_acceleration(k));
    }
    _speedPrev = _speed;
    _accSamplingStamp = _timestamp;
    _flagSpeedSampledForCoriolis = true;
  }
}
