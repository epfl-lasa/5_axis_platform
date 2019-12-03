#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! #1

#define ADC 1
#define INTEGRAL_TERM 2
#define EFFORT_M ADC 

void Platform::getMotion()
{
  getPosition();
  getSpeed();
}

#if (EFFORT_M == ADC)
//! #2
// void Platform::readActualEffort() //! ADC
// {
//   if(_innerCounterADC<NB_AXIS && (_timestamp-_analogReadStamp)>=ANALOG_SAMPLING_TIME)
//   {
//     if (_innerCounterADC>=ROLL){
//       _effortM[_innerCounterADC+2]=map(_motorCurrents[_innerCounterADC]->read()*_motorSign[_innerCounterADC],0.1,0.9,-_maxEffort[_innerCounterADC],_maxEffort[_innerCounterADC]);
//     }
//     else
//     {
//       _effortM[_innerCounterADC]=map(_motorCurrents[_innerCounterADC]->read()*_motorSign[_innerCounterADC],0.1,0.9,-_maxEffort[_innerCounterADC],_maxEffort[_innerCounterADC]);
//       _effortM[_innerCounterADC]=_effortMFilters[_innerCounterADC]->update(_effortM[_innerCounterADC]);
//     }
    
//   if(_innerCounterADC==YAW){
//     // Adapt roll and yaw angles due to differential mechanism
//     _effortM[ROLL]= (_effortM[ROLL+2]-_effortM[YAW+2])/2.0f;
//     _effortM[YAW] = (_effortM[ROLL+2]+_effortM[YAW+2])/2.0f;
//     _effortM[ROLL]=_effortMFilters[ROLL]->update(_effortM[ROLL]);
//     _effortM[YAW]=_effortMFilters[YAW]->update(_effortM[YAW]);
//     _innerCounterADC=0;
//     _analogReadStamp=_timestamp;
//   }
  
//   _innerCounterADC++;
//   }

// }

void Platform::readActualEffort() //! ADC
{
  if ((_timestamp - _analogReadStamp) >= ANALOG_SAMPLING_TIME)
  {
    
    float e_meas[5];
    float e_raw[5];

    for (uint k = 0; k < NB_AXIS; k++)
    {
      e_meas[k] = ADC_SIGN[k]*_motorCurrents[k]->read();
      e_raw[k] = (e_meas[k] - 0.1f) * (2.0f * _maxEffort[k]) / (0.8f) - 1.0f * _maxEffort[k];
      e_raw[k] *= ADC_EFFORT_SCALE[k];
    }

    _effortM[X] = _effortMFilters[X]->update(e_raw[X]);
    _effortM[Y] = _effortMFilters[Y]->update(e_raw[Y]);
    _effortM[PITCH] = _effortMFilters[PITCH]->update(e_raw[PITCH]);
    _effortM[ROLL] = _effortMFilters[ROLL]->update((e_raw[ROLL] - e_raw[YAW]) / 2.0f);
    _effortM[YAW] = _effortMFilters[YAW]->update((e_raw[ROLL] + e_raw[YAW]) / 2.0f);

    if (!_flagBiasADCOk)
      {
       if (_innerCounterADC>200)
       { 
          for (uint k = 0; k < NB_AXIS; k++)
          {
            _adc_sum[k] += _effortM[k];
          }
       }
        _innerCounterADC++;
      }

    if (_innerCounterADC == 500)
    {
      for (uint k = 0; k < NB_AXIS; k++)
        {
          _effortMFilters[k]->setBias(_adc_sum[k] / (_innerCounterADC - 200) + ADC_USER_BIAS[k]);
          _adc_sum[k]=0.0f;
        }
        _flagBiasADCOk=true;
        _innerCounterADC=0;
    }



    _analogReadStamp = _timestamp;
  }
}

#else
void Platform::readActualEffort() 
{
  for (uint k=0; k<NB_AXIS; k++)
  {
    // if (flagPositionInControl())
    //  { _effortM[k]=_pidPosition[k]->getIntegralTerm();}
    // else if (flagSpeedInControl())
    //   { _effortM[k]=_pidSpeed[k]->getIntegralTerm();}
    // else
    // {
    //   _effortM[k] = 0.0;
    // }
    _effortM[k] = _pidPosition[k]->getIntegralTerm();
  }
} 
#endif

//! #3
void Platform::getPosition()
{
  _spi->lock(); 
  for (uint k = 0; k < NB_AXIS; k++)
  {
    _encoders[k]->QEC_getPosition(_spi);
    _position[k] = _encoders[k]->outDimension + _positionOffsets[k];
    _position[k] = _positionFilters[k]->update(_position[k]);
  }
  _spi->unlock(); 
  // Adapt roll and yaw angles due to differential mechanism
  float enc1 = _position[ROLL];
  float enc2 = _position[YAW];
  _position[ROLL]= (enc1-enc2)/2.0f;
  _position[YAW] = (enc1+enc2)/2.0f;
}
//! #4
void Platform::getSpeed()
{
  if ((_timestamp-_speedSamplingStamp)>=VELOCITY_PID_SAMPLE_P)
  {
    for (uint k = 0; k < NB_AXIS; k++)
    {
      _speed[k] = (_position[k] - _positionPrev[k]) / (VELOCITY_PID_SAMPLE_P * 1e-6f);
      _speed[k] = _speedFilters[k]->update(_speed[k]);
      _positionPrev[k] = _position[k];
      _speedSamplingStamp=_timestamp;
    }
  }
}
