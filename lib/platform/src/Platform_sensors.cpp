#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! #1

void Platform::getMotion()
{
  getPosition();
  getSpeed();
}

//! #2
void Platform::readActualEffort() //! ADC
{
  if(_innerCounter<NB_AXIS && (_timestamp-_analogReadStamp)>=ANALOG_SAMPLING_TIME)
  {
    if (_innerCounter>=ROLL){
      _effortM[_innerCounter+2]=map(_motorCurrents[_innerCounter]->read()*_motorSign[_innerCounter],0.1,0.9,-_maxEffort[_innerCounter],_maxEffort[_innerCounter]);
    }
    else
    {
      _effortM[_innerCounter]=map(_motorCurrents[_innerCounter]->read()*_motorSign[_innerCounter],0.1,0.9,-_maxEffort[_innerCounter],_maxEffort[_innerCounter]);
      _effortM[_innerCounter]=_effortMFilters[_innerCounter]->update(_effortM[_innerCounter]);
    }
    
  if(_innerCounter==YAW){
    // Adapt roll and yaw angles due to differential mechanism
    _effortM[ROLL]= (_effortM[ROLL+2]-_effortM[YAW+2])/2.0f;
    _effortM[YAW] = (_effortM[ROLL+2]+_effortM[YAW+2])/2.0f;
    _effortM[ROLL]=_effortMFilters[ROLL]->update(_effortM[ROLL]);
    _effortM[YAW]=_effortMFilters[YAW]->update(_effortM[YAW]);
    _innerCounter=0;
  }
  
  _innerCounter++;
  }

}

//! #3
void Platform::getPosition()
{
  _spi->lock(); 
  for (int k = 0; k < NB_AXIS; k++)
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
    for (int k = 0; k < NB_AXIS; k++)
    {
      _speed[k] = (_position[k] - _positionPrev[k]) / (VELOCITY_PID_SAMPLE_P * 1e-6f);
      _speed[k] = _speedFilters[k]->update(_speed[k]);
      _positionPrev[k] = _position[k];
      _speedSamplingStamp=_timestamp;
    }
  }
}
