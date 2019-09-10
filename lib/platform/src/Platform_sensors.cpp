#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! #1

void Platform::getMotion()
{
  getPose();
  getTwist();
}

//! #2
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

//! #3
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
//! #4
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
