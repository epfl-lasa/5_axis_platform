#include "Platform.h"
#include "definitions.h"

Platform::~Platform()
{
  _innerTimer.~Timer();
  for(int k = 0; k <NB_AXIS; k++)
  {
    _pidPosition[k]->~PID();
    delete (_pidPosition[k]);
    _pidSpeed[k]->~PID();
    delete (_pidSpeed[k]);
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