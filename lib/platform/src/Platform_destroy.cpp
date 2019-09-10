#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

Platform::~Platform()
{
  _innerTimer.~Timer();
  for(int k = 0; k <NB_AXIS; k++)
  {
    delete (_poseFilters[k]);
    delete (_wrenchMFilters[k]);
    delete (_twistFilters[k]);
    _pidPose[k]->~PID();
    delete (_pidPose[k]);
    _pidTwist[k]->~PID();
    delete (_pidTwist[k]);
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