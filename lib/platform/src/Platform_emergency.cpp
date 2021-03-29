#include "Platform.h"


//! 1
void Platform::emergencyCallback()
{
  me->_flagEmergencyCalled=true;
}

//! 2
void Platform::releasePlatform()
{
    _enableMotors->write(0);
    totalEffortDClear(-1);
    positionCtrlClear(-1);
    speedCtrlClear(-1);
    resetControllers(POSITION_CTRL);
    resetControllers(SPEED_CTRL);
    resetControllers(FS_CTRL);
  for(int k = 0; k < NB_AXIS; k++)
  {
    _speedFilters[k].reset(0.0f);
  }
}
