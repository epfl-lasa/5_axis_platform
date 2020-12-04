#include "Platform.h"


//! 1
void Platform::emergencyCallback()
{
  me->_flagEmergencyCalled=true;
}

//! 2
void Platform::releasePlatform()
{
    totalEffortDClear(-1);
    positionCtrlClear(-1);
    speedCtrlClear(-1);
  for(int k = 0; k < NB_AXIS; k++)
  {
    _speedFilters[k].reset();
  }
}
