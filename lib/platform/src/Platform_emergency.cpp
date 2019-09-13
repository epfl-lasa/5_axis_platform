#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>


//! 1
void Platform::emergencyCallback()
{
  me->_allEsconOk=0;
  me->_state=EMERGENCY; 
}

//! 2
void Platform::releasePlatform()
{
    totalEffortDClear(-1);
    positionCtrlClear(-1);
    speedCtrlClear(-1);
  for(int k = 0; k < NB_AXIS; k++)
  {
    _positionFilters[k]->reset();
    _speedFilters[k]->reset();
    _effortMFilters[k]->reset();
  }

}