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
    totalWrenchDClear(-1);
    poseCtrlClear(-1);
    twistCtrlClear(-1);
  for(int k = 0; k < NB_AXIS; k++)
  {
    _poseFilters[k]->reset();
    _twistFilters[k]->reset();
    _wrenchMFilters[k]->reset();
  }

}
