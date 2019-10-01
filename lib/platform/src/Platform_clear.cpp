#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>


//! 1
void Platform::positionAllReset()
{
  if (_switchesState[X] == 1 && _switchesState[Y] == 1 && _switchesState[PITCH] == 1)
  {
    _positionOffsets[X] = HOMING_OFFSET_X;
    _positionOffsets[Y] = HOMING_OFFSET_Y;
    _positionOffsets[PITCH] = HOMING_OFFSET_PITCH;
    _spi->lock();
      for(int k = 0; k <NB_AXIS; k++)
      {
      _encoders[k]->QEC_clear(_spi);
      }
    _spi->unlock();
  }
}

//! 2
void Platform::limitSwitchesClear()
{
  for (uint k=0; k<NB_SWITCHES; k++) 
  {
    _switchesState[k] = 0;
  }
}

//! 3
void Platform::clearLastState()
{
  switch(_lastState)
    {
      case(HOMING):
      {
        _nh.loginfo("LEAVING HOMING STATE"); 
        _enterStateOnceFlag[HOMING]=false;   
        limitSwitchesClear();       
        //! Finally resets the effort commands given by this controller. 
        compEffortClear(-1,NORMAL); //! Clear the normal dimension of the 
        speedCtrlClear(-1);
        break;
      }

      case(CENTERING):
      {
        _nh.loginfo("LEAVING CENTERING STATE"); 
        _enterStateOnceFlag[CENTERING]=false;
        compEffortClear(-1,NORMAL);
        positionCtrlClear(-1);
        break;
      }
      case(TELEOPERATION):
      {
        _nh.loginfo("LEAVING TELEOPERATION STATE"); 
        _enterStateOnceFlag[TELEOPERATION]=false;
        totalEffortDClear(-1);
        positionCtrlClear(-1);
        speedCtrlClear(-1);
        break;
      }

      case(ROBOT_STATE_CONTROL):
      {
        _nh.loginfo("LEAVING ROBOT_STATE_CONTROL STATE"); 
        _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;
        totalEffortDClear(-1);
        positionCtrlClear(-1);
        speedCtrlClear(-1);
        break;
      }


      case(EMERGENCY):{_enterStateOnceFlag[EMERGENCY]=false; _nh.loginfo("LEAVING EMERGENCY STATE"); break;}
      case(STANDBY): {_enterStateOnceFlag[STANDBY]=false; _nh.loginfo("LEAVING STANDBY STATE");  break;} 
      case(RESET):{break;}
    }
}

//! 4
void Platform::positionCtrlClear(int axis_)
{
  if (axis_==-1){
    for (uint k=0; k<NB_AXIS; k++ )
    {
      positionCtrlClear(k);
    }
  }
  else
  {
    _positionD[axis_]=0.0f;
    _kpPosition[axis_]=0.0f;
    _kiPosition[axis_]=0.0f;
    _kdPosition[axis_]=0.0f;
    _positionCtrlOut[axis_]=0.0f;
  }
  
}

//! 5
void Platform::speedCtrlClear(int axis_)
{
  if (axis_==-1){
    for (uint k=0; k<NB_AXIS; k++ )
    {
      speedCtrlClear(k);
    }
  }
  else
  {
    _speedD[axis_]=0.0f;
    _kpSpeed[axis_]=0.0f;
    _kiSpeed[axis_]=0.0f;
    _kdSpeed[axis_]=0.0f;
    _speedCtrlOut[axis_]=0.0f;
    
  }
  
}

//! 6
void Platform::compEffortClear(int axis_, Platform::EffortComp component_)
{
  if (axis_==-1)
  {
    for (uint k=0; k<NB_AXIS; k++) 
    { 
      compEffortClear(k, component_); 
    }
    return;
  }
  else
  {
    _effortD_ADD[component_][axis_]=0.0f;
  }
}

//! 7
void Platform::totalEffortDClear(int axis_)
{
  if (axis_==-1)
    {
      for (uint k=0; k<NB_AXIS; k++) 
      { 
        totalEffortDClear(k); 
        _effortD[k] = 0.0f;
      }
    }
  else
  {  
    for (uint j=0; j<NB_EFFORT_COMPONENTS; j++)
    {
      compEffortClear(axis_, (Platform::EffortComp) j);
    }
  }
}