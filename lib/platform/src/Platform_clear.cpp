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
  switch(_platform_state)
    {
      case(HOMING):
      {
        sprintf(_logMsg, "%s : LEAVING HOMING STATE", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        _enterStateOnceFlag[HOMING]=false;   
        limitSwitchesClear();       
        //! Finally resets the effort commands given by this controller. 
        compEffortClear(-1,NORMAL); //! Clear the normal dimension of the 
        speedCtrlClear(-1);
        break;
      }

      case(CENTERING):
      {
        sprintf(_logMsg, "%s : LEAVING CENTERING STATE", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg); 
        _enterStateOnceFlag[CENTERING]=false;
        compEffortClear(-1,NORMAL);
        positionCtrlClear(-1);
        break;
      }
      case(TELEOPERATION):
      {
        sprintf(_logMsg, "%s : LEAVING TELEOPERATION STATE", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        _enterStateOnceFlag[TELEOPERATION]=false;
        totalEffortDClear(-1);
        positionCtrlClear(-1);
        speedCtrlClear(-1);
        break;
      }

      case(ROBOT_STATE_CONTROL):
      {
        sprintf(_logMsg, "%s : LEAVING ROBOT_STATE_CONTROL STATE", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;
        totalEffortDClear(-1);
        positionCtrlClear(-1);
        speedCtrlClear(-1);
        break;
      }


      case(EMERGENCY):{
        _enterStateOnceFlag[EMERGENCY]=false;
        sprintf(_logMsg, "%s : LEAVING EMERGENCY STATE", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        break;
      }
      case(STANDBY): {_enterStateOnceFlag[STANDBY]=false;
        sprintf(_logMsg, "%s : LEAVING STANDBY STATE", Platform_Names[PLATFORM_ID]);
        _nh.loginfo(_logMsg);
        break;
      }
      case(RESET):{break;}
    }
}



void Platform::resetControllers()
{
  switch(_platform_controllerType)
  {
    case(TORQUE_ONLY):
    {
      compEffortClear(-1, NORMAL);
      break;
    }
    case(POSITION_ONLY):
      {
        for (uint k=0; k<NB_AXIS; k++)
        {
           _pidPosition[k]->reset();  _posDesiredFilters[k].reset();   
          // _positionPIDIn[k]->reset();
        }
        break;
      }
    case (SPEED_ONLY): {
        for (uint k = 0; k < NB_AXIS; k++) {
          _pidSpeed[k]->reset();
          // _speedPIDIn[k]->reset();
        }
        break;
      }
    case (SPEED_POSITION_CASCADE):
    case (POSITION_SPEED_CASCADE): {
      for (uint k = 0; k < NB_AXIS; k++) {
         _pidPosition[k]->reset();  _posDesiredFilters[k].reset();    
        _pidSpeed[k]->reset();
        // _positionPIDIn[k]->reset();
        // _speedPIDIn[k]->reset();
      }
      break;
      }
  }
}

    //! 4
void Platform::positionCtrlClear(int axis_) {
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