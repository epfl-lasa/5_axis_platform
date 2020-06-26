#include "Platform.h"


//! 1
void Platform::positionAllReset()
{
  if (_switchesState[X] == 1 && _switchesState[Y] == 1 && _switchesState[PITCH] == 1)
  {
    _positionOffsets(X) = HOMING_OFFSET_X;
    _positionOffsets(Y) = HOMING_OFFSET_Y;
    _positionOffsets(PITCH) = HOMING_OFFSET_PITCH;
    _positionOffsets(ROLL) = HOMING_OFFSET_ROLL;
    _positionOffsets(YAW) = HOMING_OFFSET_YAW;
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
        //sprintf(_logMsg, "%s : LEAVING HOMING STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg);
        _enterStateOnceFlag[HOMING]=false;   
        limitSwitchesClear();       
        //! Finally resets the effort commands given by this controller. 
        compEffortClear(-1,NORMAL); //! Clear the normal dimension of the 
        speedCtrlClear(-1);
        break;
      }

      case(CENTERING):
      {
        ////sprintf(_logMsg, "%s : LEAVING CENTERING STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg); 
        _enterStateOnceFlag[CENTERING]=false;
        compEffortClear(-1,NORMAL);
        positionCtrlClear(-1);
        break;
      }
      case(TELEOPERATION):
      {
        //sprintf(_logMsg, "%s : LEAVING TELEOPERATION STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg);
        _enterStateOnceFlag[TELEOPERATION]=false;
        totalEffortDClear(-1);
        positionCtrlClear(-1);
        speedCtrlClear(-1);
        break;
      }

      case(ROBOT_STATE_CONTROL):
      {
        //sprintf(_logMsg, "%s : LEAVING ROBOT_STATE_CONTROL STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg);
        _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;
        totalEffortDClear(-1);
        positionCtrlClear(-1);
        speedCtrlClear(-1);
        break;
      }


      case(EMERGENCY):{
        _enterStateOnceFlag[EMERGENCY]=false;
        //sprintf(_logMsg, "%s : LEAVING EMERGENCY STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg);
        break;
      }
      case(STANDBY): {_enterStateOnceFlag[STANDBY]=false;
        //sprintf(_logMsg, "%s : LEAVING STANDBY STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg);
        break;
      }
      case(RESET_UC):{break;}
    }
}



void Platform::resetControllers(Controller controllerType)
{
  switch (controllerType) {
    
  case (TORQUE_CTRL): {
    compEffortClear(-1, NORMAL);
    break;
    }
    case(POSITION_CTRL):
      {
        for (uint k=0; k<NB_AXIS; k++)
        {
           _pidPosition[k]->reset();  _posDesiredFilters[k].reset();   
        }
        break;
      }
    case (SPEED_CTRL): {
        for (uint k = 0; k < NB_AXIS; k++) {
          _pidSpeed[k]->reset();
        }
        break;
      }
    case (FS_CTRL): 
    {
      for (int k = 0; k < NB_AXIS; k++) {
        _pidForceSensor[k]->reset();
      }
      break;
    }  
  }
}

    //! 4
void Platform::positionCtrlClear(int axis_) {
  if (axis_==-1){
    for (int k = 0; k < NB_AXIS; k++) {
      positionCtrlClear(k);
    }
  }
  else
  {
    _positionD(axis_)=_position(axis_);  
    _positionCtrlOut(axis_)=0.0f;
  }
  
}

//! 5
void Platform::speedCtrlClear(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++)
      {
        speedCtrlClear(k);
      }
  }
  else
  {
    _speedD(axis_)=_speed(axis_);
    _speedCtrlOut(axis_)=0.0f;
    
  }
  
}

//! 6
void Platform::compEffortClear(int axis_, EffortComp component_)
{
  if (axis_==-1)
  {
    _effortD_ADD.col(component_).setConstant(0.0f);
     return;
  }
  else
  {
    _effortD_ADD(axis_, component_) = 0.0f;
  }
}

//! 7
void Platform::totalEffortDClear(int axis_)
{
  if (axis_==-1)
    {
     // _effortD.setConstant(0.0f);
      _effortD_ADD.setConstant(0.0f);
    }
  else
    {  
      _effortD_ADD.row(axis_).setConstant(0.0f);
    }
}