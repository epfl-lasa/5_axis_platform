#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>


//! 1
void Platform::poseAllReset()
{
  if (_switchesState[X] == 1 && _switchesState[Y] == 1 && _switchesState[PITCH] == 1)
  {
    _poseOffsets[X] = HOMING_OFFSET_X;
    _poseOffsets[Y] = HOMING_OFFSET_Y;
    _poseOffsets[PITCH] = HOMING_OFFSET_PITCH;
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
  for (int k=0; k<NB_SWITCHES; k++) 
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
        //! Finally resets the wrench commands given by this controller. 
        compWrenchClear(-1,NORMAL); //! Clear the normal dimension of the 
        twistCtrlClear(-1);
        break; 
      }

      case(CENTERING):
      {
        _nh.loginfo("LEAVING CENTERING STATE"); 
        _enterStateOnceFlag[CENTERING]=false;
        compWrenchClear(-1,NORMAL);
        poseCtrlClear(-1);
        break;
      }
      case(TELEOPERATION):
      {
        _nh.loginfo("LEAVING TELEOPERATION STATE"); 
        _enterStateOnceFlag[TELEOPERATION]=false;
        totalWrenchDClear(-1);
        poseCtrlClear(-1);
        twistCtrlClear(-1);
        break;
      }

      case(ROBOT_STATE_CONTROL):
      {
        _nh.loginfo("LEAVING ROBOT_STATE_CONTROL STATE"); 
        _enterStateOnceFlag[ROBOT_STATE_CONTROL]=false;
        totalWrenchDClear(-1);
        poseCtrlClear(-1);
        twistCtrlClear(-1);
        break;
      }


      case(EMERGENCY):{ _nh.loginfo("LEAVING EMERGENCY STATE"); break;}
      case(STANDBY): {_enterStateOnceFlag[STANDBY]=false; _nh.loginfo("LEAVING STANDBY STATE");  break;} 
      
      case(RESET):{break;}
    }
}

//! 4
void Platform::poseCtrlClear(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      poseCtrlClear(k);
    }
  }
  else
  {
    _poseD[axis_]=0.0f;
    _kpPose[axis_]=0.0f;
    _kiPose[axis_]=0.0f;
    _kdPose[axis_]=0.0f;
    _poseCtrlOut[axis_]=0.0f;
  }
  
}

//! 5
void Platform::twistCtrlClear(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      twistCtrlClear(k);
    }
  }
  else
  {
    _twistD[axis_]=0.0f;
    _kpTwist[axis_]=0.0f;
    _kiTwist[axis_]=0.0f;
    _kdTwist[axis_]=0.0f;
    _twistCtrlOut[axis_]=0.0f;
  }
  
}

//! 6
void Platform::compWrenchClear(int axis_, Platform::WrenchComp component_)
{
  if (axis_==-1)
  {
    for (int k=0; k<NB_AXIS; k++) 
    { 
      compWrenchClear(k, component_); 
    }
    return;
  }
  else
  {
    _wrenchD_ADD[component_][axis_]=0.0f;
  }
}

//! 7
void Platform::totalWrenchDClear(int axis_)
{
  if (axis_==-1)
    {
      for (int k=0; k<NB_AXIS; k++) 
      { 
        totalWrenchDClear(k); 
        _wrenchD[k] = 0.0f;
      }
    }
  else
  {  
    for (int j=0; j<NB_WRENCH_COMPONENTS; j++)
    {
      compWrenchClear(axis_, (Platform::WrenchComp) j);
    }
  }
}