#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! 1
void Platform::positionAllControl(EffortComp Component)
{
  //Compute the PID

  for (uint i = 0; i < NB_AXIS; i++)
  { 
    positionAxisControl(Component,i);
  }
 
}

//! 2
void Platform::positionAxisControl(EffortComp Component, int axis)
{
    if ((axis==X)&&((_ros_controllerType==POSITION_ONLY)||(_ros_controllerType==SPEED_POSITION_CASCADE))){
      _pidPosition[X]->setOutputLimits(-EFFORT_LIMIT_DEFAULT_X,EFFORT_LIMIT_DEFAULT_X); //!N
    }

    if ((axis==Y)&&((_ros_controllerType==POSITION_ONLY)||(_ros_controllerType==SPEED_POSITION_CASCADE))){
      _pidPosition[Y]->setOutputLimits(-EFFORT_LIMIT_DEFAULT_Y,EFFORT_LIMIT_DEFAULT_Y); //!N
    }

    if (_platform_state==TELEOPERATION)
      {
        if ((axis >= 2) && ((_ros_controllerType == POSITION_ONLY) || (_ros_controllerType == SPEED_POSITION_CASCADE)))
        {
            _pidPosition[axis]->setOutputLimits(-USER_MAX_EFFORTS[axis], USER_MAX_EFFORTS[axis]);
          //!Nm
        }
      }
    else
    {
        if ((axis >= 2) && ((_ros_controllerType == POSITION_ONLY) || (_ros_controllerType == SPEED_POSITION_CASCADE)))
        {
            _pidPosition[axis]->setOutputLimits(-3, 3);
        }
    }
    


    if (_ros_controllerType==SPEED_POSITION_CASCADE){
      _positionD[axis]=_speedCtrlOut[axis];
    }

    if (abs(_positionD[axis])>0.005) //! Only interpolate if greater than 5 millimiters / millidegrees
    {
      _positionD_filtered[axis] =
      _posDesiredFilters[axis].update(_positionD[axis]);
      _positionD_filtered[axis] =
             clamp(_positionD_filtered[axis], -C_WS_LIMITS[axis], C_WS_LIMITS[axis]);
    }
    else
    {
      _positionD_filtered[axis] = _positionD[axis];
    }
    
    _pidPosition[axis]->setTunings(_kpPosition[axis], _kiPosition[axis],
                                   _kdPosition[axis]);
    _pidPosition[axis]->compute();

    if ((_ros_controllerType == POSITION_ONLY) ||
        (_ros_controllerType == SPEED_POSITION_CASCADE)) {
      _effortD_ADD[Component][axis] = _positionCtrlOut[axis];
    }
}

//! 3
void Platform::speedAxisControl(EffortComp Component, int axis)
{
    if ((axis==X)&&((_ros_controllerType==SPEED_ONLY)||(_ros_controllerType==POSITION_SPEED_CASCADE))){
      _pidSpeed[X]->setOutputLimits(-EFFORT_LIMIT_DEFAULT_X,EFFORT_LIMIT_DEFAULT_X); //!N
    }

     if ((axis==Y)&&((_ros_controllerType==SPEED_ONLY)||(_ros_controllerType==POSITION_SPEED_CASCADE))){
      _pidSpeed[Y]->setOutputLimits(-EFFORT_LIMIT_DEFAULT_Y,EFFORT_LIMIT_DEFAULT_Y); //!N
    }

    if ((axis>=2)&&((_ros_controllerType==SPEED_ONLY)||(_ros_controllerType==POSITION_SPEED_CASCADE))){
      _pidSpeed[axis]->setOutputLimits(-3,3); //!Nm
    }

    if (_ros_controllerType==POSITION_SPEED_CASCADE){
      _speedD[axis]=_positionCtrlOut[axis];
    }


     _pidSpeed[axis]->setTunings(_kpSpeed[axis], _kiSpeed[axis], _kdSpeed[axis]);
     _pidSpeed[axis]->compute();

    if ((_ros_controllerType==SPEED_ONLY)||(_ros_controllerType==POSITION_SPEED_CASCADE)){
      _effortD_ADD[NORMAL][axis]=_speedCtrlOut[axis];
    }
}

//! 4
void Platform::speedAllControl(EffortComp Component)
{

  for (uint i = 0; i < NB_AXIS; i++)
  {  
    speedAxisControl(Component,i);
  }
}

//! 5
void Platform::gotoPointAxis(int axis_, float point)
{
  if (_ros_flagDefaultControl){ gotoPointGainsDefault(axis_);}
  _positionD[axis_]=point;
  positionAxisControl(NORMAL,axis_);
}

//! 6
void Platform::gotoPointAll(float pointX, float pointY, float pointPITCH, float pointROLL, float pointYAW)
{
  if (_ros_flagDefaultControl){ gotoPointGainsDefault(-1);}
  _positionD[X]=pointX;
  _positionD[Y]=pointY;
  _positionD[PITCH]=pointPITCH;
  _positionD[ROLL]=pointROLL;
  _positionD[YAW]=pointYAW;
  positionAllControl(NORMAL);
}

//! 7
void Platform::gotoPointGainsDefault(int axis_)
{
  if (axis_==-1){
    for (uint k=0; k<NB_AXIS; k++ )
    {
      gotoPointGainsDefault(k);
    }
  }

  else{
    switch(axis_){
        case(X): {_kpPosition[X]=GT_KP_POSITION_X;_kiPosition[X]=GT_KI_POSITION_X;_kdPosition[X]=GT_KD_POSITION_X; break;}
        case(Y): {_kpPosition[Y]=GT_KP_POSITION_Y;_kiPosition[Y]=GT_KI_POSITION_Y;_kdPosition[Y]=GT_KD_POSITION_Y; break;}
        case(PITCH): {_kpPosition[PITCH]=GT_KP_POSITION_PITCH;_kiPosition[PITCH]=GT_KI_POSITION_PITCH;_kdPosition[PITCH]=GT_KD_POSITION_PITCH;break;}
        case(ROLL): {_kpPosition[ROLL]=GT_KP_POSITION_ROLL;_kiPosition[ROLL]=GT_KI_POSITION_ROLL;_kdPosition[ROLL]=GT_KD_POSITION_ROLL;break;}
        case(YAW): {_kpPosition[YAW]=GT_KP_POSITION_YAW;_kiPosition[YAW]=GT_KI_POSITION_YAW;_kdPosition[YAW]=GT_KD_POSITION_YAW;break;}
      }
  }
}