#include "Platform.h"
#include "definitions.h"

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
void Platform::positionAxisControl(EffortComp Component, int axis) {
  if ((_platform_controllerType == POSITION_CTRL)) {
    if (_platform_state==ROBOT_STATE_CONTROL)
    {
        posInterpolator(axis);
    }
    else
    {
        _positionD_filtered(axis) = _positionD(axis);
    }
      _pidPosition[axis]->compute();
      _effortD_ADD(axis, Component) = _positionCtrlOut(axis);
  }
}

//! 3
void Platform::speedAxisControl(EffortComp Component, int axis) {
  if ((_platform_controllerType == SPEED_CTRL)) {
    _pidSpeed[axis]->compute();
    _effortD_ADD(axis, Component) = _speedCtrlOut[axis];
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
  _positionD(axis_)=point;
  positionAxisControl(NORMAL,axis_);
}

//! 6
void Platform::gotoPointAll(float pointX, float pointY, float pointPITCH, float pointROLL, float pointYAW)
{
  _positionD(X)=pointX;
  _positionD(Y)=pointY;
  _positionD(PITCH)=pointPITCH;
  _positionD(ROLL)=pointROLL;
  _positionD(YAW)=pointYAW;
  positionAllControl(NORMAL);
}

//! 7
void Platform::gotoPointGainsDefault()
{
  _platform_kpPosition = Eigen::Map<const Eigen::MatrixXf>(POS_PID_GAINS_DEFAULT[KP], NB_AXIS, 1);
  _platform_kiPosition = Eigen::Map<const Eigen::MatrixXf>(POS_PID_GAINS_DEFAULT[KI], NB_AXIS, 1);
  _platform_kdPosition = Eigen::Map<const Eigen::MatrixXf>(POS_PID_GAINS_DEFAULT[KD], NB_AXIS, 1);
}

void Platform::speedPIDGainsDefault()
{

  _platform_kpSpeed = Eigen::Map<const Eigen::MatrixXf>(SPEED_PID_GAINS_DEFAULT[KP], NB_AXIS, 1);
  _platform_kiSpeed = Eigen::Map<const Eigen::MatrixXf>(SPEED_PID_GAINS_DEFAULT[KI], NB_AXIS, 1);
  _platform_kdSpeed = Eigen::Map<const Eigen::MatrixXf>(SPEED_PID_GAINS_DEFAULT[KD], NB_AXIS, 1);

}

void Platform::posCtrlLimitsSet() {
      
  for (int k=0; k<NB_AXIS; k++)
  {
    if(_platform_state==TELEOPERATION)
    {
      _pidPosition[k]->setOutputLimits(-SAFETY_MAX_EFFORTS[k],SAFETY_MAX_EFFORTS[k]);
    }
    else
    {
      _pidPosition[k]->setOutputLimits(-SAFETY_MAX_INIT[k],SAFETY_MAX_INIT[k]);
    }
  }
}

void Platform::speedCtrlLimitsSet() {
  for (int k=0; k<NB_AXIS; k++)
  {
    if (_platform_state==HOMING)
    {
      _pidSpeed[k]->setOutputLimits(-SAFETY_MAX_INIT[k],SAFETY_MAX_INIT[k]);
    }
    else
    {
      _pidSpeed[k]->setOutputLimits(-SAFETY_MAX_EFFORTS[k],SAFETY_MAX_EFFORTS[k]);
    }
  }
}

void Platform::posInterpolator(int axis){
   
    if (fabs(_positionD(axis)-_position(axis))>0.005) //! Only interpolate if greater than 5 millimiters / millidegrees
    {
      _positionD_filtered(axis) = _posDesiredFilters[axis].update(_positionD(axis));
      _positionD_filtered(axis) = clip(_positionD_filtered(axis), -C_WS_LIMITS[axis], C_WS_LIMITS[axis]);
    }
    else
    {
      _positionD_filtered(axis) = _positionD(axis);
    }
}

void Platform::loadDefaultPIDGains()
{
  if (_platform_state==TELEOPERATION)
  {
    wsConstrainsGainsDefault();
  }      
  else
  {
    gotoPointGainsDefault();
  }

  speedPIDGainsDefault();
  setPIDGains();
}

void Platform::loadROSPIDGains()
{
  for (int k=0; k<NB_AXIS; k++)
  {
    _platform_kpPosition(k)=_ros_kpPosition[k];
    _platform_kiPosition(k) = _ros_kiPosition[k];
    _platform_kdPosition(k) = _ros_kdPosition[k];
    _platform_kpSpeed(k) = _ros_kpSpeed[k];
    _platform_kiSpeed(k) = _ros_kiSpeed[k];
    _platform_kdSpeed(k) = _ros_kdSpeed[k];
  }
  setPIDGains();
}

void Platform::setPIDGains()
{
  for (int axis=0; axis<NB_AXIS; axis++)
  {
    _pidPosition[axis]->setTunings(_platform_kpPosition(axis),
                                  _platform_kiPosition(axis),
                                  _platform_kdPosition(axis));

    _pidSpeed[axis]->setTunings(_platform_kpSpeed(axis),
                               _platform_kiSpeed(axis),
                               _platform_kdSpeed(axis));
  }
}