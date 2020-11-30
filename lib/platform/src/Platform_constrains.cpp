#include "Platform.h"

//! 1
void Platform::wsConstrains(int axis_)
{
  if (axis_==-1) //! All axis constrained
  {
    for (uint k = 0; k<NB_AXIS; k++){
      wsConstrains(k);
    } 
  }
  else
  {
    if (!_platform_flagDefaultControl) {
        _virtualWall(axis_)=fabs(_ros_position[axis_]); //! A symmetric wall will be built on the set position
    }
    
    if (( _position(axis_) > _virtualWall(axis_) ))
    {
          _flagInWsConstrains[axis_]=true;
           _positionD(axis_) = _virtualWall(axis_);
           positionAxisControl(CONSTRAINS, axis_);
    }
    else if (( _position(axis_) < -_virtualWall(axis_) ))
    {
          _flagInWsConstrains[axis_]=true;
           _positionD(axis_) = -_virtualWall(axis_);
           positionAxisControl(CONSTRAINS, axis_);
    }
    else    
    {
          _pidPosition[axis_]->reset();
          _flagInWsConstrains[axis_]=false;
    }
  }
}


//! 3
void Platform::wsConstrainsGainsDefault()
{
  _platform_kpPosition = Eigen::Map<const Eigen::MatrixXf>(C_WS_PID_GAINS[KP], NB_AXIS, 1);
  _platform_kiPosition = Eigen::Map<const Eigen::MatrixXf>(C_WS_PID_GAINS[KI], NB_AXIS, 1);
  _platform_kdPosition = Eigen::Map<const Eigen::MatrixXf>(C_WS_PID_GAINS[KD], NB_AXIS, 1);               
}