#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! 1
void Platform::wsConstrains(int axis_)
{
  if (axis_==-1) //! All axis constrained
  {
    for (int k = 0; k<NB_AXIS; k++){
      wsConstrains(k);
    }
  }
  else
  {
    float wall = _c_wsLimits[axis_];
    
    if (_ros_flagDefaultControl) {
      wsConstrainsDefault(axis_);
    }
    else
    {
       wall=fabs(_ros_position[axis_]); //! A symmetric wall will be built on the set position
       _kiPosition[axis_]=0.0f;
    }

    _positionD[axis_] = _position[axis_] >= wall ? wall : (_position[axis_] <= -wall ? -wall: 0.0f);
    
    if ( _position[axis_] >= wall || _position[axis_] <= -wall )
      {
        _flagInWsConstrains=true;
        // if ( ((_position[axis_] <= 0.0) && (_speed[axis_] <= 0.0)) || ((_position[axis_] >= 0.0) && (_speed[axis_] >= 0.0)) ) 
        // {
          positionAxisControl(CONSTRAINS,axis_);
        // }
      }
      else    
      {
        _flagInWsConstrains=false;
        _effortD_ADD[CONSTRAINS][axis_]=0.0f;
      }
  }
}


//! 2
void Platform::motionDamping(int axis_)
{ 
  if (axis_==-1) //! All axis constrained
  {
    for (int k = 0; k<NB_AXIS; k++){
      motionDamping(k);
    }
    return;
  }
  else
  {

    if (!_flagInWsConstrains) //! The motion damping only applies outside the walls (constrains)
    {
      if (_ros_flagDefaultControl) {motionDampingGainsDefault(axis_);}
      else
      {
        _kiSpeed[axis_]=0.0f; //! overwrite the i and d constants sent from ROS (because it is teleoperation mode)
        _kdSpeed[axis_]=0.0f;
      }
      _speedD[axis_]=0.0f; //! the target is zero speed
      speedAxisControl(CONSTRAINS, axis_);
    }   
  }
}

//! 3
void Platform::wsConstrainsDefault(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      wsConstrainsDefault(k);
    }
  }
  else{
    switch(axis_){
        case(X): {_c_wsLimits[X] = C_WS_RANGE_X;_kpPosition[X]=C_WS_KP_POSITION_X;_kiPosition[X]=C_WS_KI_POSITION_X;_kdPosition[X]=C_WS_KD_POSITION_X; break;}
        case(Y): {_c_wsLimits[Y] = C_WS_RANGE_Y;_kpPosition[Y]=C_WS_KP_POSITION_Y;_kiPosition[Y]=C_WS_KI_POSITION_Y;_kdPosition[Y]=C_WS_KD_POSITION_Y; break;}
        case(PITCH): {_c_wsLimits[PITCH] = C_WS_RANGE_PITCH;_kpPosition[PITCH]=C_WS_KP_POSITION_PITCH;_kiPosition[PITCH]=C_WS_KI_POSITION_PITCH;_kdPosition[PITCH]=C_WS_KD_POSITION_PITCH;break;}
        case(ROLL): {_c_wsLimits[ROLL] = C_WS_RANGE_ROLL;_kpPosition[ROLL]=C_WS_KP_POSITION_ROLL;_kiPosition[ROLL]=C_WS_KI_POSITION_ROLL;_kdPosition[ROLL]=C_WS_KD_POSITION_ROLL;break;}
        case(YAW): {_c_wsLimits[YAW] = C_WS_RANGE_YAW;_kpPosition[YAW]=C_WS_KP_POSITION_YAW;_kiPosition[YAW]=C_WS_KI_POSITION_YAW;_kdPosition[YAW]=C_WS_KD_POSITION_YAW;break;}
      }
  }
}

//! 4
void Platform:: motionDampingGainsDefault(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      motionDampingGainsDefault(k);
    }
    return;
  }

  else{
    switch(axis_){
        case(X): {_kpSpeed[X]=MOTION_DAMPING_KP_SPEED_X;_kiSpeed[X]=MOTION_DAMPING_KI_SPEED_X;_kdSpeed[X]=MOTION_DAMPING_KD_SPEED_X; break;}
        case(Y): {_kpSpeed[Y]=MOTION_DAMPING_KP_SPEED_Y;_kiSpeed[Y]=MOTION_DAMPING_KI_SPEED_Y;_kdSpeed[Y]=MOTION_DAMPING_KD_SPEED_Y; break;}
        case(PITCH): {_kpSpeed[PITCH]=MOTION_DAMPING_KP_SPEED_PITCH;_kiSpeed[PITCH]=MOTION_DAMPING_KI_SPEED_PITCH;_kdSpeed[PITCH]=MOTION_DAMPING_KD_SPEED_PITCH;break;}
        case(ROLL): {_kpSpeed[ROLL]=MOTION_DAMPING_KP_SPEED_ROLL;_kiSpeed[ROLL]=MOTION_DAMPING_KI_SPEED_ROLL;_kdSpeed[ROLL]=MOTION_DAMPING_KD_SPEED_ROLL;break;}
        case(YAW): {_kpSpeed[YAW]=MOTION_DAMPING_KP_SPEED_YAW;_kiSpeed[YAW]=MOTION_DAMPING_KI_SPEED_YAW;_kdSpeed[YAW]=MOTION_DAMPING_KD_SPEED_YAW;break;}
      }
  }
}
