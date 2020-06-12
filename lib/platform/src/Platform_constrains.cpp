#include "Platform.h"
#include "definitions.h"

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
    float wall = C_WS_LIMITS[axis_];
    
    if (_ros_flagDefaultControl) {
      wsConstrainsDefault(axis_);
    }
    else
    {
       wall=fabs(_ros_position[axis_]); //! A symmetric wall will be built on the set position
       _kiPosition[axis_]=0.0f;
    }

    _positionD[axis_] = _position(axis_) >= wall ? wall : (_position(axis_) <= -wall ? -wall: 0.0f);
    
    if (( _position(axis_) >= wall || _position(axis_) <= -wall )
      )
      {
          _flagInWsConstrains=true;
           //_pidPosition[axis_]->reset();
           positionAxisControl(CONSTRAINS, axis_);
           
      }
      else    
      {
        _flagInWsConstrains=false;
        _effortD_ADD(axis_,CONSTRAINS)=0.0f;
      }
  }
}

//! 3
void Platform::wsConstrainsDefault(int axis_)
{
  if (axis_==-1){
    for (uint k=0; k<NB_AXIS; k++ )
    {
      wsConstrainsDefault(k);
    }
  }
  else{
    switch(axis_){
        case(Y): {_kpPosition[Y]=C_WS_KP_POSITION_Y;_kiPosition[Y]=C_WS_KI_POSITION_Y;_kdPosition[Y]=C_WS_KD_POSITION_Y; break;}
        case(X): {_kpPosition[X]=C_WS_KP_POSITION_X;_kiPosition[X]=C_WS_KI_POSITION_X;_kdPosition[X]=C_WS_KD_POSITION_X; break;}
        case(PITCH): {_kpPosition[PITCH]=C_WS_KP_POSITION_PITCH;_kiPosition[PITCH]=C_WS_KI_POSITION_PITCH;_kdPosition[PITCH]=C_WS_KD_POSITION_PITCH;break;}
        case(ROLL): {_kpPosition[ROLL]=C_WS_KP_POSITION_ROLL;_kiPosition[ROLL]=C_WS_KI_POSITION_ROLL;_kdPosition[ROLL]=C_WS_KD_POSITION_ROLL;break;}
        case(YAW): {_kpPosition[YAW]=C_WS_KP_POSITION_YAW;_kiPosition[YAW]=C_WS_KI_POSITION_YAW;_kdPosition[YAW]=C_WS_KD_POSITION_YAW;break;}
      }
  }
}