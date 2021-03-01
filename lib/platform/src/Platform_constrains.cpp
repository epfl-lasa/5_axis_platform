#include "Platform.h"

//! 1
void Platform::wsConstrains(int axis_)
{
  if (axis_==-1) //! All axis constrained
  {
    for (size_t k = 0; k<NB_AXIS; k++){
      wsConstrains(k);
    } 
  }
  else
  {
    
    if (( _position(axis_) > _softLimitsMax(axis_) ))
    {
          _flagInWsConstrains[axis_]=true;
           _softLimitsD(axis_) = _softLimitsMax(axis_);
           _pidSoftLimits[axis_]->compute();
           _effortD_ADD(axis_, SOFT_LIMITS) = _softLimitsCtrlOut(axis_);
    }
    else if (( _position(axis_) < _softLimitsMin(axis_) ))
    {
          _flagInWsConstrains[axis_]=true;
           _softLimitsD(axis_) = _softLimitsMin(axis_);
           _pidSoftLimits[axis_]->compute();
           _effortD_ADD(axis_, SOFT_LIMITS) = _softLimitsCtrlOut(axis_);
    }
    else    
    {
          _pidSoftLimits[axis_]->reset();
          _flagInWsConstrains[axis_]=false;
    }
  }
}
