#include "Platform.h"

void Platform::workspaceCheck(int axis_){
  
  if (axis_==-1)
  {
    for (size_t k=0; k<NB_AXIS; k++)
    {
      workspaceCheck(k);
    }
  }

  else
  {
    if (_position(axis_) > WS_LIMITS[axis_][L_MAX] || _position(axis_) < WS_LIMITS[axis_][L_MIN])
    {
      _workspaceLimitReached[axis_]=true;
    }
    else
    {
      _workspaceLimitReached[axis_]=false;
    }
  }
  

}