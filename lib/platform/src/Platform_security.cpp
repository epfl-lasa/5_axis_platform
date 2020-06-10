#include "Platform.h"
#include "definitions.h"
#include "definitions_2.h"

void Platform::workspaceCheck(int axis_){
  
  if (axis_==-1)
  {
    for (uint k=0; k<NB_AXIS; k++)
    {
      workspaceCheck(k);
    }
  }

  else
  {
    if (_position(axis_) >= WS_LIMITS[axis_] || _position(axis_) <= -WS_LIMITS[axis_])
    {
      _workspaceLimitReached[axis_]=true;
    }
    else
    {
      _workspaceLimitReached[axis_]=false;
    }
  }
  

}