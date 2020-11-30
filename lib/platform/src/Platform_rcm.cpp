#include "Platform.h"
#include "macros_rcm_control.h"

using namespace std;
using namespace Eigen;


void Platform::rcmControl()
{
  float sx_rcm = SX_RCM;
  float sy_rcm = SY_RCM;
  float sz_rcm = SZ_RCM;
  float inv_norm_rcm = INV_NORM_RCM_ANGLE;

  _cosDiffRCMCtrl = RCM_COS_DIFF_ANGLE;
  
  _pidRCM->compute();

  _rcmCtrlEffort(Y) = RCM_EFFORT_Y;
  _rcmCtrlEffort(X) = RCM_EFFORT_X;
  _rcmCtrlEffort(PITCH) = RCM_EFFORT_PITCH;
  _rcmCtrlEffort(ROLL) = RCM_EFFORT_ROLL;
  _rcmCtrlEffort(YAW) = RCM_EFFORT_YAW;

  _effortD_ADD.col(RCM_MOTION) = _rcmCtrlEffort * _rcmCtrlOut;
}

