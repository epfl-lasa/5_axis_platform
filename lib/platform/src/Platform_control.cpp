#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

//! 1
void Platform::poseControl(WrenchComp Component)
{
  //Compute the PID

  for (int i = 0; i < NB_AXIS; i++)
  { 
    posAxisControl(Component,i);
  }
 
}

//! 2
void Platform::posAxisControl(WrenchComp Component, int axis)
{

    if ((axis<2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      _pidPose[axis]->setOutputLimits(-25.0,25.0); //!N
    }

    if ((axis>=2)&&((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE))){
      if (axis==PITCH) {_pidPose[axis]->setOutputLimits(-5.0,5.0);} //!Nm
      if (axis==ROLL || axis==YAW){_pidPose[axis]->setOutputLimits(-5.0,5.0);} //!Nm
    }

    if (_controllerType==TWIST_POSE_CASCADE){
      _poseD[axis]=_twistCtrlOut[axis];
    }

     _pidPose[axis]->setTunings(_kpPose[axis], _kiPose[axis], _kdPose[axis]);
     _pidPose[axis]->compute();

    if ((_controllerType==POSE_ONLY)||(_controllerType==TWIST_POSE_CASCADE)){
      _wrenchD_ADD[Component][axis]=_poseCtrlOut[axis];
    }
}

//! 3
void Platform::speedAxisControl(WrenchComp Component, int axis)
{
    if ((axis<2)&&((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE))){
      _pidTwist[axis]->setOutputLimits(-25,25); //!N
    }

    if ((axis>=2)&&((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE))){
      _pidTwist[axis]->setOutputLimits(-3,3); //!Nm
    }

    if (_controllerType==POSE_TWIST_CASCADE){
      _twistD[axis]=_poseCtrlOut[axis];
    }


     _pidTwist[axis]->setTunings(_kpTwist[axis], _kiTwist[axis], _kdTwist[axis]);
     _pidTwist[axis]->compute();

    if ((_controllerType==TWIST_ONLY)||(_controllerType==POSE_TWIST_CASCADE)){
      _wrenchD_ADD[NORMAL][axis]=_twistCtrlOut[axis];
    }
}

//! 4
void Platform::twistControl(WrenchComp Component)
{

  for (int i = 0; i < NB_AXIS; i++)
  {  
    speedAxisControl(Component,i);
  }
}

//! 5
void Platform::gotoPointAxis(int axis_, float point)
{
  if (_flagDefaultControl){ gotoPointGainsDefault(axis_);}
  _poseD[axis_]=point;
  posAxisControl(NORMAL,axis_);
}

//! 6
void Platform::gotoPointAll(float pointX, float pointY, float pointPITCH, float pointROLL, float pointYAW)
{
  _controllerType=POSE_ONLY;
  _poseD[X]=pointX;
  _poseD[Y]=pointY;
  _poseD[PITCH]=pointPITCH;
  _poseD[ROLL]=pointROLL;
  _poseD[YAW]=pointYAW;
  gotoPointGainsDefault(-1);
  poseControl(NORMAL);
}

//! 7
void Platform::gotoPointGainsDefault(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      gotoPointGainsDefault(k);
    }
  }

  else{
    switch(axis_){
        case(X): {_kpPose[X]=GT_KP_POSE_X;_kiPose[X]=GT_KI_POSE_X;_kdPose[X]=GT_KD_POSE_X; break;}
        case(Y): {_kpPose[Y]=GT_KP_POSE_Y;_kiPose[Y]=GT_KI_POSE_Y;_kdPose[Y]=GT_KD_POSE_Y; break;}
        case(PITCH): {_kpPose[PITCH]=GT_KP_POSE_PITCH;_kiPose[PITCH]=GT_KI_POSE_PITCH;_kdPose[PITCH]=GT_KD_POSE_PITCH;break;}
        case(ROLL): {_kpPose[ROLL]=GT_KP_POSE_ROLL;_kiPose[ROLL]=GT_KI_POSE_ROLL;_kdPose[ROLL]=GT_KD_POSE_ROLL;break;}
        case(YAW): {_kpPose[YAW]=GT_KP_POSE_YAW;_kiPose[YAW]=GT_KI_POSE_YAW;_kdPose[YAW]=GT_KD_POSE_YAW;break;}
      }
  }
}