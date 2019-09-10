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
    if (_flagDefaultControl) {wsConstrainsDefault(axis_);}
    else
    {
       _c_wsLimits[axis_]=fabs(_commPoseSet[axis_]); //! A symmetric wall will be built on the set position
       _kiPose[axis_]=0.0f;
    }

    _poseD[axis_] = _pose[axis_] >= _c_wsLimits[axis_] ? _c_wsLimits[axis_] : (_pose[axis_] <= -_c_wsLimits[axis_] ? -_c_wsLimits[axis_]: 0.0f);
    
    if ( _pose[axis_] >= _c_wsLimits[axis_] || _pose[axis_] <= -_c_wsLimits[axis_] )
      {
        _flagInWsConstrains=true;
        // if ( ((_pose[axis_] <= 0.0) && (_twist[axis_] <= 0.0)) || ((_pose[axis_] >= 0.0) && (_twist[axis_] >= 0.0)) ) 
        // {
          posAxisControl(CONSTRAINS,axis_);
        // }
      }
      else    
      {
        _flagInWsConstrains=false;
        _wrenchD_ADD[CONSTRAINS][axis_]=0.0f;
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
      if (_flagDefaultControl) {motionDampingGainsDefault(axis_);}
      else
      {
        _kiTwist[axis_]=0.0f; //! overwrite the i and d constants sent from ROS (because it is teleoperation mode)
        _kdTwist[axis_]=0.0f;
      }
      _twistD[axis_]=0.0f; //! the target is zero speed
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
        case(X): {_c_wsLimits[X] = C_WS_RANGE_X;_kpPose[X]=C_WS_KP_POSE_X;_kiPose[X]=C_WS_KI_POSE_X;_kdPose[X]=C_WS_KD_POSE_X; break;}
        case(Y): {_c_wsLimits[Y] = C_WS_RANGE_Y;_kpPose[Y]=C_WS_KP_POSE_Y;_kiPose[Y]=C_WS_KI_POSE_Y;_kdPose[Y]=C_WS_KD_POSE_Y; break;}
        case(PITCH): {_c_wsLimits[PITCH] = C_WS_RANGE_PITCH;_kpPose[PITCH]=C_WS_KP_POSE_PITCH;_kiPose[PITCH]=C_WS_KI_POSE_PITCH;_kdPose[PITCH]=C_WS_KD_POSE_PITCH;break;}
        case(ROLL): {_c_wsLimits[ROLL] = C_WS_RANGE_ROLL;_kpPose[ROLL]=C_WS_KP_POSE_ROLL;_kiPose[ROLL]=C_WS_KI_POSE_ROLL;_kdPose[ROLL]=C_WS_KD_POSE_ROLL;break;}
        case(YAW): {_c_wsLimits[YAW] = C_WS_RANGE_YAW;_kpPose[YAW]=C_WS_KP_POSE_YAW;_kiPose[YAW]=C_WS_KI_POSE_YAW;_kdPose[YAW]=C_WS_KD_POSE_YAW;break;}
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
        case(X): {_kpTwist[X]=MOTION_DAMPING_KP_TWIST_X;_kiTwist[X]=MOTION_DAMPING_KI_TWIST_X;_kdTwist[X]=MOTION_DAMPING_KD_TWIST_X; break;}
        case(Y): {_kpTwist[Y]=MOTION_DAMPING_KP_TWIST_Y;_kiTwist[Y]=MOTION_DAMPING_KI_TWIST_Y;_kdTwist[Y]=MOTION_DAMPING_KD_TWIST_Y; break;}
        case(PITCH): {_kpTwist[PITCH]=MOTION_DAMPING_KP_TWIST_PITCH;_kiTwist[PITCH]=MOTION_DAMPING_KI_TWIST_PITCH;_kdTwist[PITCH]=MOTION_DAMPING_KD_TWIST_PITCH;break;}
        case(ROLL): {_kpTwist[ROLL]=MOTION_DAMPING_KP_TWIST_ROLL;_kiTwist[ROLL]=MOTION_DAMPING_KI_TWIST_ROLL;_kdTwist[ROLL]=MOTION_DAMPING_KD_TWIST_ROLL;break;}
        case(YAW): {_kpTwist[YAW]=MOTION_DAMPING_KP_TWIST_YAW;_kiTwist[YAW]=MOTION_DAMPING_KI_TWIST_YAW;_kdTwist[YAW]=MOTION_DAMPING_KD_TWIST_YAW;break;}
      }
  }
}
