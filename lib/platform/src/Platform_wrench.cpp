#include "Platform.h"


//! 1
void Platform::setEfforts()
{ 
  Eigen::Matrix<float, NB_AXIS,1> tempEffortD;
  tempEffortD.setConstant(0.0f);
  for(int j = 0; j < NB_EFFORT_COMPONENTS; j++)
  {
    if (_platform_effortComp[j]==1) {
      tempEffortD += _effortD_ADD.col(j);
    }
  }

  _effortD = tempEffortD.cwiseMin(_maxCtrlEfforts).cwiseMax(-_maxCtrlEfforts);

  setEffortAxis(_effortD(X),(int)X);
  setEffortAxis(_effortD(Y),(int)Y);
  setEffortAxis(_effortD(PITCH),(int)PITCH);
  // Adapt roll and yaw commands due to differential mechanism
  setEffortAxis((_effortD(YAW)+_effortD(ROLL))/2.0f, (int)ROLL);
  setEffortAxis((_effortD(YAW)-_effortD(ROLL))/2.0f,(int)YAW);

}

//! 2
void Platform::setEffortAxis(float effort, int axis)
{
  float escon_torque = effort/TRANSMISSIONS[axis];
  escon_torque = clip(escon_torque, -SAFETY_MAX_EFFORTS[axis], SAFETY_MAX_EFFORTS[axis]);
  setCurrentAxis(escon_torque, axis);
}

//! 3
void Platform::setCurrentAxis(float torque, int axis)
{
  float escon_current = torque*INV_TORQUE_K[axis];
  float escon_current_PWM = map(MOTOR_SIGN[axis]*escon_current, -MAX_CURRENT[axis], MAX_CURRENT[axis], 0.1f, 0.9f); //! from 10 to 90% of Duty Cycle to acknowledge connection
  _motors[axis]->write(escon_current_PWM);
}

