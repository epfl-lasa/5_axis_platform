#include "Platform.h"
#include "definitions.h"


//! 1
void Platform::setEfforts()
{ 
  Eigen::Matrix<float, NB_AXIS,1> tempEffortD;
  tempEffortD.setConstant(0.0f);
  for(int j = 0; j < NB_EFFORT_COMPONENTS; j++)
  {
    if (_ros_effortComp[j]==1) {
      tempEffortD += _effortD_ADD.col(j);
    }
  }  

  _effortD=tempEffortD;

  setEffortAxis(_effortD(X),_motors[X],MOTOR_SIGN[X],(int)X);
  setEffortAxis(_effortD(Y),_motors[Y],MOTOR_SIGN[Y],(int)Y);
  setEffortAxis(_effortD(PITCH),_motors[PITCH],MOTOR_SIGN[PITCH],(int)PITCH);
  // Adapt roll and yaw commands due to differential mechanism
  setEffortAxis((_effortD(YAW)+_effortD(ROLL))/2.0f, _motors[ROLL], MOTOR_SIGN[ROLL], (int)ROLL);
  setEffortAxis((_effortD(YAW)-_effortD(ROLL))/2.0f, _motors[YAW], MOTOR_SIGN[YAW], (int)YAW);

}

//! 2
void Platform::setEffortAxis(float effort, PwmOut *pin, int sign, int axis)
{
  float escon_torque = effort/TRANSMISSIONS[axis];
  escon_torque = clip(escon_torque, -SAFETY_MAX_EFFORTS[axis], SAFETY_MAX_EFFORTS[axis]);
  setCurrentAxis(escon_torque, pin, sign, axis);
}

//! 3
void Platform::setCurrentAxis(float torque, PwmOut *pin, int sign, int axis)
{
  float escon_current = torque/TORQUE_K[axis];
  float escon_current_PWM = map(sign*escon_current, -MAX_CURRENT[axis], MAX_CURRENT[axis], 0.1f, 0.9f); //! from 10 to 90% of Duty Cycle to acknowledge connection
  pin->write(escon_current_PWM);
}

