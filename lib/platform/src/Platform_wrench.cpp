#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>


//! 1
void Platform::setEfforts()
{
  float effortSum;

  for(int k = 0; k < NB_AXIS; k++)
  { 
     effortSum = 0.0f;
      for(int j = 0; j < NB_WRENCH_COMPONENTS; j++)
      {
        if (_rosEffortComponents[j]==1) {effortSum+= _effortD_ADD[j][k];}
      }
    _effortD[k]=effortSum;
  } 
  

  setEffortAxis(_effortD[X],_motors[X],_motorSign[X],(int)X);
  setEffortAxis(_effortD[Y],_motors[Y],_motorSign[Y],(int)Y);
  setEffortAxis(_effortD[PITCH],_motors[PITCH],_motorSign[PITCH],(int)PITCH);
  // Adapt roll and yaw commands due to differential mechanism
  setEffortAxis((_effortD[YAW]+_effortD[ROLL])/2.0f, _motors[ROLL], _motorSign[ROLL], (int)ROLL);
  setEffortAxis((_effortD[YAW]-_effortD[ROLL])/2.0f, _motors[YAW], _motorSign[YAW], (int)YAW);

}

//! 2
void Platform::setEffortAxis(float effort, PwmOut *pin, int sign, int axis)
{
  float escon_torque = effort/_transmisions[axis];
  setCurrentAxis(escon_torque, pin, sign, axis);
}

//! 3
void Platform::setCurrentAxis(float torque, PwmOut *pin, int sign, int axis)
{
  float escon_current = torque/_torqueConstants[axis];
  float escon_current_PWM = map(sign*escon_current, -_maxCurrent[axis], _maxCurrent[axis], 0.1f, 0.9f); //! from 10 to 90% of Duty Cycle to acknowledge connection
  pin->write(escon_current_PWM);
}

