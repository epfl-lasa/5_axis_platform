#ifndef DEFINITIONS_CONTROL_H
#define DEFINITIONS_CONTROL_H

#include "definitions_main.h"

// Type of control
enum Controller {
  TORQUE_CTRL,
  POSITION_CTRL,
  SPEED_CTRL,
  POS_SPEED_CTRL,
  SPEED_POS_CTRL,
  FS_CTRL,
  SOFT_LIMITS_CTRL
};

//! Sampling times and loops
#define COMM_LOOP 4000 //! [us] -> 250Hz..
#define CTRL_LOOP 1000 //! [us] -> 500us = 2KHz  /50
const uint32_t VELOCITY_PID_SAMPLE_P = 2 * CTRL_LOOP; //!  [us]
const uint32_t ACC_SAMPLE_P = 4 * CTRL_LOOP;
const uint32_t ANALOG_SAMPLING_TIME = 4 * CTRL_LOOP;
const uint32_t POSITION_PID_SAMPLE_P = CTRL_LOOP; //! [us]
const float invSpeedSampT =(1.0f / ((float)VELOCITY_PID_SAMPLE_P * 1e-6f));
const float invAccSampT = (1.0f / ((float)ACC_SAMPLE_P * 1e-6f));

//! Macros for ADC

#define ADC 1
#define EFFORT_M ADC

//! Input and output resolution
#define MY_PWM_RESOLUTION 16  // Bits
#define MY_PWM_FREQUENCY 5000 // Hz
#define MY_ADC_RESOLUTION 12  // Bits


//! Scaling of PID Gains

const float SCALE_GAINS_LINEAR_POSITION  = 1.0f;
const float SCALE_GAINS_LINEAR_SPEED = 1e-2f;
const float SCALE_GAINS_ANGULAR_POSITION = 1e-4f * RAD_TO_DEG;
const float SCALE_GAINS_ANGULAR_SPEED = 1e-5f * RAD_TO_DEG;

//! Event based haptics

#define NB_FF_COMP 1
#define FF_VIB 0

#endif // DEFINITIONS_CONTROL_H