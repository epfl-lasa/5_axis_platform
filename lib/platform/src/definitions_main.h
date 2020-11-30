#ifndef DEFINITIONS_MAIN_H
#define DEFINITIONS_MAIN_H

#include "math.h"

//! Platform type
#define RIGHT_PLATFORM 1
#define LEFT_PLATFORM 2
#define PLATFORM_ID LEFT_PLATFORM //! 1:Right 2:Left

//! Indexing
#define NB_LIMS 2
#define L_MIN 0
#define L_MAX 1

//! Communication
//! For the serial communication look the main.cpp

//! Physics and Math Constants
#define GRAVITY -9.80665F
const float RAD_TO_DEG = 180.0f / M_PI;
const float DEG_TO_RAD = M_PI / 180.0f;

//! Features of platform
extern const char *Platform_Names[];

//! Joint Space
#define AXES  \
ListofAxes(Y,"y_joint") \
ListofAxes(X,"x_joint") \
ListofAxes(PITCH,"pitch_joint") \
ListofAxes(ROLL,"roll_joint") \
ListofAxes(YAW,"yaw_joint") \
ListofAxes(NB_AXIS,"total_joints")
//#define NB_AXIS 5 //! Y, X, PITCH, ROLL, YAW
#define ListofAxes(enumeration, names) enumeration,
enum Axis : size_t { AXES };
#undef ListofAxes
extern const char *Axis_names[];

#define PLATFORM_AXES                                                          \
  ListofPlatformAxes(p_y, "platform_y") ListofPlatformAxes(p_x, "platform_x")  \
      ListofPlatformAxes(p_pitch, "platform_pitch") ListofPlatformAxes(        \
          p_roll, "platform_roll") ListofPlatformAxes(p_yaw, "platform_yaw")   \
          ListofPlatformAxes(NB_PLATFORM_AXIS, "total_platform_joints")

#define ListofPlatformAxes(enumeration, names) enumeration,
enum Platform_Axis : size_t { PLATFORM_AXES };
#undef ListofPlatformAxes
extern const char *Platform_Axis_Names[];

//! Operational Space
enum cartesianAxis { CART_X, CART_Y, CART_Z, NB_CART_AXIS };


enum WrenchAxis { FX, FY, FZ, TX, TY, TZ, NB_AXIS_WRENCH };

//! Components of the desired effort
enum EffortComp {
  NORMAL,
  CONSTRAINS,
  COMPENSATION,
  FEEDFORWARD,
  RCM_MOTION,
  NB_EFFORT_COMPONENTS
}; //! Normal, Constrains, Compensation, Feedforward, RCM Control

//! State Machine
enum State {
  HOMING,
  CENTERING,
  TELEOPERATION,
  EMERGENCY,
  STANDBY,
  RESET_UC,
  ROBOT_STATE_CONTROL,
  NB_MACHINE_STATES
};

#endif // DEFINITIONS_MAIN_H