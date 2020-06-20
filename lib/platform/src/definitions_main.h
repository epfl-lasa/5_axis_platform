#ifndef DEFINITIONS_MAIN_H
#define DEFINITIONS_MAIN_H


//! Platform type
#define RIGHT_PLATFORM 1
#define LEFT_PLATFORM 2
#define PLATFORM_ID RIGHT_PLATFORM //! 1:Right 2:Left

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
ListofAxes(Y,"Y_Joint") \
ListofAxes(X,"X_Joint") \
ListofAxes(PITCH,"PITCH_Joint") \
ListofAxes(ROLL,"ROLL_Joint") \
ListofAxes(YAW,"YAW_Joint")
#define ListofAxes(enumeration, names) enumeration,
enum Axis : size_t { AXES };
#undef ListofAxes
extern const char *Axis_names[];

//! Operational Space
enum cartesianAxis { CART_X, CART_Y, CART_Z, NB_CART_AXIS };
#define NB_AXIS 5 //! Y, X, PITCH, ROLL, YAW

//! Components of the desired effort
enum EffortComp {
  NORMAL,
  CONSTRAINS,
  COMPENSATION,
  FEEDFORWARD,
  NB_EFFORT_COMPONENTS
}; //! Normal, Constrains, Compensation, Feedforward

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