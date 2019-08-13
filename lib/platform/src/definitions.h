#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define BAUDRATE 230400  //! For the serial communication

#define NUCLEO32 1
#define NUCLEO64 2
#define BOARD NUCLEO64//! Nucleo 32 or Nucleo 64


#define X_LIMIT 0.325F //! [m] -> TO UPDATE FOR SECOND VERSION
#define Y_LIMIT 0.240F //! [m] -> TO UPDATE FOR SECOND VERSION
#define P_LIMIT 90.0F  //! [deg]

#define PI 3.14159265359F

#define RIGHT_PLATFORM 1
#define LEFT_PLATFORM 2
#define PLATFORM_ID LEFT_PLATFORM //! 1:Right 2:Left


#define PITCH_REDUCTION_R 12.0F //! Pulley Big [mm] / Pulley Belt [mm]
#define ROLL_YAW_REDUCTION_R 12.96f  //! Pulley Big [mm] / Pulley Belt [mm]

#define BELT_PULLEY_R 0.00915F //! Torque/Force

#define COMM_LOOP 2000 //! [us] -> 2ms = 500Hz
#define CTRL_LOOP 1000 //! [us] -> 1000us = 1KHz
#define VELOCITY_PID_SAMPLE_P 10 * CTRL_LOOP             //!  [us]
#define POSE_PID_SAMPLE_P 20 * VELOCITY_PID_SAMPLE_P //! [us]

#define MY_PWM_RESOLUTION 16 // Bits
#define MY_PWM_FREQUENCY 5000 //Hz
#define MY_ADC_RESOLUTION 12 //Bits

#define CURRENT_K_XY 30.2F //! Maxon Motor RE40mm 148867 [mNm/A]
#define CURRENT_K_P 80.7F

#define C_CURRENT_MAX_P 5.0F //! 12.1 [A] Maximum Nominal Current for the Pitch Motion
#define MAX_RPM_P 2110       //! [RPM] No load speed of the motor

#if (PLATFORM_ID == LEFT_PLATFORM)
#define HOMING_FORCE_X 10.0F
#define HOMING_FORCE_Y 12.0F
#define HOMING_TORQUE_P -0.5F

/*#define HOMING_SPEED_X 0.5 //! [m/s]
 #define HOMING_SPEED_Y 0.5 //! [m/s]
 #define HOMING_ANG_SPEED_P PI/8.0 //![rad/s] */

#define HOMING_OFFSET_X X_LIMIT / 2    
#define HOMING_OFFSET_Y Y_LIMIT / 2  

#define HOMING_OFFSET_PITCH -77.0 + 18.55 + 46.774 - 33.95 //! [deg]


#define ENCODERSIGN1 -1 //! LEFT
#define ENCODERSIGN2 1 //! LEFT
#define ENCODERSIGN3 -1 //! LEFT
#define ENCODERSIGN4 1 //! LEFT
#define ENCODERSIGN5 1 //! LEFT

#define ENCODERSCALE1 (X_LIMIT / 7310.0f)
#define ENCODERSCALE2 (Y_LIMIT / 12400.0f)
#define ENCODERSCALE3 360.F / PITCH_REDUCTION_R / (4 * 4095.0F)
#define ENCODERSCALE4 90.0f/53000.0f
#define ENCODERSCALE5 90.0f/53000.0f

#define CURRENT_K_1 30.2F             //! K_i Maxon Motor RE40mm 148867 [mNm/A]
#define TORQUE_CONSTANT_X 30.2F       //! Torque constant RE40mm 148867 [mNm/A]
#define MAX_CURRENT_X 6.0F            //! Max current RE40mm 148867 [A]
#define TORQUE_CONSTANT_Y 42.3F       //! Torque constant Faulhaber 3890H024C R2016 [mNm/A]
#define MAX_CURRENT_Y 5.0F           //! Max current Faulhaber 3890H024C R2016 [A]
#define TORQUE_CONSTANT_PITCH_ROLL_YAW 231.0F //! Torque constant EC 90 V1 607931 [mNm/A]
#define MAX_CURRENT_PITCH_ROLL_YAW 4.96F //! Max current EC 90 V1 607931 [A]

#define C_CURRENT_MAX_XY 7 //! A
#else

#define HOMING_FORCE_X 10.0F //! Right
#define HOMING_FORCE_Y 12.0F
#define HOMING_TORQUE_P -0.5F

/*#define HOMING_SPEED_X 0.5 //! [m/s]
 #define HOMING_SPEED_Y 0.5 //! [m/s]
 #define HOMING_ANG_SPEED_P PI/8.0 //![rad/s] */

#define HOMING_OFFSET_X X_LIMIT / 2   //! Right
#define HOMING_OFFSET_Y Y_LIMIT / 2   //! Right

#define HOMING_OFFSET_PITCH -77.0 + 18.55 //! [deg]


#define ENCODERSIGN1 -1 //! RIGHT
#define ENCODERSIGN2 -1 //! RIGHT
#define ENCODERSIGN3 -1 //! RIGHT
#define ENCODERSIGN4 1 //! RIGHT
#define ENCODERSIGN5 1 //! RIGHT

// #define ENCODERSCALE1 (X_LIMIT / 7360.0F) * 0.93129358228F
// #define ENCODERSCALE2 (Y_LIMIT / 7560.0F) * (0.1465 / 0.147585198283)
#define ENCODERSCALE1 (X_LIMIT / 7310.0f)
#define ENCODERSCALE2 (Y_LIMIT / 12400.0f)
#define ENCODERSCALE3 360.F / PITCH_REDUCTION_R / (4 * 4095.0F)
#define ENCODERSCALE4 90.0f/53000.0f
#define ENCODERSCALE5 90.0f/53000.0f

#define CURRENT_K_1 30.2F             //! K_i Maxon Motor RE40mm 148867 [mNm/A]
#define TORQUE_CONSTANT_X 30.2F       //! Torque constant RE40mm 148867 [mNm/A]
#define MAX_CURRENT_X 6.0F            //! Max current RE40mm 148867 [A]
#define TORQUE_CONSTANT_Y 42.3F       //! Torque constant Faulhaber 3890H024C R2016 [mNm/A]
#define MAX_CURRENT_Y 5.0F           //! Max current Faulhaber 3890H024C R2016 [A]
#define TORQUE_CONSTANT_PITCH_ROLL_YAW 231 //! Torque constant EC 90 V1 607931 [mNm/A]
#define MAX_CURRENT_PITCH_ROLL_YAW 4.96f //! Max current EC 90 V1 607931 [A]

#define C_CURRENT_MAX_XY 7 //! A

#endif

#endif