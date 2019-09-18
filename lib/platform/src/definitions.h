#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define NB_AXIS 5
#define NB_SWITCHES 3
#define NB_WRENCH_COMPONENTS 4
#define NB_MACHINE_STATES 7

#define BAUDRATE 230400  //! For the serial communication

#define PI 3.14159265359F

#define RIGHT_PLATFORM 0
#define LEFT_PLATFORM 1
#define PLATFORM_ID LEFT_PLATFORM //! 1:Right 2:Left

#define PITCH_REDUCTION_R 12.0F //! Pulley Big [mm] / Pulley Belt [mm]
#define ROLL_YAW_REDUCTION_R 12.96f  //! Pulley Big [mm] / Pulley Belt [mm]

#define BELT_PULLEY_R 0.00915F //! Torque/Force

#define COMM_LOOP 2000 //! [us] -> 2ms = 500Hz
#define CTRL_LOOP 500 //! [us] -> 500us = 2KHz
#define VELOCITY_PID_SAMPLE_P 5 * CTRL_LOOP             //!  [us]
#define ANALOG_SAMPLING_TIME 10 * CTRL_LOOP
#define POSITION_PID_SAMPLE_P 20 * VELOCITY_PID_SAMPLE_P //! [us]

#define MY_PWM_RESOLUTION 16 // Bits
#define MY_PWM_FREQUENCY 5000 //Hz
#define MY_ADC_RESOLUTION 12 //Bits

#define C_CURRENT_MAX_P 5.0F //! 12.1 [A] Maximum Nominal Current for the Pitch Motion
#define MAX_RPM_P 2110       //! [RPM] No load speed of the motor


#define TORQUE_CONSTANT_X 30.2F       //! Torque constant RE40mm 148867 [mNm/A]
#define MAX_CURRENT_X 6.0F            //! Max current RE40mm 148867 [A]
#define TORQUE_CONSTANT_Y 42.3F       //! Torque constant Faulhaber 3890H024C R2016 [mNm/A]
#define MAX_CURRENT_Y 5.0F           //! Max current Faulhaber 3890H024C R2016 [A]
#define TORQUE_CONSTANT_PITCH_ROLL_YAW 231.0F //! Torque constant EC 90 V1 607931 [mNm/A]
#define MAX_CURRENT_PITCH_ROLL_YAW 4.96F //! Max current EC 90 V1 607931 [A]


#define X_RANGE 0.195F //! [m]
#define Y_RANGE 0.180F //! [m] -> TO UPDATE FOR SECOND VERSION
#define PITCH_RANGE 40.0F  //! [deg]
#define ROLL_RANGE 40.0F  //! [deg]
#define YAW_RANGE 50.0F  //! [deg]

#define C_WS_RANGE_X X_RANGE*0.7/2
#define C_WS_RANGE_Y Y_RANGE*0.7/2
#define C_WS_RANGE_PITCH PITCH_RANGE/2*0.3
#define C_WS_RANGE_ROLL ROLL_RANGE/2*0.2
#define C_WS_RANGE_YAW YAW_RANGE/2*0.5

//! Security Variables
#define EFFORT_LIMIT_DEFAULT_X 25
#define EFFORT_LIMIT_DEFAULT_Y 25
#define EFFORT_LIMIT_DEFAULT_PITCH 5
#define EFFORT_LIMIT_DEFAULT_ROLL 5
#define EFFORT_LIMIT_DEFAULT_YAW 5

#define EFFORT_LIMIT_HOMING_X 25
#define EFFORT_LIMIT_HOMING_Y 25
#define EFFORT_LIMIT_HOMING_PITCH 3
#define EFFORT_LIMIT_HOMING_ROLL 3
#define EFFORT_LIMIT_HOMING_YAW 3

#define SPEED_LIMIT_X X_RANGE/0.5 //! s^-1
#define SPEED_LIMIT_Y Y_RANGE/0.5
#define SPEED_LIMIT_PITCH PITCH_RANGE/0.5
#define SPEED_LIMIT_ROLL ROLL_RANGE/0.5
#define SPEED_LIMIT_YAW YAW_RANGE/0.5

#if (PLATFORM_ID == LEFT_PLATFORM)

#define HOMING_OFFSET_X -X_RANGE / 2 
#define HOMING_OFFSET_Y Y_RANGE / 2 
#define HOMING_OFFSET_PITCH 19.0f //! [deg]

#define ENCODERSIGN_X 1 //! LEFT
#define ENCODERSIGN_Y 1 //! LEFT
#define ENCODERSIGN_PITCH 1 //! LEFT
#define ENCODERSIGN_ROLL 1 //! LEFT
#define ENCODERSIGN_YAW 1 //! LEFT

// MOTORSIGN: To programatically change the rotation sign

#define MOTORSIGN_X -1 //! LEFT
#define MOTORSIGN_Y -1 //! LEFT
#define MOTORSIGN_PITCH -1 //! LEFT
#define MOTORSIGN_ROLL -1 //! LEFT
#define MOTORSIGN_YAW -1 //! LEFT

#define ENCODERSCALE_X (X_RANGE / 7310.0f * (0.195f/0.180f))
#define ENCODERSCALE_Y (Y_RANGE / 12400.0f * (0.180f/0.1767f))
#define ENCODERSCALE_PITCH 360.F / PITCH_REDUCTION_R / (4 * 4095.0F)
#define ENCODERSCALE_ROLL 90.0f/53000.0f
#define ENCODERSCALE_YAW 90.0f/53000.0f

#else

#define HOMING_OFFSET_X X_RANGE / 2 
#define HOMING_OFFSET_Y Y_RANGE / 2 
#define HOMING_OFFSET_PITCH -24.5 //! [deg]

#define ENCODERSIGN_X -1 //! RIGHT
#define ENCODERSIGN_Y -1 //! RIGHT
#define ENCODERSIGN_PITCH -1 //! RIGHT
#define ENCODERSIGN_ROLL 1 //! RIGHT
#define ENCODERSIGN_YAW 1 //! RIGHT

// MOTORSIGN: To programatically change the rotation sign

#define MOTORSIGN_X 1 //! RIGHT
#define MOTORSIGN_Y 1 //! RIGHT
#define MOTORSIGN_PITCH 1 //! RIGHT
#define MOTORSIGN_ROLL -1 //! RIGHT
#define MOTORSIGN_YAW -1 //! RIGHT

#define ENCODERSCALE_X (X_RANGE / 7310.0f * (0.195f/0.180f))
#define ENCODERSCALE_Y (Y_RANGE / 12400.0f* (0.180f/0.1767f))
#define ENCODERSCALE_PITCH 360.F / PITCH_REDUCTION_R / (4 * 4095.0F)
#define ENCODERSCALE_ROLL 90.0f/53000.0f
#define ENCODERSCALE_YAW 90.0f/53000.0f

#endif



#endif // DEFINITIONS_H