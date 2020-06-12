#ifndef DEFINITIONS_HARDWARE_H
#define DEFINITIONS_HARDWARE_H

#include "definitions_main.h"

#define WITH_FORCE_SENSOR 1
#define WITHOUT_FORCE_SENSOR 0
#define PRESENCE_FORCE_SENSOR WITHOUT_FORCE_SENSOR

//! Transmissions
#define BELT_PULLEY_R 0.00915F //! Torque/Force

const float X_TRANSMISSION = (1.0f / BELT_PULLEY_R);                //!
const float Y_TRANSMISSION = 1.0f / BELT_PULLEY_R;                  //!

const float PITCH_REDUCTION_R = 12.0F; //! Pulley Big [mm] / Pulley Belt [mm]
const float ROLL_YAW_REDUCTION_R =
    12.96f; //! Pulley Big [mm] / Pulley Belt [mm]

const float TRANSMISSIONS[] = {Y_TRANSMISSION, X_TRANSMISSION, PITCH_REDUCTION_R,ROLL_YAW_REDUCTION_R ,ROLL_YAW_REDUCTION_R };

//!Resolution
const float Y_RESOLUTION =
    (2 * M_PI / (4 * 1024)) / Y_TRANSMISSION;                       //! 14.03 um
const float X_RESOLUTION = (2 * M_PI / (4 * 500)) / X_TRANSMISSION; //! 28.5 um
const float PITCH_RESOLUTION = ((2 * M_PI / (4 * 4096)) / PITCH_REDUCTION_R) *
                               RAD_TO_DEG; //! 32 urad -> 1.8 e-3 deg
const float ROLL_YAW_RESOLUTION =
    2 * ((2 * M_PI / (4 * 4096)) / ROLL_YAW_REDUCTION_R) * RAD_TO_DEG; //!

//! Motors Datasheet
#define MAX_RPM_P 2110       //! [RPM] No load speed of the motor
#define TORQUE_CONSTANT_Y 42.3F       //! Torque constant Faulhaber 3890H024C R2016 [mNm/A]
#define MAX_CURRENT_Y 5.0F           //! Max current Faulhaber 3890H024C R2016 [A]
#define TORQUE_CONSTANT_X 30.2F       //! Torque constant RE40mm 148867 [mNm/A]
#define MAX_CURRENT_X 6.0F            //! Max current RE40mm 148867 [A]
#define TORQUE_CONSTANT_PITCH_ROLL_YAW 231.0F //! Torque constant EC 90 V1 607931 [mNm/A]
#define MAX_CURRENT_PITCH_ROLL_YAW 4.96F //! Max current EC 90 V1 607931 [A]

const float MAX_CURRENT[] = {MAX_CURRENT_Y,MAX_CURRENT_X ,MAX_CURRENT_PITCH_ROLL_YAW ,MAX_CURRENT_PITCH_ROLL_YAW ,MAX_CURRENT_PITCH_ROLL_YAW};
const float TORQUE_K[]={TORQUE_CONSTANT_Y / 1000,TORQUE_CONSTANT_X / 1000 ,TORQUE_CONSTANT_PITCH_ROLL_YAW / 1000 , TORQUE_CONSTANT_PITCH_ROLL_YAW / 1000 ,TORQUE_CONSTANT_PITCH_ROLL_YAW / 1000 };



//! Calibration
#define NB_SWITCHES 3 //! Y, X, PITCH

//! Platform Specific Parameters

#if (PLATFORM_ID == LEFT_PLATFORM)

const float ADC_EFFORT_SCALE[NB_AXIS] = {1.0f, 1.0f, 1.0f, 2.12571428571f, 2.26086956522f};

#define X_RANGE 0.195F                 //! [m]
#define Y_RANGE 0.180F                 //! [m] -> TO UPDATE FOR SECOND VERSION
#define PITCH_RANGE 40.0F * DEG_TO_RAD //! [deg]
#define ROLL_RANGE 40.0F * DEG_TO_RAD  //! [deg]
#define YAW_RANGE 50.0F * DEG_TO_RAD   //! [deg]

const float HOMING_OFFSET_X = -X_RANGE / 2; 
const float HOMING_OFFSET_Y = Y_RANGE / 2;
const float HOMING_OFFSET_PITCH = (17.0f + 7.4139f) * DEG_TO_RAD; //! [deg]

#define ENCODERSIGN_Y 1 //! LEFT
#define ENCODERSIGN_X 1 //! LEFT
#define ENCODERSIGN_PITCH 1 //! LEFT
#define ENCODERSIGN_ROLL 1 //! LEFT
#define ENCODERSIGN_YAW 1 //! LEFT

#define ADC_SIGN_Y -1     //! LEFT
#define ADC_SIGN_X -1     //! LEFT
#define ADC_SIGN_PITCH -1 //! LEFT
#define ADC_SIGN_ROLL -1  //! LEFT
#define ADC_SIGN_YAW -1   //! LEFT

const int ADC_SIGN[NB_AXIS] = {ADC_SIGN_Y,ADC_SIGN_X, ADC_SIGN_PITCH, ADC_SIGN_ROLL,ADC_SIGN_YAW};
const float ADC_USER_BIAS[NB_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f, -0.30f};

// MOTORSIGN: To programatically change the rotation sign

#define MOTORSIGN_Y -1 //! LEFT
#define MOTORSIGN_X -1 //! LEFT
#define MOTORSIGN_PITCH -1 //! LEFT
#define MOTORSIGN_ROLL -1 //! LEFT
#define MOTORSIGN_YAW -1 //! LEFT

const float ENCODERSCALE_Y = (Y_RANGE / 12400.0f * (0.180f/0.1767f));
const float ENCODERSCALE_X = (X_RANGE / 7310.0f * (0.195f / 0.180f));
const float ENCODERSCALE_PITCH  = (360.F / PITCH_REDUCTION_R / (4 * 4095.0F))*DEG_TO_RAD;
const float ENCODERSCALE_ROLL = (90.0f / 53000.0f)* DEG_TO_RAD;
const float ENCODERSCALE_YAW = (90.0f / 53000.0f) * DEG_TO_RAD;

#else
const float ADC_EFFORT_SCALE[NB_AXIS] = {1.04135423273f,2.03291821668, 1.0f, 1.85313799022, 2.4117866005f};

#define X_RANGE 0.195F                 //! [m]
#define Y_RANGE 0.180F                 //! [m] -> TO UPDATE FOR SECOND VERSION
#define PITCH_RANGE 40.0F * DEG_TO_RAD //! [deg]
#define ROLL_RANGE 40.0F * DEG_TO_RAD  //! [deg]
#define YAW_RANGE 50.0F * DEG_TO_RAD   //! [deg]


const float HOMING_OFFSET_Y  = Y_RANGE / 2; 
const float HOMING_OFFSET_X = X_RANGE / 2; 
const float HOMING_OFFSET_PITCH =  (-27.5)*DEG_TO_RAD; //! [deg]

#define ENCODERSIGN_Y -1 //! RIGHT
#define ENCODERSIGN_X -1 //! RIGHT
#define ENCODERSIGN_PITCH -1 //! RIGHT
#define ENCODERSIGN_ROLL 1 //! RIGHT
#define ENCODERSIGN_YAW 1 //! RIGHT

#define ADC_SIGN_Y 1     //! RIGHT
#define ADC_SIGN_X 1     //! RIGHT
#define ADC_SIGN_PITCH 1 //! RIGHT
#define ADC_SIGN_ROLL -1  //! RIGHT
#define ADC_SIGN_YAW -1   //! RIGHT

const int ADC_SIGN[NB_AXIS] = {ADC_SIGN_Y,ADC_SIGN_X, ADC_SIGN_PITCH, ADC_SIGN_ROLL, ADC_SIGN_YAW};
const float ADC_USER_BIAS[NB_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f, -1.5f};
// MOTORSIGN: To programatically change the rotation sign

#define MOTORSIGN_Y 1 //! RIGHT
#define MOTORSIGN_X 1 //! RIGHT
#define MOTORSIGN_PITCH 1 //! RIGHT
#define MOTORSIGN_ROLL -1 //! RIGHT
#define MOTORSIGN_YAW -1 //! RIGHT

const float ENCODERSCALE_Y = (Y_RANGE / 12400.0f* (0.180f/0.1767f));
const float ENCODERSCALE_X = (X_RANGE / 7310.0f * (0.195f/0.180f));
const float ENCODERSCALE_PITCH = (360.F / PITCH_REDUCTION_R / (4 * 4095.0F)) * DEG_TO_RAD;
const float ENCODERSCALE_ROLL = (90.0f / 53000.0f) * DEG_TO_RAD;
const float ENCODERSCALE_YAW = (90.0f / 53000.0f) * DEG_TO_RAD;

#endif

const float ENCODER_SCALE[] = {ENCODERSCALE_Y,ENCODERSCALE_X ,ENCODERSCALE_PITCH ,ENCODERSCALE_ROLL ,ENCODERSCALE_YAW };

const float ENCODER_SIGN[] = {ENCODERSIGN_Y,ENCODERSIGN_X ,ENCODERSIGN_PITCH ,ENCODERSIGN_ROLL ,ENCODERSIGN_YAW };

const float MOTOR_SIGN[] = {MOTORSIGN_Y,MOTORSIGN_X ,MOTORSIGN_PITCH ,MOTORSIGN_ROLL ,MOTORSIGN_YAW };

#endif // DEFINITIONS_HARDWARE_H