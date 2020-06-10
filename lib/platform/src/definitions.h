#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define RIGHT_PLATFORM 1
#define LEFT_PLATFORM 2
#define PLATFORM_ID RIGHT_PLATFORM //! 1:Right 2:Left

extern const char *Platform_Names[];

#define NB_AXIS 5
#define NB_SWITCHES 3
#define NB_EFFORT_COMPONENTS 4 //! Normal, Constrains, Compensation, Feedforward
#define NB_MACHINE_STATES 7
#define NB_FI_CATEGORY 3

#define AXES  \
ListofAxes(Y,"Y_Joint") \
ListofAxes(X,"X_Joint") \
ListofAxes(PITCH,"PITCH_Joint") \
ListofAxes(ROLL,"ROLL_Joint") \
ListofAxes(YAW,"YAW_Joint")

#define ListofAxes(enumeration, names) enumeration,
enum Axis : size_t { AXES };
#undef ListofAxes

enum FootInput_Category {MSG_POSITION, MSG_SPEED, MSG_TORQUE};

extern const char *Axis_names[];

#define BAUDRATE 230400  //! For the serial communication

#define GRAVITY -9.80665F

#define BELT_PULLEY_R 0.00915F     //! Torque/Force

const float RAD_TO_DEG = 180.0f/M_PI;
const float DEG_TO_RAD = M_PI/180.0f;
const float X_TRANSMISSION = (1.0f / BELT_PULLEY_R); //!
const float X_RESOLUTION = (2 * M_PI / (4 * 500) ) /  X_TRANSMISSION; //! 28.5 um
const float Y_TRANSMISSION = 1.0f / BELT_PULLEY_R; //!
const float Y_RESOLUTION = (2 * M_PI / (4 * 1024) ) / Y_TRANSMISSION; //! 14.03 um
const float PITCH_REDUCTION_R = 12.0F; //! Pulley Big [mm] / Pulley Belt [mm]
const float PITCH_RESOLUTION = ((2 * M_PI / (4 * 4096)) / PITCH_REDUCTION_R )* RAD_TO_DEG; //! 32 urad -> 1.8 e-3 deg
const float ROLL_YAW_REDUCTION_R = 12.96f;  //! Pulley Big [mm] / Pulley Belt [mm]
const float ROLL_YAW_RESOLUTION = 2 * ((2 * M_PI / (4 * 4096)) / ROLL_YAW_REDUCTION_R) * RAD_TO_DEG; //! 

//#define COMM_LOOP 100 //! [us] -> 2ms = 500Hz
#define CTRL_LOOP 500 //! [us] -> 1000us = 1KHz  /50
const float VELOCITY_PID_SAMPLE_P =  4 * CTRL_LOOP;              //!  [us]
const float ACC_SAMPLE_P = 5 * CTRL_LOOP;
const float ANALOG_SAMPLING_TIME =  8 * CTRL_LOOP; 
const float POSITION_PID_SAMPLE_P =  2 * CTRL_LOOP;  //! [us]

#define MY_PWM_RESOLUTION 16 // Bits
#define MY_PWM_FREQUENCY 5000 //Hz
#define MY_ADC_RESOLUTION 12 //Bits

#define MAX_RPM_P 2110       //! [RPM] No load speed of the motor


#define TORQUE_CONSTANT_Y 42.3F       //! Torque constant Faulhaber 3890H024C R2016 [mNm/A]
#define MAX_CURRENT_Y 5.0F           //! Max current Faulhaber 3890H024C R2016 [A]
#define TORQUE_CONSTANT_X 30.2F       //! Torque constant RE40mm 148867 [mNm/A]
#define MAX_CURRENT_X 6.0F            //! Max current RE40mm 148867 [A]
#define TORQUE_CONSTANT_PITCH_ROLL_YAW 231.0F //! Torque constant EC 90 V1 607931 [mNm/A]
#define MAX_CURRENT_PITCH_ROLL_YAW 4.96F //! Max current EC 90 V1 607931 [A]


#define X_RANGE 0.195F //! [m]
#define Y_RANGE 0.180F //! [m] -> TO UPDATE FOR SECOND VERSION
#define PITCH_RANGE 40.0F * DEG_TO_RAD //! [deg]
#define ROLL_RANGE 40.0F * DEG_TO_RAD  //! [deg]
#define YAW_RANGE 50.0F * DEG_TO_RAD   //! [deg]

// const float WS_LIMITS[NB_AXIS] = {0.95* X_RANGE / 2.0, 0.9* Y_RANGE / 2.0, PITCH_RANGE / 2.0f, 1.25*ROLL_RANGE / 2.0f,
//    YAW_RANGE / 2.0f};

const float WS_LIMITS[NB_AXIS] = {Y_RANGE / 2.0, X_RANGE / 2.0,
                                  PITCH_RANGE / 2.0f, ROLL_RANGE / 2.0f,
                                  YAW_RANGE / 2.0f};

const float C_WS_LIMIT_Y = (Y_RANGE / 2)  * 0.8;
const float C_WS_LIMIT_X = (X_RANGE/ 2) * 0.8;
const float C_WS_LIMIT_PITCH = (PITCH_RANGE / 2) * 0.8;
const float C_WS_LIMIT_ROLL = (ROLL_RANGE / 2) * 1.0;
const float C_WS_LIMIT_YAW = (YAW_RANGE / 2) * 0.8;

const float C_WS_LIMITS[NB_AXIS] = {C_WS_LIMIT_Y, C_WS_LIMIT_X,
                                    C_WS_LIMIT_PITCH, C_WS_LIMIT_ROLL,
                                    C_WS_LIMIT_YAW};

// const float ADC_EFFORT_BIAS_RIGHT[NB_AXIS] = {0.0, 0.0, 0.0, 0.0, 0.0};

//! Security Variables
const float EFFORT_LIMIT_DEFAULT_Y = (TORQUE_CONSTANT_Y/1000) * (MAX_CURRENT_Y) * Y_TRANSMISSION;
const float EFFORT_LIMIT_DEFAULT_X =
    (TORQUE_CONSTANT_X / 1000) * (MAX_CURRENT_X)*X_TRANSMISSION;
// const float EFFORT_LIMIT_DEFAULT_PITCH= (TORQUE_CONSTANT_PITCH_ROLL_YAW/1000) * (MAX_CURRENT_PITCH_ROLL_YAW) * PITCH_REDUCTION_R;
// const float EFFORT_LIMIT_DEFAULT_ROLL= (TORQUE_CONSTANT_PITCH_ROLL_YAW/1000) * (MAX_CURRENT_PITCH_ROLL_YAW) * ROLL_YAW_REDUCTION_R;
// const float EFFORT_LIMIT_DEFAULT_YAW= (TORQUE_CONSTANT_PITCH_ROLL_YAW/1000) * (MAX_CURRENT_PITCH_ROLL_YAW) * ROLL_YAW_REDUCTION_R;

const float USER_MAX_EFFORTS[NB_AXIS] = {EFFORT_LIMIT_DEFAULT_Y,EFFORT_LIMIT_DEFAULT_X, 7.0f, 7.0f, 7.0f};

#define EFFORT_LIMIT_HOMING_Y EFFORT_LIMIT_DEFAULT_Y
#define EFFORT_LIMIT_HOMING_X EFFORT_LIMIT_DEFAULT_X
#define EFFORT_LIMIT_HOMING_PITCH 3
#define EFFORT_LIMIT_HOMING_ROLL 3
#define EFFORT_LIMIT_HOMING_YAW 3

const float SPEED_LIMIT_Y = Y_RANGE/0.5;
const float SPEED_LIMIT_X = X_RANGE / 0.5; //! s^-1
const float SPEED_LIMIT_PITCH =  PITCH_RANGE/0.5;
const float SPEED_LIMIT_ROLL =  ROLL_RANGE/0.5;
const float SPEED_LIMIT_YAW = YAW_RANGE/0.5;

#if (PLATFORM_ID == LEFT_PLATFORM)

const float ADC_EFFORT_SCALE[NB_AXIS] = {1.0f, 1.0f, 1.0f, 2.12571428571f, 2.26086956522f};

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



#endif // DEFINITIONS_H