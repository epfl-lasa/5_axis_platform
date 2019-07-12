#include <ros.h>
#include <QEC_1X_SPI.h>
#include <FootInputMsg.h>
#include <FootOutputMsg.h>
#include <LP_Filter.h>
#include <wiring_analog.h>
#include <PinNames.h>
#include <stm32f3xx_hal_dac.h>
//#include <stm32l4xx_hal_dac.h>
#include <PID_v1.h>

//! This is the main file edited in ARDUINO IDE. Using the STM32duino HAL and other packages ( for matter of time)

//! Definition of the platform specific parameters 

#define X_LIMIT 0.195F //! [m] -> TO UPDATE FOR SECOND VERSION
#define Y_LIMIT 0.213F //! [m] -> TO UPDATE FOR SECOND VERSION
#define P_LIMIT 90.0F //! [deg]

#define RIGHT_PLATFORM 1
#define LEFT_PLATFORM 2

#define PITCH_REDUCTION_R 12.0F //! Pulley Big [mm] / Pulley Belt [mm]

#define BELT_PULLEY_R 0.00915F //! Torque/Force

#define PLATFORM_ID RIGHT_PLATFORM  //! 1:Right 2:Left

#define LOOP_P 200
#define VELOCITY_PID_SAMPLE_P 5*LOOP_P //!  [us]*/
#define POSE_PID_SAMPLE_P 10*VELOCITY_PID_SAMPLE_P //! [us]

#define CURRENT_K_XY 30.2F //! Maxon Motor RE40mm 148867 [mNm/A]
#define CURRENT_K_P 80.7F

#define C_CURRENT_MAX_P 5.0F  //! 12.1 [A] Maximum Nominal Current for the Pitch Motion 
#define MAX_RPM_P 2110 //! [RPM] No load speed of the motor


#if(PLATFORM_ID==LEFT_PLATFORM)
  #define HOMING_FORCE_X -6.0
  #define HOMING_FORCE_Y 6.0 
  #define HOMING_TORQUE_P -0.3

 /* #define HOMING_SPEED_X -0.5 //! [m/s]
  #define HOMING_SPEED_Y 0.5 //! [m/s]
  #define HOMING_ANG_SPEED_P -PI/8.0 //![rad/s] 
*/
  #define HOMING_OFFSET_X -X_LIMIT/2 //! Left
  #define HOMING_OFFSET_Y Y_LIMIT/2 //! Left
  #define HOMING_OFFSET_P -77.0 //! [deg]

  #define ENCODERSIGN1  -1 //! LEFT
  #define ENCODERSIGN2  1 //! LEFT
  #define ENCODERSIGN3  -1 //! LEFT

  #define ENCODERSCALE1 X_LIMIT/15735.0F*(350/353.937041) 
  #define ENCODERSCALE2 Y_LIMIT/16986.0F*(293/273.32206)
  #define ENCODERSCALE3 P_LIMIT/4256.0F //!4476
 
  #define C_CURRENT_MAX_XY 5 //! For X and Y. 6 for Right and 5 for Left [A] 
#else

 #define HOMING_FORCE_X 10.0F //! Right
 #define HOMING_FORCE_Y 10.0F
 #define HOMING_TORQUE_P -0.5F
  
 /*#define HOMING_SPEED_X 0.5 //! [m/s]
 #define HOMING_SPEED_Y 0.5 //! [m/s]
 #define HOMING_ANG_SPEED_P PI/8.0 //![rad/s] */

  #define HOMING_OFFSET_X X_LIMIT/2 //! Right
  #define HOMING_OFFSET_Y Y_LIMIT/2 //! Right
  #define HOMING_OFFSET_P -77.0 + 18.55 //! [deg]

  #define ENCODERSIGN1  -1 //! RIGHT
  #define ENCODERSIGN2  -1 //! RIGHT
  #define ENCODERSIGN3  -1 //! RIGHT

  #define ENCODERSCALE1 (X_LIMIT/7360.0F)*0.93129358228F
  #define ENCODERSCALE2 (Y_LIMIT/7560.0F)*(0.1465/0.147585198283)
  #define ENCODERSCALE3 360.F/PITCH_REDUCTION_R/(4*4095.0F)

  #define CURRENT_K_1 30.2F //! K_i Maxon Motor RE40mm 148867 [mNm/A]

  #define C_CURRENT_MAX_XY 7 //! A

#endif

uint32_t v_timestamp; //! Timestamp to set the frequency of the computention of speed


// Commanded forces and torques
double forceX=0.0F;
double forceY=0.0F;
double torqueP=0.0F;  

// Sensed forces and torques
double s_forceX=0.0F;
double s_forceY=0.0F;
double s_torqueP=0.0F;

// Measured state (position and orientation, and linear/angular velocities)
double positionX = 0.0; 
double positionY = 0.0; 
double positionX_offset=0.0;
double positionY_offset=0.0;
double orientationP_offset=0.0;
double orientationP = 0.0; 
double orientationY = 0.0;
double orientationR = 0.0;
double velocityX, velocityY, angVelocityP, angVelocityY, angVelocityR;

// Low pass filters
LP_Filter positionX_filter(0.05); //! (alpha) alpha in [0 - 1]  0-> No filter 1->filter_all
LP_Filter positionY_filter(0.05);
LP_Filter orientationP_filter(0.05);
LP_Filter orientationR_filter(0.05);
LP_Filter orientationY_filter(0.05);
LP_Filter velocityX_filter(0.1);
LP_Filter velocityY_filter(0.1);
LP_Filter angVelocityP_filter(0.1);
LP_Filter angVelocityR_filter(0.1);
LP_Filter angVelocityY_filter(0.1);

// Previous positions and orientations (for velocities computation)
double positionX_old; 
double positionY_old; 
double orientationP_old;
double orientationY_old;
double orientationR_old;


// Chip select for SPI communication
int CS1 = D6; //! CS1 -> Lateral 
int CS2 = D9; //! CS2  -> Dorsi/Plantar Flexion
int CS3 = D10; //! CS3 -> Flexion/Extension of the Leg
int CS4 = A1; //! CS4 -> roll and yaw encoder 1
int CS5 = A2; //! CS5 -> roll and yaw encoder 2

// Other pins for control
int motorRollYaw_pin1 = D0;
int motorRollYaw_pin2 = D1;
int limitSwitchPitch = D2;
int motorX_pin = D3; //!D3 
int motorY_pin = D4; //! D4 
int motorPitch_pin = D5; //! A3: DAC || D5
int limitSwitchY = D7;
int limitSwitchX = D8; 


// State machine
enum {
  homing, 
  centering,
  normal_op,
  compensation_op,
  feedforward_op
}states; 

int stateM=homing;
int switchesState[3] = {0,0,0};

/* PIDs INITIALIZATION*/ 

double referencePosX, referencePosY, referenceOriP;
double referenceVelX, referenceVelY, referenceAngVelP;

double kp_PosX=0, ki_PosX=0, kd_PosX=0;
double kp_PosY=0, ki_PosY=0, kd_PosY=0;
double kp_OriP=0*PI/180.0F*0.001, ki_OriP=0*PI/180.0F*0.001, kd_OriP=0*PI/180.0F*0.001; //! Nm/rad


double kp_VelX=0, ki_VelX=0, kd_VelX=0;
double kp_VelY=0, ki_VelY=0, kd_VelY=0;
double kp_AngVelP=0*PI/180.0F*0.001, ki_AngVelP=0*PI/180.0F*0.001, kd_AngVelP=0*PI/180.0F*0.001; //! Nm/rad



PID PID_posX(&positionX, &forceX, &referencePosX, kp_PosX, ki_PosX, kd_PosX, DIRECT); //! On measurement
PID PID_posY(&positionY, &forceY, &referencePosY, kp_PosY, ki_PosY, kd_PosY, DIRECT);
PID PID_oriP(&orientationP, &torqueP, &referenceOriP, kp_OriP, ki_OriP, kd_OriP, DIRECT);

PID PID_velX(&velocityX, &forceX, &referenceVelX, kp_VelX, ki_VelX, kd_VelX, DIRECT); //! On measurement
PID PID_velY(&velocityY, &forceY, &referenceVelY, kp_VelY, ki_VelY, kd_VelY, DIRECT);
PID PID_angvelP(&angVelocityP, &torqueP, &referenceAngVelP, kp_AngVelP, ki_AngVelP, kd_AngVelP, DIRECT);

ros::NodeHandle nh;
custom_msgs::FootOutputMsg output_msg;


void FI_subInput(const custom_msgs::FootInputMsg& input_msg){ 
    s_forceX = input_msg.FxDes; 
    s_forceY = input_msg.FyDes; 
    /* s_forceZ = s_wrench_msg.force.z;*/
    s_torqueP = input_msg.TphiDes;
    /* s_torqueR = s_wrench_msg.torque.y;
    s_torqueY = s_wrench_msg.torque.z */
    stateM=input_msg.stateDes;
}

#if(PLATFORM_ID==LEFT_PLATFORM)
  ros::Subscriber<custom_msgs::FootInputMsg> s_input("/FI_Input/Left",FI_subInput);
  ros::Publisher p_output("/FI_Output/Left", &output_msg);
#else
  ros::Subscriber<custom_msgs::FootInputMsg> s_input("/FI_Input/Right",FI_subInput);
  ros::Publisher p_output("/FI_Output/Right", &output_msg);
#endif

typedef enum 
{
  axisX=1, axisY=2, axisP=3
}axisID;


QEC_1X encoderX(CS1);
QEC_1X encoderY(CS3);
QEC_1X encoderP(CS2);

/************************ Set of functions for Force Reflection *****************************

*******************************************************************************************/ 

void FI_setTorque(float torque, int pin, int sign, int axis ){

  float k_i=0.0F, i_max = 0.0F,reduction =0.0;
  
  switch(axis) {   /*Loading of parameters of the torque control depending on the axis of interest*/
    case axisP:
      k_i=CURRENT_K_P;
      i_max=C_CURRENT_MAX_P;
      reduction=PITCH_REDUCTION_R;
      break;
    case axisX:
      k_i=CURRENT_K_XY;
      i_max=C_CURRENT_MAX_XY;
      reduction=1.0F;
      break;
    case axisY:
      k_i=CURRENT_K_XY;
      i_max=C_CURRENT_MAX_XY;
      reduction=1.0F;
      break;
  }


  double escon_current = (torque*1000/k_i)/reduction;
  escon_current = (escon_current > i_max ? i_max : (escon_current<-i_max ? -i_max : escon_current)); // Saturate the Current to the max of the escon
  int escon_current_PWM = map(escon_current,-i_max,i_max,410,3685); //! The range set in ESCON studio is from 10% to 90% PWM
  escon_current_PWM *=sign;
  analogWrite(pin,escon_current_PWM);
}

void FI_setForce(float force, int pin, int sign, int axis ){

  switch(axis)
  {   /*Loading of parameters of the torque control depending on the axis of interest*/
    case axisX:
    {
      force/=cos(PI/3);
      break;
    }
    case axisY:
    {
      force/=cos(PI/3);
      break;
    }
    default:
    {
      break;
    }
  }
  float escon_torque = force*BELT_PULLEY_R; //! Convert from torque to force
  FI_setTorque(escon_torque,pin,sign,axis);
}

void FI_setWrenchs(){

    FI_setForce(forceX,motorX_pin,1,axisX);
    FI_setForce(forceY,motorY_pin,1,axisY);
    FI_setTorque(torqueP,motorPitch_pin,1,axisP); 
}



void FI_mPoseControl(){
  PID_posX.SetTunings(kp_PosX,ki_PosX,kd_PosX);
  PID_posY.SetTunings(kp_PosY,ki_PosY,kd_PosY);
  PID_oriP.SetTunings(kp_OriP,ki_OriP,kd_OriP);
  PID_posX.Compute();
  PID_posY.Compute();
  PID_oriP.Compute();
}

void FI_mTwistControl(){
  PID_velX.SetTunings(kp_VelX,ki_VelX,kd_VelX);
  PID_velY.SetTunings(kp_VelY,ki_VelY,kd_VelY);
  PID_angvelP.SetTunings(kp_AngVelP,ki_AngVelP,kd_AngVelP);
  PID_velX.Compute();
  PID_velY.Compute();
  PID_angvelP.Compute();
}

void FI_sPoseControl(PID PID_axis, double kp, double ki, double kd){

  PID_axis.SetTunings(kp,ki,kd);
  PID_axis.Compute();
}


/***********************************************************************************/

void FI_getPose(){ //! Get the pose of the platform using the values of the encoders
   encoderX.QEC_getPose(); //! Motor for motion in X
   encoderY.QEC_getPose(); //! Motor for motion in Y
   encoderP.QEC_getPose(); //! Motor for motion in Pitch
   positionX=encoderX.outDimension + positionX_offset; 
   positionX=positionX_filter.Update(positionX);
   positionY=encoderY.outDimension + positionY_offset;
   positionY=positionY_filter.Update(positionY);
   orientationP=encoderP.outDimension + orientationP_offset; //! Rad/s
   orientationP=orientationP_filter.Update(orientationP);
   orientationR= 0.0F;
   orientationR=orientationR_filter.Update(orientationR);
   orientationY= 0.0F;
   orientationY=orientationY_filter.Update(orientationY);
}


void FI_getTwist(){
    
    if (fabs(micros()-v_timestamp)>= VELOCITY_PID_SAMPLE_P) /*Only compute speed if the elapsed time is equal to the velocity sampling period*/ 
    {
      velocityX= (positionX-positionX_old)/(VELOCITY_PID_SAMPLE_P*1e-6);
      velocityX=velocityX_filter.Update(velocityX);
      velocityY= (positionY-positionY_old)/(VELOCITY_PID_SAMPLE_P*1e-6);
      velocityY=velocityY_filter.Update(velocityY);
      angVelocityP = (orientationP-orientationP_old)/(VELOCITY_PID_SAMPLE_P*1e-6);
      angVelocityP = angVelocityP_filter.Update(angVelocityP);
      angVelocityR = (orientationR-orientationR_old)/(VELOCITY_PID_SAMPLE_P*1e-6);
      angVelocityR = angVelocityR_filter.Update(angVelocityR);
      angVelocityY = (orientationY-orientationY_old)/(VELOCITY_PID_SAMPLE_P*1e-6);
      angVelocityY = angVelocityY_filter.Update(angVelocityY);
      positionX_old=positionX; 
      positionY_old=positionY; 
      orientationP_old=orientationP;
      orientationR_old=orientationR;
      orientationY_old=orientationY;
      v_timestamp=micros();
    }
} 


void FI_getMotion(){
    FI_getPose();
    FI_getTwist();
}

void FI_encBias(QEC_1X &encoder){ //! Set an offset to the counter for setting a new zero //! Alternatively clear counters? 0X20 SPI
   encoder.QEC_offset(); 
}


void FI_posResetX_cb(){  //! FOR THE MOMENT THERE ARE ONLY 2 LIMIT SWITCHES ( FOR X AND Y )
  if (switchesState[0]==0){
  //  FI_encBias(encoderX);
  //  positionX_offset=HOMING_OFFSET_X;
    switchesState[0]=1;
  }
}

void FI_posResetY_cb(){ 
   if (switchesState[1]==0){
   // FI_encBias(encoderY);
   // positionY_offset=HOMING_OFFSET_Y;
    switchesState[1]=1;
  }
}

void FI_oriResetP_cb(){ 
   if (switchesState[2]==0){
    //FI_encBias(encoderP);
    //orientationP_offset=HOMING_OFFSET_P;
    switchesState[2]=1;
  }
}

void FI_allReset(){
  if (switchesState[0]==1 && switchesState[1]==1 && switchesState[2]==1){
    FI_encBias(encoderX);
    FI_encBias(encoderY);
    FI_encBias(encoderP);
    positionX_offset=HOMING_OFFSET_X;
    positionY_offset=HOMING_OFFSET_Y;
    orientationP_offset=HOMING_OFFSET_P;
  }
}
/************************ Set of functions for Communication *****************************
* FI_pubPose() will broadcast a topic /FI_pose with the pose of the platform
*******************************************************************************************/ 

void FI_pubOutput(){
  output_msg.id=PLATFORM_ID;
  output_msg.stamp=nh.now();
  output_msg.x = positionX;
  output_msg.y = positionY;
  /*output_msg.z = 0.0f;*/
  output_msg.phi = orientationP;
  output_msg.theta = orientationR;
  output_msg.psi = orientationY;
 /* pose_msg.pose.orientation.w = 0.0f;*/
  output_msg.Fx = forceX;
  output_msg.Fy = forceY;
  /*output_msg.z = 0.0f;*/
  output_msg.Tphi = torqueP;
  output_msg.Ttheta = 0.0;
  output_msg.Tpsi = 0.0;
  output_msg.vx= velocityX;
  output_msg.vy= velocityY;
  output_msg.wphi= angVelocityP;
  output_msg.wtheta= angVelocityR;
  output_msg.wpsi= angVelocityY;
  output_msg.state = stateM;
 /* pose_msg.pose.orientation.w = 0.0f;*/
  p_output.publish(&output_msg);
}


/************************ Set of functions for Debugging *****************************
* FI_print() will allow you to test the serial communication, by testing the broadcast of the pose of the platform.
*******************************************************************************************/ 

void FI_print() { //! For serial debugging
  Serial.print("PositionX = ");
  Serial.print(encoderX.outDimension,4);
  Serial.print(", ");
  
  Serial.print("PositionY = ");
  Serial.print(encoderY.outDimension,4);
  Serial.print(", ");

  Serial.print("orientationP = ");
  Serial.print(encoderP.outDimension,4);
  Serial.print(", ");
  Serial.println();
  //!delay(100);
}


void setup() {

  
  pinMode(limitSwitchX, INPUT);
  pinMode(limitSwitchY, INPUT);
  pinMode(limitSwitchPitch, INPUT);
  attachInterrupt(limitSwitchX, FI_posResetX_cb, RISING);
  attachInterrupt(limitSwitchY, FI_posResetY_cb, RISING);
  attachInterrupt(limitSwitchPitch, FI_oriResetP_cb, RISING);
  nh.initNode();
  nh.advertise(p_output);
  /*nh.advertise(p_pose);*/
  /*nh.advertise(p_wrench);*/
  /*nh.advertise(p_twist);*/
  /*nh.subscribe(s_wrench);*/
  nh.subscribe(s_input);

      /*****ROS communication initialization*******/
  analogWriteResolution(12); //! Set resolution of the PWM's and DAC outputs for the ESCON Drivers.  
  analogReadResolution(12);
  //! Automatic initializations of PID's

  PID_posX.SetOutputLimits(-25.0,25.0); //! N
  PID_posY.SetOutputLimits(-25.0,25.0); //! N
  PID_oriP.SetOutputLimits(-12.0,12.0); //! Nm
  PID_posX.SetSampleTime(POSE_PID_SAMPLE_P); //! [us]
  PID_posY.SetSampleTime(POSE_PID_SAMPLE_P); //! [us]
  PID_oriP.SetSampleTime(POSE_PID_SAMPLE_P); //! [us]

  PID_velX.SetOutputLimits(-25.0,25.0); //! N
  PID_velY.SetOutputLimits(-25.0,25.0); //! N
  PID_angvelP.SetOutputLimits(-12.1,12.1); //! Nm
  PID_velX.SetSampleTime(VELOCITY_PID_SAMPLE_P); //! [us]
  PID_velY.SetSampleTime(VELOCITY_PID_SAMPLE_P); //! [us]
  PID_angvelP.SetSampleTime(VELOCITY_PID_SAMPLE_P); //! [us]

  PID_posX.SetMode(AUTOMATIC);
  PID_posY.SetMode(AUTOMATIC);
  PID_oriP.SetMode(AUTOMATIC);   

  PID_velX.SetMode(AUTOMATIC);
  PID_velY.SetMode(AUTOMATIC);
  PID_angvelP.SetMode(AUTOMATIC);   

  Serial.begin(230400);
  while (!Serial) {
    ; 
  }

//! Set up of the tasks

  encoderX.QEC_init( axisX , ENCODERSCALE1, ENCODERSIGN1);
  encoderY.QEC_init( axisY , ENCODERSCALE2, ENCODERSIGN2);
  encoderP.QEC_init( axisP , ENCODERSCALE3, ENCODERSIGN3);

  v_timestamp=micros(); //! Get the time of the micro
}

void loop() {


    //!First step -> Get the pose of the platform.
    FI_getMotion();
    //FI_ImpUpdate();
    
    switch(stateM) {
      case homing: //! The robot tries to go outside the workspace until it meets with the limits switches
          forceY= HOMING_FORCE_Y; //![N]
          forceX= HOMING_FORCE_X; //! 
          torqueP = HOMING_TORQUE_P;  //! [Nm]
          //!FI_oriResetP_cb(); //! resembling the limit switch
          
          /* Velocity Controller Details */ 

          /*referenceVelX=0.05F; //! [m/s]
          referenceVelY=0.05F; //! [m/s]
          referenceAngVelP=0.0F; //! [m/s]
          kp_VelX=500, kp_VelY=0, kp_AngVelP=0*PI/180.0F*0.001;
          kd_VelX=0.0, kd_VelY=0.0, kd_AngVelP=0*PI/180.0F*0.001;
          ki_VelX=0, ki_VelY=5, ki_AngVelP=0*PI/180.0F*0.001;
          FI_mTwistControl();
          */

          /*Definition of the transition rule to the next state*/ 



          if ( ( switchesState[0]==1 ) && ( switchesState[1]==1 ) && ( switchesState[2]==1 )){
              FI_allReset();
              static int idle = micros(); 
              if ((micros()-idle)>1500000){ //! After 1+1/2 second move to next state
                stateM=centering;
              }
              
            }
          break;
      case centering:
          switchesState[0] = 0; switchesState[1] = 0; switchesState[2] = 0; //! To forget that I am homed
          referencePosX=0.0, referencePosY=0.0, referenceOriP=0.0;
         /* kp_PosX=50, kp_PosY=60, kp_OriP=0*PI/180.0F*0.001;
          kd_PosX=0.3, kd_PosY=0.3, kd_OriP=0*PI/180.0F*0.001;
          ki_PosX=5, ki_PosY=10, ki_OriP=0*PI/180.0F*0.001;
          */
          kp_PosX=1000, kp_PosY=1000, kp_OriP=5000*PI/180.0F*0.001;
          kd_PosX=0.0, kd_PosY=0.0, kd_OriP=0*PI/180.0F*0.001;
          ki_PosX=0, ki_PosY=0, ki_OriP=0*PI/180.0F*0.001;
          FI_mPoseControl();
          if  ((abs(referencePosX-positionX) < 0.003) && (abs(referencePosY-positionY) < 0.003) && (abs(referenceOriP-orientationP) < 3) ){
            static int idle = micros(); 
            if ((micros()-idle)>1500000){ //! After a second and a half move to next state
                stateM=normal_op;
              };
          }
          break;
      case normal_op:
          switchesState[0] = 0; switchesState[1] = 0; switchesState[2] = 0; //! To forget that I am homed
          break;
        }

    FI_setWrenchs(); //! Apply forces and torques
    FI_pubOutput();
    nh.spinOnce();
    delayMicroseconds(LOOP_P);

 }