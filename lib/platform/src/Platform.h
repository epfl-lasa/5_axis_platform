#ifndef PLATFORM_H
#define PLATFORM_H

#include <ros.h>
#include <mbed.h>
#include <SPI.h>
#include <QEC_1X_SPI.h>
#include <LP_Filter.h>
#include <MA_Filter.h>
//#include <pid_interpolator.h>
#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>
#include <FootInputMsg_v2.h>
#include <FootOutputMsg_v2.h>
#include <setControllerSrv.h>
#include <setStateSrv.h>

#include <PID_v1.h>
#include "/home/lsrob107772/.platformio/lib/Eigen_ID3522/Dense.h"

class Platform
{
  public:    
    // ROS variables
    ros::NodeHandle _nh;
    char _logMsg[512];
    bool _stop;

    //Power Electronics Variables
    PinName _esconEnabledPins[NB_AXIS];
    InterruptIn* _esconEnabled[NB_AXIS];
    volatile unsigned int _allEsconOk;
    bool _recoveringFromError;
    bool _flagBiasADCOk;
    float _adc_sum[NB_AXIS];
    DigitalOut* _enableMotors;

    //Public Time
    uint32_t _timestamp;
    uint32_t _timestep;
    uint32_t _analogReadStamp;
    uint32_t _speedSamplingStamp;
    uint32_t _accSamplingStamp;
    Timer _innerTimer; //! micros()
    uint64_t _innerCounterADC;

  private:
    // Enum for axis ID
    
    // enum Axis {X,Y,PITCH,ROLL,YAW} -> Move to definitions 

    enum JointState {POSITION, SPEED, ACCELERATION}; 

    enum EffortComp {NORMAL,CONSTRAINS,COMPENSATION,FEEDFORWARD}; //! Count := 4

    // Enum for state machine
    enum State {HOMING,CENTERING,TELEOPERATION,EMERGENCY,STANDBY,RESET,ROBOT_STATE_CONTROL}; 

    // Enum for the controller that is directly ouput for the motors
    enum Controller {TORQUE_ONLY, POSITION_ONLY, SPEED_ONLY, SPEED_POSITION_CASCADE, POSITION_SPEED_CASCADE}; //! F= D(K(x-xd)-x_dot) SPEED_POSITION_CASCADE IS AN IMPEDANCE CTRL MODULATED BY DYNAMICAL SYSTEM
    
    // ROS variables  

      ros::Subscriber<custom_msgs::FootInputMsg_v2>*  _subFootInput;
      ros::Publisher *_pubFootOutput;
      custom_msgs::FootOutputMsg_v2 _msgFootOutput;

      ros::ServiceServer<custom_msgs::setStateSrvRequest,custom_msgs::setStateSrvResponse> *_servChangeState;
      ros::ServiceServer<custom_msgs::setControllerSrvRequest,custom_msgs::setControllerSrvResponse> *_servChangeCtrl;

      //CLIENT VARIABLES FROM (ROS)
        volatile float _ros_position[NB_AXIS];
        volatile float _ros_speed[NB_AXIS];
        volatile float _ros_effort[NB_AXIS];

        volatile bool _ros_flagDefaultControl;
        volatile int8_t _ros_ControlledAxis;
        volatile Controller _ros_controllerType;
        volatile uint8_t _ros_effortComp[NB_EFFORT_COMPONENTS];
        volatile State _ros_state;
        


    // State variables
    State _platform_state;
    Controller _platform_controllerType;

    volatile bool _flagClearLastState;
    volatile bool _flagControllerTypeChanged;
    volatile bool _flagInputReceived[NB_FI_CATEGORY];
    volatile bool _flagEmergencyCalled;
    bool _enterStateOnceFlag[NB_MACHINE_STATES];

    bool _workspaceLimitReached[NB_AXIS];

    float _position[NB_AXIS];
    float _positionOffsets[NB_AXIS];
    float _positionPrev[NB_AXIS];
    float _positionD[NB_AXIS];
    float _positionD_filtered[NB_AXIS];
    float _positionCtrlOut[NB_AXIS];
    float _speed[NB_AXIS];
    float _speedPrev[NB_AXIS];
    float _acceleration[NB_AXIS];
    float _speedD[NB_AXIS];
    float _speedCtrlOut[NB_AXIS];
    float _effortD[NB_AXIS];
    float _effortM[NB_AXIS + 2]; //The last two elements are temporary variables
    Eigen::Matrix<float, NB_AXIS, NB_EFFORT_COMPONENTS> _effortD_ADD;
    LP_Filter _positionFilters[NB_AXIS];
    LP_Filter _posDesiredFilters[NB_AXIS];
    LP_Filter _speedFilters[NB_AXIS];
    LP_Filter _accFilters[NB_AXIS];
    LP_Filter _effortMFilters[NB_AXIS];
    volatile uint _switchesState[NB_AXIS];
    float _maxEffort[NB_AXIS];
    float _maxCurrent[NB_AXIS];
    float _transmisions[NB_AXIS];
    float _torqueConstants[NB_AXIS];
    int _encoderSign[NB_AXIS];
    int _motorSign[NB_AXIS];
    float _encoderScale[NB_AXIS];
    float _c_wsLimits[NB_AXIS];
    bool _flagInWsConstrains;
    float _wsRange[NB_AXIS];
    float _effortLimits[NB_AXIS];
    // PIDInterpolator *_positionPIDIn[NB_AXIS];
    // PIDInterpolator *_speedPIDIn[NB_AXIS];

    // Hardware variables
    PinName _csPins[NB_AXIS];
    QEC_1X* _encoders[NB_AXIS];
    PinName _motorPins[NB_AXIS];
    PwmOut* _motors[NB_AXIS];
    PinName _limitSwitchesPins[NB_AXIS];
    PinName _motorCurrentsPins[NB_AXIS];
    AnalogIn* _motorCurrents[NB_AXIS];

    InterruptIn* _limitSwitches[NB_AXIS]; 
    SPI* _spi;

    // PID variables
      //General Variables
    volatile float _kpPosition[NB_AXIS];
    volatile float _kiPosition[NB_AXIS];
    volatile float _kdPosition[NB_AXIS];
    volatile float _kpSpeed[NB_AXIS];
    volatile float _kiSpeed[NB_AXIS];
    volatile float _kdSpeed[NB_AXIS];



    // PID 

    PID* _pidPosition[NB_AXIS];
    PID* _pidSpeed[NB_AXIS];

    // Other variables
    
    static Platform *me;

    uint32_t _toc;
    bool _tic; //flag for timer

//*********************************************LIST-OF-METHODS**********
  //!Platform_main.cpp
  public:
    Platform(); //! 1
    ~Platform();//! 2
    void init();//! 3
    void step();//! 4
  private:
      //! GPIO Interruptions
    static void switchCallbackX();      //! 5
    static void switchCallbackY();      //! 6
    static void switchCallbackPitch();  //! 7
    //! ROS verification
    bool flagPositionInControl();
    bool flagSpeedInControl();
    bool flagTorqueInControl();
    //! Platform_reset.cpp
  private:
      void resetEscons(); //! 1
      void softReset();   //!2
  
  //! Platform_ros.cpp
  public:
    void communicateToRos();                                              //! 1
  private:
  //ROS
    static void updateFootInput(const custom_msgs::FootInputMsg_v2 &msg); //! 2
    static void updateState(const custom_msgs::setStateSrv::Request 
    &req,custom_msgs::setStateSrv::Response &resp );                      //! 3
    static void updateController(const custom_msgs::setControllerSrv::Request 
    &req,custom_msgs::setControllerSrv::Response &resp );                 //! 4
    void pubFootOutput();                                                 //! 5

  //!Platform_effort.cpp
  public:  
    void setEfforts();                                                     //! 1
  private:
    // Effort Computation for the ESCONS
      void setEffortAxis(float effort,  PwmOut *pin, int sign, int axis);   //! 2
      void setCurrentAxis(float torque, PwmOut *pin, int sign, int axis);   //! 3

  //! Platform_utils.cpp
  public:  
    float map(float x, float in_min, float in_max, float out_min, float out_max); //! 1    
    float clip(float x, float out_min, float out_max);
    float smoothRise(float x, float a, float b);
    float smoothFall(float x, float a, float b);
    float smoothRiseFall(float x, float a, float b, float c, float d);
    Eigen::Matrix<float, Eigen::Dynamic, 1> bound(Eigen::Matrix<float, Eigen::Dynamic, 1> x, float limit);

        //!Platform_sensors.cpp
        public : void getMotion(); //! 1
  private:
      //! Robot State
      void getPosition();                     //! 2
      void getSpeed();                    //! 3
      void getAcceleration();
      //! Estimate Robot Effort (ADC)    
      void readActualEffort();            //! 4

  //! Platform_emergency.cpp
  public:
    static void emergencyCallback();  //! 1
    void releasePlatform();           //! 2

  //! Platform_security.cpp

  private:
    void workspaceCheck(int axis_);

  //! Platform_control.cpp
  private:
      // Position and Speed control
      void positionAllControl(EffortComp Component);                             //! 1
      void positionAxisControl(EffortComp Component, int axis);                //! 2
      void speedAxisControl(EffortComp Component, int axis);              //! 3
      void speedAllControl(EffortComp Component);                            //! 4
      void gotoPointAxis(int axis_, float point);                         //! 5
      void gotoPointAll(float pointX, float pointY, float pointPITCH,     
      float pointROLL, float pointYAW);                                   //! 6
      void gotoPointGainsDefault(int axis_);                              //! 7
      //! Platform_constrains.cpp
    private:    
      //Compensation
  #define NB_STICTION_COMP 2
  #define NB_SIGN_COMP 2

  #define COMP_GRAVITY 0
  #define COMP_DRY_FRICTION 1
  #define COMP_VISC_FRICTION 2
  #define COMP_INERTIA 3
  #define NB_COMPENSATION_COMP 4

  #define NB_LIMS 2
  #define L_MIN 0
  #define L_MAX 1
  #define NEG 0
  #define POS 1
  #define MID 3

  void dynamicCompensation();
  void gravityCompensation();
  void dryFrictionCompensation();
  void viscFrictionCompensation();
  void inertiaCompensation();
  
  void quadraticRegression();

  Eigen::Matrix<float, NB_STICTION_COMP, 1> _dryFrictionEffortSign[NB_SIGN_COMP];
  Eigen::Matrix<float, NB_AXIS, NB_COMPENSATION_COMP> _compensationEffort;
  Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_COMP> _predictors[NB_SIGN_COMP];
  Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_COMP> _betas[NB_SIGN_COMP];
  Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_COMP> _mean[NB_SIGN_COMP];
  Eigen::Matrix<float, 2 * NB_AXIS, NB_STICTION_COMP> _stdInv[NB_SIGN_COMP];
  Eigen::Matrix<float, 1 , NB_STICTION_COMP> _bias[NB_SIGN_COMP];

  //Constrains
  void
  wsConstrains(int axis_);                   //! -1 := all of them                 //! 1
  void motionDamping(int axis_);             //! -1:= all of them                 //! 2
  void wsConstrainsDefault(int axis_);       //! 3
  void motionDampingGainsDefault(int axis_); //! 4      

  //!Platform_clear.cpp
  private:
      //! Maintenance
      void limitSwitchesClear();
      void positionAllReset();
      void positionCtrlClear(int axis_); //! Put gains and set point to zero of the Position Control
      void speedCtrlClear(int axis_); //! Put gains and set point to zero of the Speed Control
      void totalEffortDClear(int axis_);
      void compEffortClear(int axis_, Platform::EffortComp comp_);
      void clearLastState();
      void resetControllers();
      
};

#endif //PLATFORM_H