#ifndef PLATFORM_H
#define PLATFORM_H

#include <ros.h>
#include <mbed.h>
#include <SPI.h>
#include <QEC_1X_SPI.h>
#include <LP_Filter.h>
#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>

// #include <FootInputMsg.h>
// #include <FootOutputMsg.h>
#include <FootInputMsg_v2.h>
#include <FootOutputMsg_v2.h>
#include <setControllerSrv.h>
#include <setStateSrv.h>

#include <PID_v1.h>

#define NB_AXIS 5
#define NB_SWITCHES 3
#define NB_WRENCH_COMPONENTS 4
#define NB_MACHINE_STATES 7

#define SPEED_CONTROLLED_HOMING 1
#define TORQUE_CONTROLLED_HOMING 2
#define HOMING_TECHNIQUE SPEED_CONTROLLED_HOMING

class Platform
{
  public:    
    // ROS variables
    ros::NodeHandle _nh;

    //Power Electronics Variables
    PinName _esconEnabledPins[NB_AXIS];
    InterruptIn* _esconEnabled[NB_AXIS];
    volatile int _allEsconOk;
    DigitalOut* _enableMotors;

    //Public Time
    uint32_t _timestamp;
    uint32_t _timestep;
    uint32_t _analogReadStamp;
    uint32_t _speedSamplingStamp;
    Timer _innerTimer; //! micros()
    int _innerCounter;

  private:
    // Enum for axis ID
    enum Axis {X,Y,PITCH,ROLL,YAW};

    enum RobotState {POSITION, SPEED, ACCELERATION}; 

    enum WrenchComp {NORMAL,CONSTRAINS,COMPENSATION,FEEDFORWARD}; //! Count := 4

    // Enum for state machine
    enum State {HOMING,CENTERING,TELEOPERATION,EMERGENCY,STANDBY,RESET,ROBOT_STATE_CONTROL}; 

    // Enum for the controller that is directly ouput for the motors
    enum Controller {TORQUE_ONLY, POSE_ONLY, TWIST_ONLY, TWIST_POSE_CASCADE, POSE_TWIST_CASCADE}; //! F= D(K(x-xd)-x_dot) TWIST_POSE_CASCADE IS AN IMPEDANCE CTRL MODULATED BY DYNAMICAL SYSTEM
    
    // NB. FOR THE MOMENT ONLY POSE_ONLY AND TWIST ONLY ARE USED

    // ROS variables  

      ros::Subscriber<custom_msgs::FootInputMsg_v2>*  _subFootInput;
      ros::Publisher *_pubFootOutput;
      custom_msgs::FootOutputMsg_v2 _msgFootOutput;

      ros::ServiceServer<custom_msgs::setStateSrvRequest,custom_msgs::setStateSrvResponse> *_servChangeState;
      ros::ServiceServer<custom_msgs::setControllerSrvRequest,custom_msgs::setControllerSrvResponse> *_servChangeCtrl;

      //CLIENT VARIABLES FROM (ROS)
        volatile double _commPoseSet[NB_AXIS];// TODO: Expand this to speed and acceleration
        volatile double _commTwistSet[NB_AXIS];
        volatile bool _flagDefaultControl;
        volatile int8_t _commControlledAxis;
        volatile Controller _controllerType;
        volatile uint8_t _desWrenchComponents[NB_WRENCH_COMPONENTS];
        volatile State _newState;


    // State variables
    State _state;
    State _lastState;
    
    volatile bool _flagClearLastState; 
    volatile bool _enterStateOnceFlag[NB_MACHINE_STATES];

    double _pose[NB_AXIS];
    double _poseOffsets[NB_AXIS];
    double _posePrev[NB_AXIS];
    double _poseD[NB_AXIS];
    double _poseCtrlOut[NB_AXIS];
    double _twist[NB_AXIS];
    double _twistD[NB_AXIS];
    double _twistCtrlOut[NB_AXIS];
    double _wrench[NB_AXIS];
    double _wrenchD[NB_AXIS];
    double _wrenchM[NB_AXIS+2]; //The last two elements are temporary variables
    volatile double _wrenchD_ADD[NB_WRENCH_COMPONENTS][NB_AXIS];
    LP_Filter* _poseFilters[NB_AXIS];
    LP_Filter* _twistFilters[NB_AXIS];
    LP_Filter* _wrenchMFilters[NB_AXIS];
    volatile int _switchesState[NB_AXIS];
    float _maxWrench[NB_AXIS];
    float _maxCurrent[NB_AXIS];
    float _transmisions[NB_AXIS];
    float _torqueConstants[NB_AXIS];
    int _encoderSign[NB_AXIS];
    int _motorSign[NB_AXIS];
    float _encoderScale[NB_AXIS];
    float _c_wsLimits[NB_AXIS];
    bool _flagInWsConstrains;
    double _wsRange[NB_AXIS];
    double _effortLimits[NB_AXIS];
    

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
    volatile double _kpPose[NB_AXIS];
    volatile double _kiPose[NB_AXIS];
    volatile double _kdPose[NB_AXIS];
    volatile double _kpTwist[NB_AXIS];
    volatile double _kiTwist[NB_AXIS];
    volatile double _kdTwist[NB_AXIS];



    // PID 

    PID* _pidPose[NB_AXIS];
    PID* _pidTwist[NB_AXIS];

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

  //!Platform_wrench.cpp
  public:  
    void setWrenches();                                                     //! 1
  private:
    // Effort Computation for the ESCONS
      void setWrenchAxis(float wrench,  PwmOut *pin, int sign, int axis);   //! 2
      void setCurrentAxis(float torque, PwmOut *pin, int sign, int axis);   //! 3

  //! Platform_utils.cpp
  public:  
    float map(float x, float in_min, float in_max, float out_min, float out_max); //! 1    

  //!Platform_sensors.cpp
  public:  
    void getMotion();                     //! 1
  private:
      //! Robot State
      void getPose();                     //! 2
      void getTwist();                    //! 3
      //! Estimate Robot Effort (ADC)    
      void readActualWrench();            //! 4

  //! Platform_emergency.cpp
  public:
    static void emergencyCallback();  //! 1
    void releasePlatform();           //! 2

  //! Platform_control.cpp
  private:
      // Position and Speed control
      void poseControl(WrenchComp Component);                             //! 1
      void posAxisControl(WrenchComp Component, int axis);                //! 2
      void speedAxisControl(WrenchComp Component, int axis);              //! 3
      void twistControl(WrenchComp Component);                            //! 4
      void gotoPointAxis(int axis_, float point);                         //! 5
      void gotoPointAll(float pointX, float pointY, float pointPITCH,     
      float pointROLL, float pointYAW);                                   //! 6
      void gotoPointGainsDefault(int axis_);                              //! 7  
      
  //! Platform_constrains.cpp
  private:    
      //Constrains 
      void wsConstrains(int axis_); //! -1 := all of them                 //! 1
      void motionDamping(int axis_); //! -1:= all of them                 //! 2
      void wsConstrainsDefault(int axis_);                                //! 3
      void motionDampingGainsDefault(int axis_);                          //! 4      

  //!Platform_clear.cpp
  private:
      //! Maintenance
      void limitSwitchesClear();
      void poseAllReset();
      void poseCtrlClear(int axis_); //! Put gains and set point to zero of the Pose Control
      void twistCtrlClear(int axis_); //! Put gains and set point to zero of the Twist Control
      void totalWrenchDClear(int axis_);
      void compWrenchClear(int axis_, Platform::WrenchComp comp_);
      void clearLastState(); 

  
};

#endif //PLATFORM_H