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



    // State variables
    volatile State _state;
    State _lastState;
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

  public:

    Platform();
    ~Platform();

    void init();

    void step();

    void communicateToRos();

    void setWrenches();
    
    void getMotion();
    
     //EMERGENCY

    static void emergencyCallback();

    void releasePlatform();

  private:

    void getPose();

    void getTwist();

    void poseControl(WrenchComp Component);

    void posAxisControl(WrenchComp Component, int axis);
    
    void speedAxisControl(WrenchComp Component, int axis);

    void twistControl(WrenchComp Component);

    void setWrenchAxis(float wrench,  PwmOut *pin, int sign, int axis);
    
    void setCurrentAxis(float torque, PwmOut *pin, int sign, int axis);

    static void switchCallbackX();

    static void switchCallbackY();

    static void switchCallbackPitch();

    void poseAllReset();

    static void updateFootInput(const custom_msgs::FootInputMsg_v2 &msg);
    
    static void updateState(const custom_msgs::setStateSrv::Request &req,custom_msgs::setStateSrv::Response &resp );
    static void updateController(const custom_msgs::setControllerSrv::Request &req,custom_msgs::setControllerSrv::Response &resp );
    
    void pubFootOutput();

    void wsConstrains(int axis_); //! -1 := all of them 

    void motionDamping(int axis_); //! -1:= all of them

    void gotoPointAxis(int axis_, float point);

    void gotoPointAll(float pointX, float pointY, float pointPITCH, float pointROLL, float pointYAW);

    void wsConstrainsDefault(int axis_);
    
    void gotoPointGainsDefault(int axis_);

    void motionDampingGainsDefault(int axis_); 

    void limitSwitchesClear();

    // void fetchOutputLimitsState(int axis_, Platform::RobotState robotState_);

    // void fetchOutputLimitsPose(Platform::Controller controllerType_, int axis_, double limit_);
    
    // void fetchOutputLimitsTwist(Platform::Controller controllerType_, int axis_, double limit_);

    void poseCtrlClear(int axis_); //! Put gains and set point to zero of the Pose Control

    void twistCtrlClear(int axis_); //! Put gains and set point to zero of the Twist Control

    void totalWrenchDClear(int axis_);

    void compWrenchClear(int axis_, Platform::WrenchComp comp_);
    
    void readActualWrench();

    void clearLastState();

    void resetEscons();

    void softReset();
};

#endif //PLATFORM_H