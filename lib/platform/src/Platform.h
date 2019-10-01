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
#include <FootInputMsg_v2.h>
#include <FootOutputMsg_v2.h>
#include <setControllerSrv.h>
#include <setStateSrv.h>

#include <PID_v1.h>

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
    
    // enum Axis {X,Y,PITCH,ROLL,YAW};

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
        float _ros_position[NB_AXIS];
        float _ros_speed[NB_AXIS];
        float _ros_effort[NB_AXIS];

        bool _ros_flagDefaultControl;
        int8_t _ros_ControlledAxis;
        Controller _ros_controllerType;
        uint8_t _ros_effortComp[NB_EFFORT_COMPONENTS];
        State _state;
        


    // State variables
    State _lastState;
    
    bool _flagClearLastState; 
    bool _flagInputReceived;

    bool _enterStateOnceFlag[NB_MACHINE_STATES];

    double _position[NB_AXIS];
    double _positionOffsets[NB_AXIS];
    double _positionPrev[NB_AXIS];
    double _positionD[NB_AXIS];
    double _positionCtrlOut[NB_AXIS];
    double _speed[NB_AXIS];
    double _speedD[NB_AXIS];
    double _speedCtrlOut[NB_AXIS];
    double _effort[NB_AXIS];
    double _effortD[NB_AXIS];
    double _effortM[NB_AXIS+2]; //The last two elements are temporary variables
    double _effortD_ADD[NB_EFFORT_COMPONENTS][NB_AXIS];
    LP_Filter* _positionFilters[NB_AXIS];
    LP_Filter* _speedFilters[NB_AXIS];
    LP_Filter* _effortMFilters[NB_AXIS];
    volatile int _switchesState[NB_AXIS];
    float _maxEffort[NB_AXIS];
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
    double _kpPosition[NB_AXIS];
    double _kiPosition[NB_AXIS];
    double _kdPosition[NB_AXIS];
    double _kpSpeed[NB_AXIS];
    double _kiSpeed[NB_AXIS];
    double _kdSpeed[NB_AXIS];



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

  //!Platform_sensors.cpp
  public:  
    void getMotion();                     //! 1
  private:
      //! Robot State
      void getPosition();                     //! 2
      void getSpeed();                    //! 3
      //! Estimate Robot Effort (ADC)    
      void readActualEffort();            //! 4

  //! Platform_emergency.cpp
  public:
    static void emergencyCallback();  //! 1
    void releasePlatform();           //! 2

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
      //Constrains 
      void wsConstrains(int axis_); //! -1 := all of them                 //! 1
      void motionDamping(int axis_); //! -1:= all of them                 //! 2
      void wsConstrainsDefault(int axis_);                                //! 3
      void motionDampingGainsDefault(int axis_);                          //! 4      

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

  
};

#endif //PLATFORM_H