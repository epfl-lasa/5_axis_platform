#ifndef PLATFORM_H
#define PLATFORM_H

#include <ros.h>
#include <mbed.h>
#include <SPI.h>
#include <QEC_1X_SPI.h>
#include <LP_Filter.h>
#include <FootInputMsg.h>
#include <FootOutputMsg.h>
#include <Platform.h>
#include <definitions.h>
#include <PID_v1.h>

#define NB_AXIS 5
#define NB_SWITCHES 3
#define WRENCH_COMPONENTS 3

#define SPEED_CONTROLLED_HOMING 1
#define TORQUE_CONTROLLED_HOMING 2
#define HOMING_TECHNIQUE SPEED_CONTROLLED_HOMING

class Platform
{
  public:    
    // ROS variables
    ros::NodeHandle _nh;
    DigitalIn* _esconEnabled;
  private:
    // Enum for axis ID
    enum Axis {X,Y,PITCH,ROLL,YAW};

    enum WrenchComp {NORMAL,CONSTRAINS,COMPENSATION,FEEDFORWARD};

    // Enum for state machine
    enum State {HOMING,CENTERING,TELEOPERATION}; 

    // Enum for the controller that is directly ouput for the motors
    enum Controller {TORQUE_ONLY, POSE_ONLY, TWIST_ONLY, TWIST_POSE_CASCADE, POSE_TWIST_CASCADE}; //! F= D(K(x-xd)-x_dot)

    // ROS variables  

    ros::Subscriber<custom_msgs::FootInputMsg>*  _subFootInput;
    ros::Publisher *_pubFootOutput;
    custom_msgs::FootOutputMsg _msgFootOutput;

    // State variables
    volatile State _state;
    volatile bool _stateOnceFlag[3];
    Controller _controllerType;
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
    volatile double _wrenchD_ADD[WRENCH_COMPONENTS][NB_AXIS];
    LP_Filter* _poseFilters[NB_AXIS];
    LP_Filter* _twistFilters[NB_AXIS];
    volatile int _switchesState[NB_AXIS];
    int _encoderSign[NB_AXIS];
    int _motorSign[NB_AXIS];
    float _encoderScale[NB_AXIS];
    float _c_wsLimits[NB_AXIS];
    float _wsRange[NB_AXIS];
    

    // Hardware variables
    PinName _csPins[NB_AXIS];
    QEC_1X* _encoders[NB_AXIS];
    PinName _motorPins[NB_AXIS];
    PwmOut* _motors[NB_AXIS];
    PinName _limitSwitchesPins[NB_AXIS];
    
    InterruptIn* _limitSwitches[NB_AXIS]; 
    SPI* _spi;

    // PID variables
      //General Variables
    double _kpPose[NB_AXIS];
    double _kiPose[NB_AXIS];
    double _kdPose[NB_AXIS];
    double _kpTwist[NB_AXIS];
    double _kiTwist[NB_AXIS];
    double _kdTwist[NB_AXIS];
      //GoTo constants
    double _gtKpPose[NB_AXIS];
    double _gtKiPose[NB_AXIS];
    double _gtKdPose[NB_AXIS];
    double _gtKpTwist[NB_AXIS];
    double _gtKiTwist[NB_AXIS];
    double _gtKdTwist[NB_AXIS];
    
    

    PID* _pidPose[NB_AXIS];
    PID* _pidTwist[NB_AXIS];

    // Other variables
    uint32_t _timestamp;
    static Platform *me;
    float _epson;

    Timer _innerTimer; //! micros()
    uint32_t _toc;
    bool _tic; //flag for timer


    //Compensation variables

    bool _frictionIDFlag;


  public:

    Platform();
    ~Platform();

    void init();

    void step();

    void communicateToRos();

    void setWrenches();
    
    void getMotion();

  private:

    void getPose();

    void getTwist();

    void poseControl(WrenchComp Component);

    void posAxisControl(WrenchComp Component, int axis);
    
    void speedAxisControl(WrenchComp Component, int axis);

    void twistControl(WrenchComp Component);

    void setForce(float force,  PwmOut *pin, int sign, int axis);
    
    void setTorque(float torque, PwmOut *pin, int sign, int axis);

    static void switchCallbackX();

    static void switchCallbackY();

    static void switchCallbackPitch();

    void poseAllReset();

    static void updateFootInput(const custom_msgs::FootInputMsg &msg);

    void pubFootOutput();

    void wsConstrains();

    void frictionID(int axis_, int direction_);

    void gotoPointAxis(int axis_, float point);

    void gotoPointAll(float pointX, float pointY, float pointPITCH, float pointROLL, float pointYAW);
    
};

#endif