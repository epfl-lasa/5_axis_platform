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
#include <PID.h>

#define NB_AXIS 5
#define NB_SWITCHES 3
class Platform
{
  public:    
    // ROS variables
    ros::NodeHandle _nh;

  private:
    // Enum for axis ID
    enum Axis {X,Y,PITCH,ROLL,YAW};

    // Enum for state machine
    enum State {HOMING,CENTERING,NORMAL,COMPENSATION,FEEDFORWARD}; 

    // Enum for the controller that is directly ouput for the motors
    enum Controller {TORQUE_ONLY, POSE_ONLY, TWIST_ONLY, TWIST_POSE_CASCADE, POSE_TWIST_CASCADE}; //! F= D(K(x-xd)-x_dot)

    // ROS variables  

    ros::Subscriber<custom_msgs::FootInputMsg>*  _subFootInput;
    ros::Publisher *_pubFootOutput;
    custom_msgs::FootOutputMsg _msgFootOutput;

    // State variables
    State _state;
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
    LP_Filter* _poseFilters[NB_AXIS];
    LP_Filter* _twistFilters[NB_AXIS];
    volatile int _switchesState[NB_AXIS];

    // Hardware variables
    PinName _csPins[NB_AXIS];
    QEC_1X* _encoders[NB_AXIS];
    PinName _motorPins[NB_AXIS];
    PwmOut* _motors[NB_AXIS];
    PinName _limitSwitchesPins[NB_AXIS];
    InterruptIn* _limitSwitches[NB_AXIS]; 
    SPI* _spi;

    // PID variabless
    double _kpPose[NB_AXIS];
    double _tiPose[NB_AXIS];
    double _tdPose[NB_AXIS];
    double _kpTwist[NB_AXIS];
    double _tiTwist[NB_AXIS];
    double _tdTwist[NB_AXIS];
    PID* _pidPose[NB_AXIS];
    PID* _pidTwist[NB_AXIS];

    // Other variables
    uint32_t _timestamp;
    static Platform *me;
    float _epson;
    Timer _innerTimer; //! micros()

  public:

    Platform();
    ~Platform();

    void init();

    void step();

    void communicateToRos();

    void setWrenches();
    
    void getMotion();

  private:

    // void getMotion();

    void getPose();

    void getTwist();

    void poseControl();
    
    void twistControl();

    //void setWrenches();

    void setForce(float force,  PwmOut *pin, int sign, int axis);
    
    void setTorque(float torque, PwmOut *pin, int sign, int axis);

    static void switchCallbackX();

    static void switchCallbackY();

    static void switchCallbackPitch();

    void poseAllReset();

    static void updateFootInput(const custom_msgs::FootInputMsg &msg);

    void pubFootOutput();
    
};

#endif