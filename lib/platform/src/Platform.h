#ifndef PLATFORM_H
#define PLATFORM_H

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
#include <Platform.h>
#include <definitions.h>

#define NB_AXIS 5

class Platform
{
  private:
    // Enum for axis ID
    enum Axis {X,Y,PITCH,ROLL,YAW};

    // Enum for state machine
    enum State {HOMING,CENTERING,NORMAL,COMPENSATION,FEEDFORWARD}; 

    // ROS variables
    ros::NodeHandle _nh;

    ros::Subscriber<custom_msgs::FootInputMsg>*  _subFootInput;
    ros::Publisher *_pubFootOutput;
    custom_msgs::FootOutputMsg _msgFootOutput;

    // State variables
    State _state;
    double _pose[NB_AXIS];
    double _poseOffsets[NB_AXIS];
    double _posePrev[NB_AXIS];
    double _poseD[NB_AXIS];
    double _twist[NB_AXIS];
    double _twistD[NB_AXIS];
    double _wrench[NB_AXIS];
    double _commands[NB_AXIS];
    LP_Filter* _poseFilters[NB_AXIS];
    LP_Filter* _twistFilters[NB_AXIS];
    int _switchesState[NB_AXIS];

    // Hardware variables
    int _cs[NB_AXIS];
    QEC_1X* _encoders[NB_AXIS];
    int _motorsPins[NB_AXIS];
    int _limitSwitchesPins[NB_AXIS];

    // PID variables
    double _kpPose[NB_AXIS];
    double _kiPose[NB_AXIS];
    double _kdPose[NB_AXIS];
    double _kpTwist[NB_AXIS];
    double _kiTwist[NB_AXIS];
    double _kdTwist[NB_AXIS];
    PID* _pidPose[NB_AXIS];
    PID* _pidTwist[NB_AXIS];

    // Other variables
    uint32_t _timestamp;
    static Platform *me;
    float _epson;

  public:

    Platform();
    ~Platform();

    void init();

    void step();

  private:

    void getMotion();

    void getPose();

    void getTwist();

    void poseControl();
    
    void twistControl();

    void setWrenches();

    void setForce(float force, int pin, int sign, int axis);
    
    void setTorque(float torque, int pin, int sign, int axis);

    static void switchCallbackX();

    static void switchCallbackY();

    static void switchCallbackPitch();

    void allReset();

    static void updateFootInput(const custom_msgs::FootInputMsg &msg);

    void pubFootOutput();
};

#endif