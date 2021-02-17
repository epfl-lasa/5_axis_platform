#ifndef __mixedPlatformStatePublisher_H__
#define __mixedPlatformStatePublisher_H__

#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <custom_msgs/FootInputMsg.h>
#include <custom_msgs/FootOutputMsg.h>
#include <custom_msgs/setControllerSrv.h>
#include <custom_msgs/setStateSrv.h>
#include "../../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_pid.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include "../../../5_axis_platform/lib/platform/src/definitions_security.h"
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "custom_msgs/TwoFeetOneToolMsg.h"

using namespace std;

#define AUX_PLATFORMS_ACTIONS  \
    ListofAuxPlatformActions(CLUTCH,"clutch") \
    ListofAuxPlatformActions(SWITCH,"switch") \
    ListofAuxPlatformActions(GRASP,"grasp") \
    ListofAuxPlatformActions(AUX_NOTHING,"~") \
    ListofAuxPlatformActions(NB_AUX_ACTIONS,"nb_aux_actions")
  #define ListofAuxPlatformActions(enumeration, names) enumeration,
  enum Aux_Platform_Actions : size_t { AUX_PLATFORMS_ACTIONS };
  #undef ListofAuxPlatformActions
  extern const char *Aux_Platform_Actions_Names[];
  
  #define NB_AUX_ACTIONS_CHECK NB_AUX_ACTIONS-2
  
  #define MAIN_PLATFORMS_ACTIONS  \
    ListofMainPlatformActions(MOVE_Y,"move_y") \
    ListofMainPlatformActions(MOVE_X,"move_x") \
    ListofMainPlatformActions(MOVE_Z,"move_z") \
    ListofMainPlatformActions(MOVE_YAW,"move_yaw") \
    ListofMainPlatformActions(MAIN_NOTHING,"~") \
    ListofMainPlatformActions(NB_MAIN_ACTIONS,"nb_main_actions")
  #define ListofMainPlatformActions(enumeration, names) enumeration,
  enum Main_Platform_Actions : size_t { MAIN_PLATFORMS_ACTIONS };
  #undef ListofMainPlatformActions
  extern const char *Main_Platform_Actions_Names[];

class mixedPlatformStatePublisher {

public:
  enum Action_Aux_State {AUX_ON, AUX_OFF,NB_A_AUX_STATES};
  enum Tool_Control {TOOL_POSITION_CTRL,TOOL_SPEED_CTRL};
  enum Tool_IT {RIGHT_TOOL_IT=0, LEFT_TOOL_IT=1, NB_TOOLS};
  enum Tool_ID {NO_TOOL_ID=0, RIGHT_TOOL_ID=1, LEFT_TOOL_ID=2};
  enum Tool_Type {FORCEPS=0, CAMERA=1}; 
  enum MixedPlatform_State {BOTH_TOOLS_DISABLED, 
                            TOOL_ENABLED,
                            TOOL_SWITCH,
                            };
private:
  
  
  MixedPlatform_State _mixedPlatformState;
  MixedPlatform_State _nextMixedPlatformState;

  uint8_t _mainPlatformID, _auxPlatformID;
  Platform_IT _mainPlatformIT, _auxPlatformIT;
  Tool_IT _currentToolIT, _prevEnabledToolIT;
  Tool_ID _currentToolID, _prevEnabledToolID;
  Tool_Type _toolTypes[NB_TOOLS];
  Tool_Control _toolControls[NB_TOOLS];
  Main_Platform_Actions _mainPlatformAxisToAction[NB_PLATFORM_AXIS];
  Aux_Platform_Actions _auxPlatformAxisToAction[NB_PLATFORM_AXIS];

  #define NB_TOOLS 2

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformPosition[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformVelocity[NB_PLATFORMS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformEffort[NB_PLATFORMS];
  
  double *_mixedPlatformPosition[NB_PLATFORM_AXIS];
  double *_mixedPlatformVelocity[NB_PLATFORM_AXIS];
  double *_mixedPlatformEffort[NB_PLATFORM_AXIS];


  Eigen::Matrix<double,NB_PLATFORM_AXIS,NB_LIMS> _auxPlatformWSLims;

  Eigen::Matrix<double,NB_PLATFORM_AXIS,NB_LIMS> _deadZoneValues[NB_PLATFORMS];

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformPositionOffset[NB_TOOLS];
  // ros variables
  custom_msgs::TwoFeetOneToolMsg _msgMixedPlatformState;

  bool _requestAuxAction[NB_AUX_ACTIONS_CHECK];
  bool _flagOffsetCalculated;
  bool _flagPrevToolSaved;

  bool _flagWarningCenterMainPlatform;
  // ros variables

  ros::NodeHandle _n;
  ros::Rate _loopRate;


  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
 
  // Publisher declaration
  ros::Publisher _pubMixedPlatformJointStates;
  ros::Subscriber _subPlatformJointStatesOutput[NB_PLATFORMS]; // sensor_msgs::JointState
  //! boolean variables
  bool _flagPlatformConnected[NB_PLATFORMS];


  bool _stop;
  std::mutex _mutex;
  static mixedPlatformStatePublisher *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  mixedPlatformStatePublisher(ros::NodeHandle &n_1, double frequency);

  ~mixedPlatformStatePublisher();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void checkAuxPlatformsInput();
  void mixPlatformsActions();
  void changeMainPlatformJointsImpedanceGains();
  void publishMixedPlatformJointStates();
  void readPlatformsJointState(const sensor_msgs::JointState::ConstPtr &msg, unsigned int n_);

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __mixedPlatformStatePublisher_H__