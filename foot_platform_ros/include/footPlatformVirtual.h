#ifndef __footPlatformVirtual_H__
#define __footPlatformVirtual_H__

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

using namespace std;

class footPlatformVirtual {

public:
  enum Platform_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2};

private:
  
  int8_t _ros_platform_id;
  
  Platform_Name _platform_id;

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_position;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_positionPrev;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_velocity;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_effort;

  double _msgTime;
  double _msgTimePrev;
  
  //KDL variables

  // ros variables

  custom_msgs::FootOutputMsg _msgVirtualOutput;
  sensor_msgs::JointState _msgPlatformJointStates;


  // ros variables

  ros::NodeHandle _n;
  ros::Rate _loopRate;


  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
 
  // Publisher declaration
  ros::Publisher _pubVirtualFootOutput; // FootOutputMsg
  ros::Publisher _pubPlatformJointStates; // sensor_msgs::JointState
  ros::Subscriber _subVirtualFootInput; // FootInputMsg
  ros::Subscriber _subPlatformJointStates; //sensor_msgs::JointState
  //! boolean variables
  bool _flagPlatformConnected;


  bool _stop;

  std::mutex _mutex;
  static footPlatformVirtual *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  footPlatformVirtual(ros::NodeHandle &n_1, double frequency,
                     footPlatformVirtual::Platform_Name platform_id_);

  ~footPlatformVirtual();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void publishVirtualFootOutput();
  void publishPlatformJointStates();
  void readPlatformJointStates(const sensor_msgs::JointState::ConstPtr &msg);
  void readVirtualFootInput(const custom_msgs::FootInputMsg::ConstPtr &msg);

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __footPlatformVirtual_H__