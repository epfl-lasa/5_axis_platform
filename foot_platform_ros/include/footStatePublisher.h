#ifndef __footStatePublisher_H__
#define __footStatePublisher_H__

#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <custom_msgs/FootInputMsg_v5.h>
#include <custom_msgs/FootOutputMsg_v3.h>
#include <custom_msgs/setControllerSrv.h>
#include <custom_msgs/setStateSrv_v2.h>
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

class footStatePublisher {

public:
  enum Platform_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2};

private:
  
  int8_t _ros_platform_id;
  
  Platform_Name _platform_id;

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_position;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_velocity;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _ros_state_effort;
  
  //KDL variables

  // ros variables

  sensor_msgs::JointState _msgJointStates;

  // ros variables

  ros::NodeHandle _n;
  ros::Rate _loopRate;


  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
 
  // Publisher declaration
  ros::Publisher _pubFootJointStates;
  ros::Subscriber _subPlatformOutput; // FootOutputMsg_v3
  //! boolean variables
  bool _flagPlatformConnected;


  bool _stop;

  std::mutex _mutex;
  static footStatePublisher *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  footStatePublisher(ros::NodeHandle &n_1, double frequency,
                     footStatePublisher::Platform_Name platform_id_);

  ~footStatePublisher();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void publishFootJointStates();
  void readPlatformOutput(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg);

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __footStatePublisher_H__