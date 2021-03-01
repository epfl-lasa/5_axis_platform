#include "footStatePublisher.h"

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{
  PLATFORM_AXES};
#undef ListofPlatformAxes

char const *Platform_Names[]{"none", "right", "left"};

float const FORCE_ALPHA = 0.5f;

footStatePublisher *footStatePublisher::me = NULL;

footStatePublisher::footStatePublisher(
    ros::NodeHandle &n_1, double frequency,
    footStatePublisher::Platform_Name platform_id)
    : _n(n_1), _platform_id(platform_id), _loopRate(frequency),
      _dt(1.0f / frequency) {
  me = this;
  _stop = false;
  _ros_state_position.setZero();
  _ros_state_velocity.setZero();
  _ros_state_effort.setZero();
  _flagPlatformConnected=false;
  _ros_platform_id=1;

}

footStatePublisher::~footStatePublisher() { me->_n.shutdown(); }

bool footStatePublisher::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
  _pubFootJointStates = _n.advertise<sensor_msgs::JointState>("joint_states", 1);

  if (_platform_id == LEFT) {
    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg_v3>(
        PLATFORM_PUBLISHER_NAME_LEFT, 1,
        boost::bind(&footStatePublisher::readPlatformOutput, this, _1),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  }
  if (_platform_id == RIGHT) {

    _subPlatformOutput = _n.subscribe<custom_msgs::FootOutputMsg_v3>(
        PLATFORM_PUBLISHER_NAME_RIGHT, 1,
        boost::bind(&footStatePublisher::readPlatformOutput, this, _1),
        ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  }

  // Subscriber definitions
  signal(SIGINT, footStatePublisher::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[%s platform j.pub.]: The platform joint state publisher is about to start ",Platform_Names[_platform_id]);
    return true;
  } 
  else {
    ROS_ERROR("[%s platform j.pub.]: The ros node has a problem.",Platform_Names[_platform_id]);
    return false;
  }
}

void footStatePublisher::stopNode(int sig) { me->_stop = true; }

void footStatePublisher::run() {
  while (!_stop) {
    if (_flagPlatformConnected) {
      if ((uint8_t) _platform_id != _ros_platform_id) 
      {
        ROS_ERROR("[%s platform j.pub.]: This node is acting on the wrong platform",Platform_Names[_platform_id]);
        ros::spinOnce();
        break;
      } 
      else {
          publishFootJointStates();
      }
    }
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[%s platform j.pub.]: Platform state variables stopped",Platform_Names[_platform_id]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void footStatePublisher::publishFootJointStates() {
  //! Keep send the same valuest that the platform is broadcasting
  //_mutex.lock();

  _msgJointStates.header.stamp = ros::Time::now();

  _msgJointStates.name.resize(NB_PLATFORM_AXIS);
  _msgJointStates.position.resize(NB_PLATFORM_AXIS);
  _msgJointStates.velocity.resize(NB_PLATFORM_AXIS);
  _msgJointStates.effort.resize(NB_PLATFORM_AXIS);
  
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _msgJointStates.name[k] = std::string(Platform_Names[_platform_id])  + "_" +  std::string(Platform_Axis_Names[k]);
    _msgJointStates.position[k] = _ros_state_position[k];
    _msgJointStates.velocity[k] = _ros_state_velocity[k];
    _msgJointStates.effort[k] = _ros_state_effort[k];
  }
  _pubFootJointStates.publish(_msgJointStates);
  //_mutex.unlock();
}


void footStatePublisher::readPlatformOutput(const custom_msgs::FootOutputMsg::ConstPtr &msg) {  
  me->_ros_platform_id = msg->platform_id;

  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    me->_ros_state_position(k) = msg->platform_position[rosAxis[k]] * conversion_factor[rosAxis[k]];
    me->_ros_state_velocity(k) = msg->platform_speed[rosAxis[k]] * conversion_factor[rosAxis[k]];
    me->_ros_state_effort(k) = msg->platform_effortD[rosAxis[k]];
  }
  if (!_flagPlatformConnected) {
    me->_flagPlatformConnected = true;
  }
}

