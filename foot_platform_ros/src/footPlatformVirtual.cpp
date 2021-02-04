#include "footPlatformVirtual.h"

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};
const float inv_conversion_factor[] = {1.0, 1.0, RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG};

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{
  PLATFORM_AXES};
#undef ListofPlatformAxes

char const *Platform_Names[]{"none", "right", "left"};

char const *Platform_Names_2[]{"None", "Right", "Left"};

float const FORCE_ALPHA = 0.5f;

footPlatformVirtual *footPlatformVirtual::me = NULL;

footPlatformVirtual::footPlatformVirtual(
    ros::NodeHandle &n_1, double frequency,
    footPlatformVirtual::Platform_Name platform_id)
    : _n(n_1), _platform_id(platform_id), _loopRate(frequency),
      _dt(1.0f / frequency) {
  me = this;
  _stop = false;
  _ros_state_position.setZero();
  _ros_state_positionPrev.setZero();
  _ros_state_velocity.setZero();
  _ros_state_effort.setZero();
  _msgTime=0.1;
  _msgTimePrev=0.0;

}

footPlatformVirtual::~footPlatformVirtual() { me->_n.shutdown(); }

bool footPlatformVirtual::init() //! Initialization of the node. Its datatype
                                 //! (bool) reflect the success in
                                 //! initialization
{
    _pubVirtualFootOutput = _n.advertise<custom_msgs::FootOutputMsg_v3>("/FI_Output/"+std::string(Platform_Names_2[_platform_id]), 0);
    
    _pubPlatformJointStates = _n.advertise<sensor_msgs::JointState>("/"+std::string(Platform_Names[_platform_id])+"_platform/platform_joint_publisher/joint_states", 0);
    
    _subVirtualFootInput = _n.subscribe<custom_msgs::FootInputMsg_v5>("/FI_Input/"+std::string(Platform_Names_2[_platform_id])
    , 1, boost::bind(&footPlatformVirtual::readVirtualFootInput, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    
    _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>("/"+std::string(Platform_Names[_platform_id])+"_platform/platform_joint_publisher/joint_states_2"
    , 1, boost::bind(&footPlatformVirtual::readPlatformJointStates, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  // Subscriber definitions
  signal(SIGINT, footPlatformVirtual::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[%s virtual_platform]: The virtual platform output is about to start ",Platform_Names[_platform_id]);
    return true;
  } 
  else {
    ROS_ERROR("[%s virtual_platform]: The ros node has a problem.",Platform_Names[_platform_id]);
    return false;
  }
}

void footPlatformVirtual::stopNode(int sig) { me->_stop = true; }

void footPlatformVirtual::run() {
  while (!_stop) {
    if (_flagPlatformConnected) {
          publishVirtualFootOutput();
          publishPlatformJointStates();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[%s virtual_platform]: Virtual platform stopped",Platform_Names[_platform_id]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void footPlatformVirtual::publishVirtualFootOutput() {
  //! Keep send the same valuest that the platform is broadcasting
  //_mutex.lock();
   _msgVirtualOutput.platform_id = _platform_id;
   _msgVirtualOutput.platform_stamp = ros::Time::now();
   _msgVirtualOutput.platform_machineState = TELEOPERATION;
   _msgVirtualOutput.platform_controllerType = TORQUE_CTRL;
  
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _msgVirtualOutput.platform_effortM[rosAxis[k]] = _ros_state_effort(rosAxis[k]);
    _msgVirtualOutput.platform_position[rosAxis[k]] = _ros_state_position(k) * inv_conversion_factor[rosAxis[k]];
    _msgVirtualOutput.platform_speed[rosAxis[k]] = _ros_state_velocity(k) * inv_conversion_factor[rosAxis[k]];
  }
  _pubVirtualFootOutput.publish(_msgVirtualOutput);
  //_mutex.unlock();
}

void footPlatformVirtual::readPlatformJointStates(const sensor_msgs::JointState::ConstPtr &msg) {  
  _ros_state_positionPrev=_ros_state_position;
  _msgTimePrev = _msgTime;
  _msgTime = msg->header.stamp.toSec();
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _ros_state_position(k) = msg->position[k];
    _ros_state_effort(k) = 0.0;
  }
  

  _ros_state_velocity= (_ros_state_position - _ros_state_positionPrev) / (_msgTime - _msgTimePrev);
  if (!_flagPlatformConnected) {
    _flagPlatformConnected = true;
  }
}


void footPlatformVirtual::readVirtualFootInput(const custom_msgs::FootInputMsg_v5::ConstPtr &msg)
{

}


void footPlatformVirtual::publishPlatformJointStates() {
  //! Keep send the same valuest that the platform is broadcasting
  //_mutex.lock();

  _msgPlatformJointStates.header.stamp = ros::Time::now();

  _msgPlatformJointStates.name.resize(NB_PLATFORM_AXIS);
  _msgPlatformJointStates.position.resize(NB_PLATFORM_AXIS);
  _msgPlatformJointStates.velocity.resize(NB_PLATFORM_AXIS);
  _msgPlatformJointStates.effort.resize(NB_PLATFORM_AXIS);
  
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _msgPlatformJointStates.name[k] = std::string(Platform_Names[_platform_id]) +"_"+ std::string(Platform_Axis_Names[k]);
    _msgPlatformJointStates.position[k] = _ros_state_position[k];
    _msgPlatformJointStates.velocity[k] = _ros_state_velocity[k];
    _msgPlatformJointStates.effort[k] = _ros_state_effort[k];
  }
  _pubPlatformJointStates.publish(_msgPlatformJointStates);
  //_mutex.unlock();
}