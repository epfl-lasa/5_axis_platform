cmake_minimum_required(VERSION 2.8.3)

project(custom_msgs)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(
  DIRECTORY msg
	FILES 
	FootInputMsg_v2.msg
	FootOutputMsg_v2.msg
	FeetOutputMsg_v2.msg
)

add_service_files(
  FILES
  setControllerSrv.srv
  setStateSrv.srv
)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

## Generate added messages and services
