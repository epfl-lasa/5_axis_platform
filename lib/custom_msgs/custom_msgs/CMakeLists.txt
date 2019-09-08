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
	FootInputMsg.msg
	FootInputMsg_v2.msg
	FootOutputMsg.msg
	FootOutputMsg_v2.msg
	FeetOutputMsg.msg)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options( cfg/frictionID.cfg) 

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
 )


## Generate added messages and services

#add dynamic reconfigure api

include_directories(${catkin_INCLUDE_DIRS})
