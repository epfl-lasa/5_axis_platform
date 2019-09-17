cmake_minimum_required(VERSION 2.8.3)

project(5_axis_platform)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy )

include_directories(${catkin_INCLUDE_DIRS})
