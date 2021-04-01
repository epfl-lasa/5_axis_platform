cmake_minimum_required(VERSION 2.8.3)
project(foot_platform_ros)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  leg_as_robot
  roscpp
  tf2
  tf2_eigen
  tf2_ros
  tf2_kdl
  eigen_conversions
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  custom_msgs
  utils_math
  urdf
  kdl_parser
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#Generate dynamic_reconfigure parameters

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy std_msgs tf2 tf2_ros tf2_eigen tf2_kdl custom_msgs dynamic_reconfigure  utils_math custom_msgs urdf kdl_parser eigen_conversions leg_as_robot
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(footStatePublisher src/footStatePublisher.cpp src/footStatePublisher_node.cpp)
add_executable(footHapticController src/footHapticController.cpp src/footHapticController_node.cpp)
add_executable(footPlatformVirtual src/footPlatformVirtual.cpp src/footPlatformVirtual_node.cpp)
add_executable(footForceMeasModifier src/footForceMeasModifier.cpp src/footForceMeasModifier_node.cpp)
add_executable(mixedPlatformStatePublisher src/mixedPlatformStatePublisher.cpp src/mixedPlatformStatePublisher_node.cpp)

target_link_libraries(footStatePublisher ${catkin_LIBRARIES})
target_link_libraries(footHapticController ${catkin_LIBRARIES})
target_link_libraries(footPlatformVirtual ${catkin_LIBRARIES})
target_link_libraries(footForceMeasModifier ${catkin_LIBRARIES})
target_link_libraries(mixedPlatformStatePublisher ${catkin_LIBRARIES})

# add_dependencies(footStatePublisher ${PROJECT_NAME}_gencpp)
# add_dependencies(footStatePublisher ${PROJECT_NAME}_gencfg)




