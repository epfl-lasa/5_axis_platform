#!/bin/bash
chmod 777 /dev/right_force_sensor
cd ~/catkin_hasler
source devel/setup.bash
roslaunch foot_platform_ros platform_ros.launch rviz:=false virtual_platform:=false fsensor:=true
