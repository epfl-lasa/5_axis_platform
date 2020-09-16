#!/bin/bash
platformID=$1
chmod 777 /dev/${platformID}_force_sensor
cd ~/catkin_hasler
source devel/setup.bash
roslaunch foot_platform_ros platform_ros.launch rviz:=false virtual_platform:=false id:=${platformID}
