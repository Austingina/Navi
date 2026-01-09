#! /bin/bash

source ~/work/Navi/ros2_ws/install/setup.bash



if [[ "$1" == "-v" ]]; then
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=true
else
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=false
fi
