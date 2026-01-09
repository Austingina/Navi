#! /bin/bash

source ~/work/Navi/ros2_ws/install/setup.bash



if [[ "$1" == "-v" ]]; then
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=true map_file_path:=/home/nvidia/maps/lab525_5.pcd
else
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=false map_file_path:=/home/nvidia/maps/lab525_5.pcd
fi
