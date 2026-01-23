#! /bin/bash

source "$(dirname "$0")/common.sh"



if [[ "$1" == "-v" ]]; then
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=true map_file_path:=$NAVI_ROOT_DIR/maps/lab_2cm.pcd
else
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=false map_file_path:=$NAVI_ROOT_DIR/maps/lab_2cm.pcd
fi
