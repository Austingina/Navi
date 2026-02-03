#! /bin/bash
source "$(dirname "$0")/common.sh"


if [[ "$1" == "-v" ]]; then
    ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py rviz:=true pcd_filename:=xunjian_2cm.pcd
else
    ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py rviz:=false pcd_filename:=xunjian_2cm.pcd
fi