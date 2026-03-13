#! /bin/bash
source "$(dirname "$0")/common.sh"


if [[ "$1" == "-v" ]]; then
    ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py rviz:=true map_file_path:=$NAVI_ROOT_DIR/maps/xunjian_2cm.pcd
else
    ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py rviz:=false map_file_path:=$NAVI_ROOT_DIR/maps/xunjian_2cm.pcd
fi