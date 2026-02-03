#! /bin/bash
source "$(dirname "$0")/common.sh"


ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py pcd_filename:=xunjian_2cm.pcd
