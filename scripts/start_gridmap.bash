#! /bin/bash

source ~/work/Navi/ros2_ws/install/setup.bash

ros2 launch grid_map_pcl grid_map_pcl_loader_launch.py pcd_filename:=120w5floor.pcd
