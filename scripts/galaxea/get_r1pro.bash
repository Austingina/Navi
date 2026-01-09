#! /bin/bash

source ~/galaxea/atc_standard-V2.1.3-20250807_15_22_50_aarch64/install/setup.bash
ros2 topic echo /hdas/bms

cd ~/galaxea/atc_standard-V2.1.3-20250807_15_22_50_aarch64/install
ros2 run rds_ros rds_ros_node
