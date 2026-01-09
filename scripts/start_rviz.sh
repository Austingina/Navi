#! /bin/bash

source "$(dirname "$0")/common.sh"
ros2 launch fast_lio rviz_only.launch.py
