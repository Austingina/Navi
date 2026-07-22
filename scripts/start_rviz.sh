#! /bin/bash

source "$(dirname "$0")/common.sh"

# 解析参数：-s / --simulated 使用仿真时钟 (use_sim_time:=true)
USE_SIM_TIME=false
for arg in "$@"; do
    case "$arg" in
        -s|--simulated)
            USE_SIM_TIME=true
            ;;
    esac
done

ros2 launch fast_lio rviz_only.launch.py use_sim_time:=$USE_SIM_TIME
