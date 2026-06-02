#! /bin/bash

source "$(dirname "$0")/common.sh"

# rcl_logging_spdlog 文件 sink 默认 flush_on(error)，INFO 级别不会即时落盘。
# 这里通过：1) 让 ROS2 节点 stderr 无缓冲；2) 关掉 ANSI 颜色；3) 让 ros2 launch
# (Python) 的 stdout 也无缓冲；再 4) tee 到我们自己的文件，绕开 spdlog 的缓冲，
# 实现 INFO 级别日志的实时落盘。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/localization_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"

if [[ "$1" == "-v" ]]; then
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=true map_file_path:=$NAVI_ROOT_DIR/mapdata/408/2f.pcd 2>&1 \
        | stdbuf -oL -eL tee -a "$LOG_FILE"
else
    ros2 launch fast_lio odometry_lslidar.launch.py rviz:=false map_file_path:=$NAVI_ROOT_DIR/mapdata/408/2f.pcd 2>&1 \
        | stdbuf -oL -eL tee -a "$LOG_FILE"
fi
