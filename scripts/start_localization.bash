#! /bin/bash

source "$(dirname "$0")/common.sh"

# rcl_logging_spdlog 文件 sink 默认 flush_on(error)，INFO 级别不会即时落盘。
# 这里通过：1) 让 ROS2 节点 stderr 无缓冲；2) 关掉 ANSI 颜色；3) 让 ros2 launch
# (Python) 的 stdout 也无缓冲；再 4) tee 到我们自己的文件，绕开 spdlog 的缓冲，
# 实现 INFO 级别日志的实时落盘。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

# 解析参数：-v 打开 rviz；-s/--simulated 使用仿真时钟 (use_sim_time:=true)；
# -m/--map <pcd> 指定加载的点云地图（空值/缺省时回退到默认 pcd）。
RVIZ_USE=false
USE_SIM_TIME=false
MAP_PCD=""
while [ $# -gt 0 ]; do
    case "$1" in
        -v)
            RVIZ_USE=true
            ;;
        -s|--simulated)
            USE_SIM_TIME=true
            ;;
        -m|--map)
            shift; MAP_PCD="$1"
            ;;
        -m=*|--map=*)
            MAP_PCD="${1#*=}"
            ;;
    esac
    shift
done

# 缺省 pcd：调用方（server.py / start_slam.sh）通常会显式传 -m，未传时用此默认值兜底。
MAP_PCD="${MAP_PCD:-$NAVI_ROOT_DIR/maps/zhichengA.pcd}"

LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/localization_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"
echo "map_file_path=$MAP_PCD"

ros2 launch fast_lio odometry_lslidar.launch.py rviz:=$RVIZ_USE use_sim_time:=$USE_SIM_TIME map_file_path:=$MAP_PCD 2>&1 \
    | stdbuf -oL -eL tee -a "$LOG_FILE"
