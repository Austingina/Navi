#! /bin/bash

source "$(dirname "$0")/common.sh"

# 让 INFO 级别日志也实时落盘，绕开 spdlog 文件 sink 的 flush_on(error) 缓冲。
# 详见 start_localization.bash 的注释。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

# 解析参数：--simulated 使用仿真时钟 (use_sim_time:=true)
USE_SIM_TIME=false
for arg in "$@"; do
    case "$arg" in
        -s|--simulated)
            USE_SIM_TIME=true
            ;;
    esac
done

LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/navigation_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"

ros2 launch fast_lio r1pro_nav2.launch.py use_sim_time:=$USE_SIM_TIME 2>&1 \
    | stdbuf -oL -eL tee -a "$LOG_FILE"
