#! /bin/bash
# 启动「室外 GPS」栈：定位(heading-align) + Nav2 + campus_nav 路网规划器。
# 一条 launch 全包（内部已错峰 t=0 定位 / t=3 Nav2 / t=7 路网）。
#
# 用法:
#   ./start_outdoor.bash            # 真实环境（use_sim_time:=false）
#   ./start_outdoor.bash -s         # 仿真环境（use_sim_time:=true）
#
# 说明：与 start_localization.bash / start_navigation.bash 同风格——统一日志实时落盘、
# tee 到 logs/。雷达(start_lidar) 与 NaviSkill(start_mcp) 不在本脚本内，需常驻另开。

source "$(dirname "$0")/common.sh"

# 让 INFO 级别日志也实时落盘，绕开 spdlog 文件 sink 的 flush_on(error) 缓冲。
# 详见 start_localization.bash 的注释。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

# 解析参数：-s/--simulated 使用仿真时钟 (use_sim_time:=true)，默认真实环境 false。
USE_SIM_TIME=false
for arg in "$@"; do
    case "$arg" in
        -s|--simulated)
            USE_SIM_TIME=true
            ;;
    esac
done
echo "NAVI_SIM_ARG: $NAVI_SIM_ARG"
LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/outdoor_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"
echo "use_sim_time=$USE_SIM_TIME"

ros2 launch fast_lio outdoor_bringup_heading_align.launch.py \
    use_sim_time:=$USE_SIM_TIME 2>&1 \
    | stdbuf -oL -eL tee -a "$LOG_FILE"
