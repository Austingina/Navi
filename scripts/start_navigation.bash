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

# Nav2 的 costmap 没有 map->base_link 就拒绝 activate，而这个失败不可恢复：
# lifecycle_manager 会 "Aborting bringup"，之后所有目标都被 "Action server is
# inactive" 拒掉，只能整栈重启。原来靠 navi.slam.yaml 里的 sleep 8 挡这个race，
# 但 FAST-LIO 要先加载 PCD 再定位，经常超过 8s。这里改成等 TF 真的出现。
# 仿真下还会一并等 /clock（详见 wait_for_tf.py 的说明）。
SIM_FLAG=""
[ "$USE_SIM_TIME" = "true" ] && SIM_FLAG="--sim"
python3 "$NAVI_SCRIPT_DIR/wait_for_tf.py" map base_link --timeout 120 $SIM_FLAG \
    2>&1 | stdbuf -oL -eL tee -a "$LOG_FILE"
# 超时也继续起 Nav2：让它自己报错比静默没有导航更好排查（上面已留下告警）。

ros2 launch fast_lio r1pro_nav2.launch.py use_sim_time:=$USE_SIM_TIME 2>&1 \
    | stdbuf -oL -eL tee -a "$LOG_FILE"
