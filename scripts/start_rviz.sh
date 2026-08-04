#! /bin/bash

source "$(dirname "$0")/common.sh"

# 解析参数：-s/--simulated 使用仿真时钟 (use_sim_time:=true)；
#           -o/--outdoor 加载室外 rviz 配置 (rviz/outdoor.rviz)，缺省用 launch 里的默认配置。
USE_SIM_TIME=false
RVIZ_CFG=""
for arg in "$@"; do
    case "$arg" in
        -s|--simulated)
            USE_SIM_TIME=true
            ;;
        -o|--outdoor)
            RVIZ_CFG="$NAVI_ROOT_DIR/ros2_ws/src/FAST_LIO/rviz/outdoor.rviz"
            ;;
    esac
done

LAUNCH_ARGS=(use_sim_time:=$USE_SIM_TIME)
if [ -n "$RVIZ_CFG" ]; then
    LAUNCH_ARGS+=(rviz_cfg:=$RVIZ_CFG)
fi

ros2 launch fast_lio rviz_only.launch.py "${LAUNCH_ARGS[@]}"
