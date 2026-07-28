#! /bin/bash
# 启动 SLAM 室内定位栈 session=slam（navi.yaml）：FAST-LIO 定位 + r1pro Nav2。
# 与室外 GPS 栈互斥（二者都会发布 map->odom TF，不能同时跑），故先杀 outdoor 会话。
# 雷达 / NaviSkill（session=navi）不受影响，需另用 start_navi.sh 常驻。
#
# 用法:
#   ./start_slam.sh          # 真实环境
#   ./start_slam.sh -s       # 仿真环境（use_sim_time:=true）
source "$(dirname "$0")/common.sh"

SIM_ARG=""
for arg in "$@"; do
    case "$arg" in
        -s|--simulated) SIM_ARG="--simulated" ;;
    esac
done

# 互斥：切到 SLAM 前先停室外会话；再重启 slam 会话本身。
tmux kill-session -t outdoor 2>/dev/null
tmux kill-session -t slam 2>/dev/null

tmux set-environment -g NAVI_SCRIPT_DIR "$NAVI_SCRIPT_DIR"
tmux set-environment -g NAVI_ROOT_DIR "$NAVI_ROOT_DIR"
tmux set-environment -g NAVI_SIM_ARG "$SIM_ARG"

tmuxp load -d "$NAVI_SCRIPT_DIR/navi.yaml"
