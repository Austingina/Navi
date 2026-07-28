#! /bin/bash
# 启动共享常驻会话 session=navi（navi.base.yaml）：雷达 + NaviSkill。
# 这套与导航模式无关、常驻不重启；SLAM / 室外由 start_slam.sh / start_outdoor.sh
# 各自的会话切换（见 switch_nav_mode.sh）。
#
# 用法:
#   ./start_navi.sh          # 真实环境：启动激光雷达 + NaviSkill
#   ./start_navi.sh -s       # 仿真环境：跳过激光雷达（用仿真环境已有的雷达信号），只起 NaviSkill
source "$(dirname "$0")/common.sh"

# -s/--simulated：仿真环境。用 NAVI_SIM_ARG 统一标记（与 start_slam.sh / start_outdoor.sh
# 一致），非空即仿真——navi.base.yaml 的雷达 pane 会据此跳过 start_lidar。
SIM_ARG=""
for arg in "$@"; do
    case "$arg" in
        -s|--simulated) SIM_ARG="--simulated" ;;
    esac
done

# 只重启 navi（base）会话本身，不动 slam / outdoor 会话。
tmux kill-session -t navi 2>/dev/null

# 将关键环境变量注入 tmux 全局环境，确保所有 pane 都能继承（yaml 里用 $NAVI_SCRIPT_DIR）。
tmux set-environment -g NAVI_SCRIPT_DIR "$NAVI_SCRIPT_DIR"
tmux set-environment -g NAVI_ROOT_DIR "$NAVI_ROOT_DIR"
tmux set-environment -g NAVI_SIM_ARG "$SIM_ARG"

tmuxp load -d "$NAVI_SCRIPT_DIR/navi.base.yaml"
