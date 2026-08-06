#! /bin/bash
# 启动共享常驻会话 session=navi（navi.base.yaml）：雷达 + NaviSkill。
# 这套与导航模式无关、常驻不重启。SLAM / 室外导航栈的"首次拉起"由 NaviSkill(server.py)
# 依据 start_mcp.sh 的 -s 参数自动决定并调起 start_slam.sh / start_outdoor.sh。
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

# 启动前先清理环境：退出所有 Navi 相关 tmux 会话（navi / slam / outdoor）。
bash "$NAVI_SCRIPT_DIR/stop_navi.sh"

# 将关键环境变量注入 tmux 全局环境，确保所有 pane 都能继承（yaml 里用 $NAVI_SCRIPT_DIR）。
# 注意：set-environment -g 需要 tmux server 已在运行，否则静默失败。此处 server 常还
# 不存在，真正让变量进入 pane 的是：tmuxp load 拉起 server 时会继承本进程已 export 的
# 变量。NAVI_SCRIPT_DIR/NAVI_ROOT_DIR 由 common.sh export，故必须同样 export
# NAVI_SIM_ARG，否则它只靠(失败的)set-environment，在 pane 里会是空的。
export NAVI_SIM_ARG="$SIM_ARG"
tmux set-environment -g NAVI_SCRIPT_DIR "$NAVI_SCRIPT_DIR"
tmux set-environment -g NAVI_ROOT_DIR "$NAVI_ROOT_DIR"
tmux set-environment -g NAVI_SIM_ARG "$SIM_ARG"

tmuxp load -d "$NAVI_SCRIPT_DIR/navi.base.yaml"
