#! /bin/bash
# 启动室外 GPS 栈 session=outdoor（navi.outdoor.yaml）：
# outdoor_bringup_heading_align（定位 heading-align + Nav2 + campus_nav 路网）。
# 与 SLAM 室内栈互斥（二者都会发布 map->odom TF，不能同时跑），故先杀 slam 会话。
# 雷达 / NaviSkill（session=navi）不受影响，需另用 start_navi.sh 常驻。
#
# 用法:
#   ./start_outdoor.sh       # 真实环境
#   ./start_outdoor.sh -s    # 仿真环境（use_sim_time:=true）
source "$(dirname "$0")/common.sh"

SIM_ARG=""
for arg in "$@"; do
    case "$arg" in
        -s|--simulated) SIM_ARG="--simulated" ;;
    esac
done
echo "SIM_ARG: $SIM_ARG"
# 互斥：切到室外前先停 SLAM 会话；再重启 outdoor 会话本身。
# 必须用 navi_kill_session_wait 而非裸 kill-session：后者立即返回，旧栈的同名 nav2
# 节点还在退出中就被新栈的 lifecycle_manager 发现，会永久卡在
# "Waiting for service <node>/get_state..."。详见 common.sh 里的说明。
navi_kill_session_wait slam
navi_kill_session_wait outdoor
navi_reap_orphans

# 注意：set-environment -g 需要 tmux server 已在运行，否则静默失败。此处 server 常
# 还不存在，真正让变量进入 pane 的是：tmuxp load 拉起 server 时会继承本进程已 export
# 的变量。NAVI_SCRIPT_DIR/NAVI_ROOT_DIR 由 common.sh export，故必须同样 export
# NAVI_SIM_ARG，否则它只靠(失败的)set-environment，在 pane 里会是空的。
export NAVI_SIM_ARG="$SIM_ARG"
tmux set-environment -g NAVI_SCRIPT_DIR "$NAVI_SCRIPT_DIR"
tmux set-environment -g NAVI_ROOT_DIR "$NAVI_ROOT_DIR"
tmux set-environment -g NAVI_SIM_ARG "$SIM_ARG"

tmuxp load -d "$NAVI_SCRIPT_DIR/navi.outdoor.yaml"
