#! /bin/bash
# 启动 SLAM 室内定位栈 session=slam（navi.slam.yaml）：FAST-LIO 定位 + r1pro Nav2。
# 与室外 GPS 栈互斥（二者都会发布 map->odom TF，不能同时跑），故先杀 outdoor 会话。
# 雷达 / NaviSkill（session=navi）不受影响，需另用 start_navi.sh 常驻。
#
# 用法:
#   ./start_slam.sh                       # 真实环境，使用默认 pcd
#   ./start_slam.sh -m /path/to/map.pcd   # 指定加载的点云地图 pcd（透传给 start_localization.bash）
#   ./start_slam.sh -s                    # 仿真环境（use_sim_time:=true）
#
# 通常由 server.py（start_mcp.sh 带 -s 时）依据 slam_map/submap 解析出 pcd 后自动调起。
source "$(dirname "$0")/common.sh"

SIM_ARG=""
MAP_PCD=""
while [ $# -gt 0 ]; do
    case "$1" in
        -s|--simulated) SIM_ARG="--simulated" ;;
        -m|--map) shift; MAP_PCD="$1" ;;
        -m=*|--map=*) MAP_PCD="${1#*=}" ;;
    esac
    shift
done

# 互斥：切到 SLAM 前先停室外会话；再重启 slam 会话本身。
tmux kill-session -t outdoor 2>/dev/null
tmux kill-session -t slam 2>/dev/null
# 同 start_navi.sh：export 保证 tmuxp 拉起 server 时能继承；set-environment 覆盖 server 已存在的时序。
export NAVI_SIM_ARG="$SIM_ARG"
export NAVI_MAP_PCD="$MAP_PCD"
tmux set-environment -g NAVI_SCRIPT_DIR "$NAVI_SCRIPT_DIR"
tmux set-environment -g NAVI_ROOT_DIR "$NAVI_ROOT_DIR"
tmux set-environment -g NAVI_SIM_ARG "$SIM_ARG"
tmux set-environment -g NAVI_MAP_PCD "$MAP_PCD"

tmuxp load -d "$NAVI_SCRIPT_DIR/navi.slam.yaml"
