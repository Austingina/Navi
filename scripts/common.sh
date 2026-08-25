#!/bin/bash
# 定义脚本目录常量
export NAVI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAVI_ROOT_DIR="$(dirname "$NAVI_SCRIPT_DIR")"
echo $NAVI_ROOT_DIR
echo $NAVI_SCRIPT_DIR
source $NAVI_ROOT_DIR/ros2_ws/install/setup.bash

# DDS 发现范围：必须显式置 ROS_LOCALHOST_ONLY=0，否则整栈的节点互相发现不了。
#
# ~/.bashrc 会 source robot-sensors/scripts/dds_env.sh，那里有一行
#   export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# 于是每个交互 shell（包括 tmux pane）都带着 ROS_LOCALHOST_ONLY=1。而这个变量在
# Jazzy 里已废弃，且会【静默覆盖】同时设置的 ROS_AUTOMATIC_DISCOVERY_RANGE——
# 启动日志里那两行一直被当噪声的告警说的就是这件事：
#   [rcl] ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled.
#   [rcl] 'localhost_only' is enabled, 'automatic_discovery_range' ... will be ignored.
#
# localhost_only=1 时 Fast DDS 不加入 UDP 组播（域 0 里绑 7400 的 socket 只剩没设这个
# 变量的那几个 systemd 服务），只靠一条容量有限的本机路径做发现。参与者少时看不出
# 问题，超过阈值后新起的参与者之间就互相发现不了，且各节点自身日志全部正常，很难查。
# 2026-08-24 同机实测：18 个节点时 talker/listener 17→17 通，起满 Navi 栈到 51 个
# 节点后同一测试 15→0；仅把这一个变量改成 0（发现走 SUBNET 组播）即恢复 15→15。
#
# 注意 robot-sensors 的驱动由 systemd 拉起、不经过本文件，仍是 localhost_only=1，
# 域内因此是混合模式。根治要改 dds_env.sh，需与传感器组协商。
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

# 递归列出 pid 及其全部后代（pane 里是 bash -> ros2 launch -> 各节点进程）。
_navi_descendants() {
    local pid="$1" kid
    echo "$pid"
    for kid in $(ps -o pid= --ppid "$pid" 2>/dev/null); do
        _navi_descendants "$kid"
    done
}

# 杀掉 tmux 会话并等它的进程树真正回收，再返回。
#
# 为什么必须等：tmux kill-session 只发 SIGHUP 就立即返回，而 ros2 launch 的优雅退出
# 要数秒（nav2 costmap、FAST-LIO 点云）。这个窗口里新旧两栈的同名节点
# （behavior_server / smoother_server / controller_server …）会同时出现在 DDS 图中，
# 新的 lifecycle_manager 可能发现到正在退出的那个端点，于是永久卡在
#   [lifecycle_manager_navigation]: Waiting for service behavior_server/get_state...
# 此时 Nav2 一个节点都不会被 configure，整栈是死的。
#
# 用法: navi_kill_session_wait <session> [超时秒数=20] [DDS 静置秒数=1]
navi_kill_session_wait() {
    local session="$1"
    local timeout="${2:-20}"
    local settle="${3:-1}"

    tmux has-session -t "$session" 2>/dev/null || return 0

    # kill-session 之后 pane 就查不到了，必须先把整棵进程树抓下来。
    local tree p
    tree="$(for p in $(tmux list-panes -s -t "$session" -F '#{pane_pid}' 2>/dev/null); do
                _navi_descendants "$p"
            done | sort -u)"

    tmux kill-session -t "$session" 2>/dev/null

    local alive deadline=$((SECONDS + timeout))
    while :; do
        alive=""
        for p in $tree; do
            kill -0 "$p" 2>/dev/null && alive="$alive $p"
        done
        [ -z "$alive" ] && break
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "warn: session '$session' 等待 ${timeout}s 后仍有残留进程，强制 SIGKILL:$alive" >&2
            for p in $alive; do kill -9 "$p" 2>/dev/null; done
            break
        fi
        sleep 0.3
    done

    # 进程没了不等于 DDS 已经把 participant 从图里摘干净，留一点静置时间。
    sleep "$settle"
    echo "killed session: $session (进程树已回收)"
}