#!/bin/bash
# 定义脚本目录常量
export NAVI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAVI_ROOT_DIR="$(dirname "$NAVI_SCRIPT_DIR")"
echo $NAVI_ROOT_DIR
echo $NAVI_SCRIPT_DIR
source $NAVI_ROOT_DIR/ros2_ws/install/setup.bash

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