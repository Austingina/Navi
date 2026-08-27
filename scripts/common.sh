#!/bin/bash
# 定义脚本目录常量
export NAVI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAVI_ROOT_DIR="$(dirname "$NAVI_SCRIPT_DIR")"
echo $NAVI_ROOT_DIR
echo $NAVI_SCRIPT_DIR
source $NAVI_ROOT_DIR/ros2_ws/install/setup.bash

export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE="${NAVI_DISCOVERY_RANGE:-LOCALHOST}"

export RMW_IMPLEMENTATION="${NAVI_RMW:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain id=\"any\"><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

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

navi_reap_orphans() {
    local sess p args live="" orphans=""

    for sess in navi slam outdoor; do
        tmux has-session -t "$sess" 2>/dev/null || continue
        for p in $(tmux list-panes -s -t "$sess" -F '#{pane_pid}' 2>/dev/null); do
            # _navi_descendants 一行一个 pid，必须压成空格分隔：下面拿
            # case " $live " in *" $p "* 做整词匹配，留着换行会全部失配，
            # 于是在跑的节点会被当成孤儿全杀掉。
            live="$live $(_navi_descendants "$p" | tr '\n' ' ')"
        done
    done

    # 必须卡到 install/<pkg>/lib/ 这一层，节点的可执行文件都在那儿。只匹配 install/
    # 会把 rviz2 也算进来——它跑的是 /opt/ros 下的二进制，只是 -d 参数指向
    # install/fast_lio/share/.../*.rviz，而 rviz 是手起的调试工具，不该被清掉。
    for p in $(pgrep -f "$NAVI_ROOT_DIR/ros2_ws/install/[^ ]*/lib/" 2>/dev/null); do
        # 别把正在执行这个函数的脚本自己算进去。
        if [ "$p" = "$$" ] || [ "$p" = "$PPID" ]; then continue; fi
        case " $live " in *" $p "*) continue ;; esac
        orphans="$orphans $p"
    done

    [ -z "$orphans" ] && return 0

    echo "reap: 发现不属于任何会话的残留节点，清理中：" >&2
    for p in $orphans; do
        args="$(ps -o args= -p "$p" 2>/dev/null | cut -c1-100)"
        [ -n "$args" ] && echo "  [$p] $args" >&2
    done

    for p in $orphans; do kill "$p" 2>/dev/null; done
    sleep 2
    for p in $orphans; do
        kill -0 "$p" 2>/dev/null && kill -9 "$p" 2>/dev/null
    done
}