#!/bin/bash
# 一键切换导航模式（tmux 会话 + 软状态一体）。
#
# 切换以 tmux 会话为单位：slam 会话（navi.slam.yaml）↔ outdoor 会话（navi.outdoor.yaml）
# 互斥切换，session=navi（雷达 + NaviSkill）常驻不动。本脚本在会话切换之外，额外
# 先给 NaviSkill 打一个 set_nav_mode（干净交接：停车、取消旧目标、复位定位判据），
# 切完再轮询 get_status().localized 等目标栈定位就绪。
#
#   1) 调 NaviSkill set_nav_mode(<target>)                       —— 软状态干净交接
#   2) start_slam.sh / start_outdoor.sh                          —— 杀旧会话 + 起目标会话
#   3) 轮询 get_status().localized                               —— 等目标栈定位就绪
#
# 用法:
#   ./switch_nav_mode.sh gps            # 切到室外 GPS（真实环境）
#   ./switch_nav_mode.sh slam           # 切回室内 SLAM
#   ./switch_nav_mode.sh gps -s         # 仿真环境（use_sim_time:=true）
#   ./switch_nav_mode.sh gps --ws ws://127.0.0.1:9999/ws         # 指定 NaviSkill 端口(测试9999)
#   ./switch_nav_mode.sh slam --no-wait --timeout 90

# 先 source（colcon 的 setup.bash 会引用 COLCON_TRACE 等未定义变量，在 nounset 下会
# 直接报错退出），之后再开 set -u 保护本脚本自身逻辑。
source "$(dirname "$0")/common.sh"
set -u

PY="$NAVI_ROOT_DIR/NaviSkill/vpr/.venv/bin/python3"
RPC="$NAVI_SCRIPT_DIR/tools/navi_rpc.py"

# ---- 参数解析 ----
TARGET=""
SIM=""
WS="ws://127.0.0.1:9000/ws"
WAIT=1
READY_TIMEOUT=120
while [ $# -gt 0 ]; do
    case "$1" in
        slam|gps) TARGET="$1" ;;
        -s|--simulated) SIM="-s" ;;
        --ws) shift; WS="$1" ;;
        --no-wait) WAIT=0 ;;
        --timeout) shift; READY_TIMEOUT="$1" ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
    shift
done

if [ "$TARGET" != "slam" ] && [ "$TARGET" != "gps" ]; then
    echo "用法: $0 <slam|gps> [-s] [--ws URL] [--no-wait] [--timeout SEC]"
    exit 1
fi

# 目标模式 → 对应的会话启动脚本（其内部已做 slam/outdoor 互斥：先杀另一会话再起自己）
if [ "$TARGET" = "gps" ]; then
    START_SCRIPT="$NAVI_SCRIPT_DIR/start_outdoor.sh"
else
    START_SCRIPT="$NAVI_SCRIPT_DIR/start_slam.sh"
fi

log() { echo "[switch_nav_mode] $*"; }

# ---- 1) 软状态切换 + 干净交接（尽力而为，NaviSkill 未起也继续切会话）----
log "1/3 set_nav_mode($TARGET) via $WS"
if "$PY" "$RPC" set_nav_mode "{\"mode\":\"$TARGET\"}" --ws "$WS"; then
    log "NaviSkill 已切到 $TARGET 模式（已停车/取消旧目标/复位定位判据）"
else
    rc=$?
    log "警告：set_nav_mode 调用未成功 (rc=$rc)。可能 NaviSkill 未运行或端口不对；继续切换会话。"
fi

# ---- 2) 切换 tmux 会话（杀旧栈会话 + 起目标栈会话）----
log "2/3 启动目标会话: $TARGET ${SIM:+(仿真)}  ($START_SCRIPT)"
bash "$START_SCRIPT" $SIM

# ---- 3) 等目标栈定位就绪 ----
if [ "$WAIT" -eq 0 ]; then
    log "3/3 跳过就绪等待 (--no-wait)。可用: $PY $RPC get_status --ws $WS 自查。"
    exit 0
fi

log "3/3 等待 $TARGET 定位就绪（最多 ${READY_TIMEOUT}s）..."
deadline=$(( $(date +%s) + READY_TIMEOUT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    loc="$("$PY" "$RPC" get_status --field localized --ws "$WS" 2>/dev/null || true)"
    if [ "$loc" = "true" ]; then
        log "✓ $TARGET 定位已就绪，切换完成。"
        exit 0
    fi
    sleep 2
done

log "✗ 等待超时：$TARGET 定位在 ${READY_TIMEOUT}s 内未就绪。"
log "  排障：$PY $RPC get_status --ws $WS ；tmux 会话：tmux ls"
exit 1
