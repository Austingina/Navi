#!/usr/bin/env bash
# 用假 GPS 顶替真 GNSS 驱动，方便没信号/室内时联调室外栈。
#
#   ./start_fake_gps.sh            # 停真驱动节点 + 起假 GPS（固定点）
#   ./start_fake_gps.sh --follow-odom       # 假 GPS 跟随 LIO 里程计移动
#   ./start_fake_gps.sh --lat 22.5 --lon 113.9 --alt 30
#   ./start_fake_gps.sh --keep-real         # 不动真驱动（只在真驱动已停时用）
#   ./start_fake_gps.sh stop                # 停假 GPS
#   ./start_fake_gps.sh stop --restore      # 停假 GPS 并把真驱动拉回来
#
# 为什么要停真驱动：unicore_gnss 没星时照样 10Hz 发 /gps/fix（status=-1,
# lat=lon=0）。同一话题上两个发布者，消息交替到达，nav_node 取"最新一帧"判
# is_gps_available() 会 true/false 抖动 —— 比完全没有 GPS 更难排查。
#
# 只杀 robot-sensors 里 `ros2 launch unicore_gnss` 那一棵子树：run_driver_all.sh
# 没有子进程重启逻辑（每个驱动 `start` 后统一 `wait`），所以杀掉 RTK 这一支不会
#影响雷达/底盘/相机，也不会让 systemd 重启整个 service。闭源 rtk SDK 二进制留着
# 不管（它只往串口写，没有 ROS 话题）。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVI_ROOT_DIR="$(dirname "$SCRIPT_DIR")"
FAKE_GPS_PY="$NAVI_ROOT_DIR/ros2_ws/src/FAST_LIO/scripts/fake_gps_publisher.py"
SESSION="fakegps"
REAL_RTK_SCRIPT="/home/user/work/robot-sensors/scripts/run_driver_rtk.sh"

# 真驱动跑在 ROS_LOCALHOST_ONLY=1 下（robot-sensors/scripts/dds_env.sh），这里必须
# 一致，否则 DDS 发现不到对方。
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

LAT=22.57348767
LON=113.93442003
ALT=30.0
FOLLOW_ODOM=false
ODOM_YAW_OFFSET=0.0
NOISE_SIGMA=0.0
STATUS=2
RATE=10.0
KEEP_REAL=0
RESTORE=0
ACTION=start

while [ $# -gt 0 ]; do
  case "$1" in
    start|stop) ACTION="$1"; shift ;;
    --lat) LAT="$2"; shift 2 ;;
    --lon) LON="$2"; shift 2 ;;
    --alt) ALT="$2"; shift 2 ;;
    --follow-odom) FOLLOW_ODOM=true; shift ;;
    --odom-yaw-offset) ODOM_YAW_OFFSET="$2"; shift 2 ;;
    --noise) NOISE_SIGMA="$2"; shift 2 ;;
    --status) STATUS="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --keep-real) KEEP_REAL=1; shift ;;
    --restore) RESTORE=1; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 1 ;;
  esac
done

kill_real_gnss() {
  # 匹配 `ros2 launch unicore_gnss ...` 和 unicore_gnss_node 两级进程。
  local pids
  pids="$(pgrep -f 'unicore_gnss' 2>/dev/null | tr '\n' ' ' || true)"
  if [ -z "${pids// /}" ]; then
    echo "真 GNSS 节点没在跑，跳过。"
    return 0
  fi
  echo "停掉真 GNSS 节点 (pids: $pids)..."
  # shellcheck disable=SC2086
  kill -INT $pids 2>/dev/null || true
  for _ in 1 2 3 4 5 6; do
    pgrep -f 'unicore_gnss' >/dev/null 2>&1 || break
    sleep 0.5
  done
  pids="$(pgrep -f 'unicore_gnss' 2>/dev/null | tr '\n' ' ' || true)"
  if [ -n "${pids// /}" ]; then
    # shellcheck disable=SC2086
    kill -KILL $pids 2>/dev/null || true
  fi
}

if [ "$ACTION" = "stop" ]; then
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "停掉假 GPS (tmux: $SESSION)..."
    tmux kill-session -t "$SESSION"
  else
    echo "假 GPS 没在跑。"
  fi
  if [ "$RESTORE" = 1 ]; then
    if [ -x "$REAL_RTK_SCRIPT" ]; then
      echo "重新拉起真 GNSS 驱动: $REAL_RTK_SCRIPT"
      tmux new-session -d -s realgnss "bash $REAL_RTK_SCRIPT; exec bash"
      echo "已在 tmux 会话 realgnss 里启动（注意：它不再挂在 robot-sensors.service 下，"
      echo "  想彻底回到原状态就 sudo systemctl restart robot-sensors —— 会顺带重启雷达）。"
    else
      echo "找不到 $REAL_RTK_SCRIPT，无法自动恢复。" >&2
    fi
  fi
  exit 0
fi

if [ ! -f "$FAKE_GPS_PY" ]; then
  echo "找不到 $FAKE_GPS_PY" >&2
  exit 1
fi

if [ "$KEEP_REAL" = 0 ]; then
  kill_real_gnss
else
  echo "--keep-real：不动真驱动（若它还在发 /gps/fix，两个发布者会互相打架）。"
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

CMD="source /opt/ros/jazzy/setup.bash; \
export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY; \
python3 $FAKE_GPS_PY --ros-args \
  -p lat:=$LAT -p lon:=$LON -p alt:=$ALT \
  -p status:=$STATUS -p rate:=$RATE \
  -p noise_sigma:=$NOISE_SIGMA \
  -p follow_odom:=$FOLLOW_ODOM \
  -p odom_yaw_offset:=$ODOM_YAW_OFFSET"

tmux new-session -d -s "$SESSION" "$CMD; exec bash"

echo "假 GPS 已启动（tmux 会话 $SESSION，看日志: tmux attach -t $SESSION）"
echo "  位置: lat=$LAT lon=$LON alt=$ALT  status=$STATUS  ${RATE}Hz  follow_odom=$FOLLOW_ODOM"
echo "  验证: ros2 topic echo /gps/fix --once"
echo "  停止: $0 stop [--restore]"
