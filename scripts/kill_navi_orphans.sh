#!/usr/bin/env bash
# 清理上一次 bringup 残留/孤儿节点，避免同名节点重复。
#
# 背景：某些节点（尤其 Python 的 elevation_mapping_node）在 launch 被强杀后不会
# 随之退出，会变成 PPID=1 的孤儿继续发布。多份同名节点叠在同一个 topic 上会让
# /grid_map_from_raw_pointcloud 乱跳，并把 terrain_scan 灌爆，导致 /livox_scan2
# 饿死（障碍消失）。
#
# 用法：
#   scripts/kill_navi_orphans.sh          # 清理
#   scripts/kill_navi_orphans.sh -n       # 只列出，不杀（dry-run）
#
# 说明：只匹配本栈已知的节点可执行，不动其它进程。outdoor_bringup.launch.py 已在
# 启动前自动调用等价逻辑；此脚本供手动清理或其它 launch 复用。
set -u

DRY_RUN=0
[ "${1:-}" = "-n" ] && DRY_RUN=1

PATTERNS=(
  "elevation_mapping_node.py"
  "cloud_to_laserscan/terrain_scan"
  "cloud_to_laserscan/cloud_to_2d_laserscan"
)

self_pid=$$

collect() {
  local pat="$1"
  # -f 匹配整条命令行；排除本脚本自身
  pgrep -f "$pat" 2>/dev/null | grep -vw "$self_pid" || true
}

total=0
for pat in "${PATTERNS[@]}"; do
  for pid in $(collect "$pat"); do
    kill -0 "$pid" 2>/dev/null || continue
    if [ "$DRY_RUN" = "1" ]; then
      echo "[dry-run] would kill $pid  ($pat)"
    else
      echo "SIGTERM $pid  ($pat)"
      kill "$pid" 2>/dev/null || true
    fi
    total=$((total + 1))
  done
done

if [ "$DRY_RUN" = "1" ]; then
  echo "[kill_navi_orphans] dry-run: matched $total process(es)"
  exit 0
fi

if [ "$total" -gt 0 ]; then
  sleep 1.5
  # 顽固残留升级为 SIGKILL
  for pat in "${PATTERNS[@]}"; do
    for pid in $(collect "$pat"); do
      kill -0 "$pid" 2>/dev/null || continue
      echo "SIGKILL $pid  ($pat)"
      kill -9 "$pid" 2>/dev/null || true
    done
  done
fi

echo "[kill_navi_orphans] done, terminated $total process(es)"
