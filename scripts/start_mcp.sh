#! /bin/bash
source "$(dirname "$0")/common.sh"

cd $NAVI_ROOT_DIR/NaviSkill

# 所有参数原样透传给 server.py（见 server.py main() 中的 argparse 定义）：
#   -t, --test                    测试环境
#   -m, --map <src>               多楼层地图来源，支持两种形式：
#                                   * 本地路径，如 /home/nvidia/Navi/maps/408/408.json
#                                   * mapserver URL，如 http://10.239.18.181:8080/mapserver/408
#                                     （首次会下载到 $NAVI_ROOT_DIR/maps/<id>/，之后增量同步；
#                                     网络失败但本地已有缓存时自动降级使用本地版本）
#   -s, --submap <slam_map_id/submap_id>
#                                 初始定位目标，严格 slam_map_id/submap_id 形式（如 caihong/2f、
#                                 单文件地图用 default/2f）。给出 -s 即进入 SLAM 室内模式：server.py
#                                 解析该 submap 的 pcd 并自动调起 start_slam.sh -m <pcd>；
#                                 不给 -s 则进入室外 GPS 模式，自动调起 start_outdoor.sh。
#                                 （仿真时钟经 $NAVI_SIM_ARG 透传给被调起的启动脚本）
#       --image_topic <topic>     VPR 使用的 CompressedImage 话题
#       --vpr_device <cuda|cpu>   VPR 子进程使用的设备
#
# 唯一的便利逻辑：若用户没显式给 -m/--map，则填入默认的 408.json。
DEFAULT_MAP="$NAVI_ROOT_DIR/maps/408/408.json"

HAS_MAP=0
for arg in "$@"; do
	case "$arg" in
		-m|--map|-m=*|--map=*)
			HAS_MAP=1
			break
			;;
	esac
done

CMD=($NAVI_ROOT_DIR/NaviSkill/vpr/.venv/bin/python3 server.py)
if [[ $HAS_MAP -eq 0 ]]; then
	CMD+=(-m "$DEFAULT_MAP")
fi
CMD+=("$@")

# 让 INFO 级别日志也实时落盘，绕开 spdlog 文件 sink 的 flush_on(error) 缓冲。
# 详见 start_localization.bash 的注释。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/mcp_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"
echo "command: ${CMD[*]}"

"${CMD[@]}" 2>&1 | stdbuf -oL -eL tee -a "$LOG_FILE"
