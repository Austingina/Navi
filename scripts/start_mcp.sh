#! /bin/bash
source "$(dirname "$0")/common.sh"

cd $NAVI_ROOT_DIR/NaviSkill

# Parse arguments: support -t (test) and -m <map_path>
TEST=0
MAP_PATH=""
while [[ $# -gt 0 ]]; do
	case "$1" in
		-t)
			TEST=1
			shift
			;;
		-m)
			MAP_PATH="$2"
			shift 2
			;;
		*)
			# ignore unknown args
			shift
			;;
	esac
done

# Default map if not provided
if [[ -z "$MAP_PATH" ]]; then
	MAP_PATH="$NAVI_ROOT_DIR/maps/multifloor/multifloor.json"
fi

# Build command
CMD=(/usr/bin/python3 server.py)
if [[ "$TEST" -eq 1 ]]; then
	CMD+=("-t")
fi
CMD+=(-m "$MAP_PATH")

# 让 INFO 级别日志也实时落盘，绕开 spdlog 文件 sink 的 flush_on(error) 缓冲。
# 详见 start_localization.bash 的注释。
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

LOG_DIR="$NAVI_ROOT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/mcp_$(date +%Y%m%d_%H%M%S).log"
echo "logging to $LOG_FILE"

# Execute
"${CMD[@]}" 2>&1 | stdbuf -oL -eL tee -a "$LOG_FILE"
