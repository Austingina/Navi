#! /bin/bash
source "$(dirname "$0")/common.sh"

cd $NAVI_ROOT_DIR/NaviAgent

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
CMD=(python server.py)
if [[ "$TEST" -eq 1 ]]; then
	CMD+=("-t")
fi
CMD+=(-m "$MAP_PATH")

# Execute
"${CMD[@]}"
