#! /bin/bash

set -e

# 用法：
#   record_vpr_data.sh -s <submap_id>
#   例如：record_vpr_data.sh -s 2f
#   录制结果会落到 ~/vpr_data/<submap_id>/rosbag_<YYYYMMDD_HHMMSS>/

SUBMAP_ID=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        -s)
            SUBMAP_ID="$2"
            shift 2
            ;;
        -h|--help)
            echo "usage: $0 -s <submap_id>"
            echo "  e.g. $0 -s 2f"
            exit 0
            ;;
        *)
            echo "unknown arg: $1" >&2
            shift
            ;;
    esac
done

if [[ -z "$SUBMAP_ID" ]]; then
    echo "error: -s <submap_id> 必填，例如：$0 -s 2f" >&2
    exit 1
fi

OUT_PARENT="$HOME/vpr_data/$SUBMAP_ID"
OUT_DIR="$OUT_PARENT/rosbag_$(date +%Y%m%d_%H%M%S)"

# ros2 bag record 自己负责创建 OUT_DIR；只需保证父目录存在即可
mkdir -p "$OUT_PARENT"
echo "recording to $OUT_DIR"

ros2 bag record /hdas/camera_head/left_raw/image_raw_color/compressed /Odometry -o "$OUT_DIR"
