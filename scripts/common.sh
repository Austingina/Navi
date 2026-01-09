#!/bin/bash
# 定义脚本目录常量
export NAVI_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAVI_ROOT_DIR="$(dirname "$NAVI_SCRIPT_DIR")"
echo $NAVI_ROOT_DIR
echo $NAVI_SCRIPT_DIR
source $NAVI_ROOT_DIR/ros2_ws/install/setup.bash