#! /bin/bash
source "$(dirname "$0")/common.sh"

cd $NAVI_ROOT_DIR/NaviAgent/test
# 允许传入url参数，默认值为http://localhost:9999/mcp
URL=${1:-http://localhost:9999/mcp}
echo "连接 MCP Server: $URL"
python test_client_ui.py --url "$URL"
