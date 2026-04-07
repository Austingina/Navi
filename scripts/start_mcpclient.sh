#! /bin/bash
source "$(dirname "$0")/common.sh"

cd "$NAVI_ROOT_DIR/NaviAgent/test" || exit 1
# 用法示例:
#   ./start_mcpclient.sh
#   ./start_mcpclient.sh -r 线路A
#   ./start_mcpclient.sh --ws ws://127.0.0.1:9999/ws -r 线路A
# -r 由命令行传入；省略则使用服务器返回的第一条路线（见 test_client_fast.py）
exec python test_client_fast.py "$@"
