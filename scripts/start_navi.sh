#! /bin/bash
source "$(dirname "$0")/common.sh"
# filepath: /home/nvidia/work/Navi/scripts/start_all.sh
bash "$NAVI_SCRIPT_DIR/stop_navi.sh"


tmux kill-server
SESSION="navi"
if [[ "$1" == "-c" ]]; then
    # 使用 tmuxp 加载配置
    tmuxp load -d "$NAVI_SCRIPT_DIR/navi.caihong.yaml"
    echo "caihong" > $NAVI_ROOT_DIR/NaviSkill/config/mapuse.txt
else
    tmuxp load -d "$NAVI_SCRIPT_DIR/navi.yaml"
    echo "caihong" >$NAVI_ROOT_DIR/NaviSkill/config/mapuse.txt
fi