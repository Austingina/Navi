#! /bin/bash
#! /bin/bash
# filepath: /home/nvidia/work/Navi/scripts/start_all.sh

bash ~/work/Navi/scripts/stop_navi.sh
SESSION="navi"
if [[ "$1" == "-c" ]]; then
    # 使用 tmuxp 加载配置
    tmuxp load -d ~/work/Navi/scripts/navi.caihong.yaml
    echo "caihong" > ~/work/Navi/NaviAgent/config/mapuse.txt
else
    tmuxp load -d ~/work/Navi/scripts/navi.yaml
    echo "525" > ~/work/Navi/NaviAgent/config/mapuse.txt
fi