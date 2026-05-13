#! /bin/bash
source "$(dirname "$0")/common.sh"
# filepath: /home/nvidia/work/Navi/scripts/start_all.sh

bash "$NAVI_SCRIPT_DIR/stop_navi.sh"
SESSION="navi"

# 新建会话并在第一个面板运行 r1pro
tmux new-session -d -s $SESSION "bash $NAVI_SCRIPT_DIR/start_r1pro.bash; exec bash"

# 水平分割面板并运行 Lidar
tmux split-window -h "bash $NAVI_SCRIPT_DIR/start_navigation.bash; exec bash"

# 在左侧面板垂直分割并运行 Localization
tmux select-pane -t 0
tmux split-window -v "bash $NAVI_SCRIPT_DIR/start_lidar.bash; exec bash"

# 在右侧面板垂直分割并运行 Navigation
tmux select-pane -t 1
tmux split-window -v "bash $NAVI_SCRIPT_DIR/start_localization.bash; exec bash"

# 选择第一个面板并附加到会话
tmux select-pane -t 0
tmux attach-session -t $SESSION