#! /bin/bash

SESSION="compare"
# 新建会话并在第一个面板运行 r1pro
tmux new-session -d -s $SESSION "ros2 topic echo /tf"

# 水平分割面板并运行 Lidar
tmux split-window -h "python print_timestamp.py"

# 选择第一个面板并附加到会话
tmux select-pane -t 0
tmux attach-session -t $SESSION