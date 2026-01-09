#! /bin/bash
source "$(dirname "$0")/common.sh"
# filepath: /home/nvidia/work/Navi/scripts/start_all.sh

SESSION="navi"

# 如果会话已存在则杀掉
tmux kill-session -t HDAS_modules
tmux kill-session -t mobiman
tmux kill-session -t $SESSION 2>/dev/null
