#! /bin/bash
source "$(dirname "$0")/common.sh"

# 退出所有 Navi 相关 tmux 会话：navi（雷达+NaviSkill 常驻）、slam、outdoor。
for SESSION in navi slam outdoor; do
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "killed session: $SESSION"
done
