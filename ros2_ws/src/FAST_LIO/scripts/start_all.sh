#! /bin/bash

# 开发环境
# gnome-terminal --title="r1pro" -- bash -c "~/work/Navi/scripts/start_r1pro.bash; exec bash" &
# gnome-terminal --title="Lidar" -- bash -c "~/work/Navi/scripts/start_lidar.bash; exec bash" &
# gnome-terminal --title="Localization" -- bash -c "~/work/Navi/scripts/start_localization.bash; exec bash" &
# gnome-terminal --title="Navigation" -- bash -c "~/work/Navi/scripts/start_navigation.launch; exec bash" &
# gnome-terminal --title="MCP Server" -- bash -c "~/work/Navi/scripts/start_mcp.sh; exec bash" &




# 开发环境

gnome-terminal --tab --title="Lidar" -- bash -ic "~/work/Navi/scripts/start_lidar.bash; exec bash" &
sleep 5
gnome-terminal --tab --title="Localization" -- bash -ic "~/work/Navi/scripts/start_localization.bash; exec bash" &
sleep 5
gnome-terminal --tab --title="Navigation" -- bash -ic "~/work/Navi/scripts/start_navigation.launch; exec bash" &
sleep 5
gnome-terminal --tab --title="MCP Server" -- bash -ic "~/work/Navi/scripts/start_mcp.sh; exec bash" &
sleep 5
gnome-terminal --tab --title="r1pro" -- bash -ic "~/work/Navi/scripts/start_r1pro.bash; exec bash" &
