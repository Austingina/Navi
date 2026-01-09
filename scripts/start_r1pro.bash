#! /bin/bash
source "$(dirname "$0")/common.sh"

cd ~/galaxea/install/startup_config/share/startup_config/script/
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

/usr/bin/python $NAVI_SCRIPT_DIR/cmd_odom.py
