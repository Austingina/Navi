#! /bin/bash


tmux kill-session -t HDAS_modules
tmux kill-session -t mobiman

cd ~/galaxea/atc_standard-V2.1.3-20250807_15_22_50_aarch64/install/startup_config/share/startup_config/script/
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/
