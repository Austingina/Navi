#! /bin/bash

cd ~/galaxea/install/startup_config/share/startup_config/script/
./robot_startup.sh boot ../sessions.d/ATCNavigation/R1PROVRTeleopNAV.d/

/usr/bin/python /home/nvidia/GalaxeaDataset/lxy/cmd_odom.py
