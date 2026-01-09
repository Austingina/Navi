#! /bin/bash

source ~/miniconda3/etc/profile.d/conda.sh
conda activate zzp
cd /home/nvidia/work/Navi/planning/mcp/NaviAgent
python server.py
