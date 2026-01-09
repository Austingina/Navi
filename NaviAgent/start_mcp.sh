#! /bin/bash

source ~/miniconda3/etc/profile.d/conda.sh
conda activate zzp
script_dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
cd "$script_dir"
python server.py -t
