#! /bin/bash
source "$(dirname "$0")/common.sh"

cd $NAVI_ROOT_DIR/NaviAgent

# Check for -t argument
if [[ "$1" == "-t" ]]; then
	python server.py -t
else
	python server.py
fi
