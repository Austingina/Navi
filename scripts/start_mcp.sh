#! /bin/bash
source "$(dirname "$0")/common.sh"

cd $NAVI_ROOT_DIR/NaviAgent

# Check for -t argument
if [[ "$1" == "-t" ]]; then
	python server.py -t -m $NAVI_ROOT_DIR/maps/multifloor/multifloor.json
else
	python server.py -m $NAVI_ROOT_DIR/maps/multifloor/multifloor.json
fi
