#!/usr/bin/env bash
set -e

sudo systemctl stop gdm
sudo /etc/NX/nxserver --restart
