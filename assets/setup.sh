#!/bin/bash
set -e

echo "setup finish"
source /opt/ros/humble/setup.bash
source $HOME/.bashrc
exec "$@"

