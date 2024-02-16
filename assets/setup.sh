#!/bin/bash
set -e

echo "setup finish"
source /opt/ros/kinetic/setup.bash
source ~/.bashrc
exec "$@"
