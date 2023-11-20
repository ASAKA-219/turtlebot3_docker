#!/bin/bash
set -e

echo "setup finish"
source /opt/ros/noetic/setup.bash
source ~/.bashrc
exec "$@"
