#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ -f /workspace/project/catkin_ws/devel/setup.bash ]; then
  source /workspace/project/catkin_ws/devel/setup.bash
fi

exec "$@"
