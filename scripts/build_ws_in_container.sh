#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
cd /workspace/project/catkin_ws

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi

rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true

rm -rf build devel install
catkin_make
