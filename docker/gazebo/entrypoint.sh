#!/usr/bin/env bash
set -e
# Source ROS 2
source /opt/ros/humble/setup.bash
# Allow user override launch file via CMD
if [ -z "$1" ]; then
  ros2 launch launch/turtlebot3.launch.py
else
  exec "$@"
fi