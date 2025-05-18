#!/usr/bin/env bash
set -e

# 1. Запускаем Gazebo headless
ign gazebo -r iris_world.sdf &          # -r = headless

# 2. Bridge: Ignition→ROS 2 (одометрия, laser, battery)
ros2 run ros_ign_bridge parameter_bridge \
    /world/iris/model/iris/pose@tf2_msgs/msg/TFMessage@tf2_msgs/msg/TFMessage \
    /world/iris/model/iris/odometry@nav_msgs/msg/Odometry@ros_ign_interfaces/msg/42 \
    /world/iris/model/iris/scan@sensor_msgs/msg/LaserScan@ros_ign_interfaces/msg/23 \
    __ns:=/ &

wait -n