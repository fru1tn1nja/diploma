# ──────────────────────────────────────────────────────────────
# ROS 2 Humble + Gazebo Classic (11) + TurtleBot3 + bridge utils
# ──────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop-full AS base

# 1. system deps
RUN apt-get update && apt-get install -y \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-turtlebot3-gazebo \
        python3-colcon-common-extensions \
        && rm -rf /var/lib/apt/lists/*

# 2. environment for TB3
ENV TURTLEBOT3_MODEL=burger \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. copy launch assets
WORKDIR /sim_ws
COPY launch/ launch/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]