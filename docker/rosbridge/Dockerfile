FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rosbridge-server && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
CMD bash -c "source /opt/ros/humble/setup.bash && \
             ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
             port:=9090"