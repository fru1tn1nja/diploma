FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip && \
    pip install --no-cache-dir paho-mqtt

WORKDIR /app
COPY src/bridge/ros_mqtt_bridge/ros_mqtt_bridge_node.py .

ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/humble/setup.bash && python3 /app/ros_mqtt_bridge_node.py"]