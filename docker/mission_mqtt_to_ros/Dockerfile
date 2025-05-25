FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip && \
    pip install --no-cache-dir paho-mqtt

WORKDIR /app
COPY src/mission_to_ros/mission_mqtt_to_ros/mission_mqtt_to_ros_node.py .

ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/humble/setup.bash && python3 /app/mission_mqtt_to_ros_node.py"]
