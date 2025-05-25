FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip && \
    pip install --no-cache-dir paho-mqtt

WORKDIR /app
COPY src/backend/shared_control_switch/shared_control_switch/switch_logic_node.py .

ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/humble/setup.bash && python3 /app/switch_logic_node.py"]