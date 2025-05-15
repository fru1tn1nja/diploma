FROM ros:humble AS builder
RUN apt-get update && apt-get install -y python3-pip && \
    pip install -U colcon-common-extensions "asyncio-mqtt==0.16.2" "paho-mqtt<2.0"
WORKDIR /ws
COPY src/bridge/ ./src/ros_mqtt_bridge/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ros_mqtt_bridge --install-base /opt/install

FROM ros:humble AS runtime
ENV PYTHONUNBUFFERED=1
COPY --from=builder /opt/install /opt/install
COPY --from=builder /usr/local /usr/local
WORKDIR /opt/install
CMD bash -c "source /opt/ros/humble/setup.bash && \
             source setup.bash && \
             ros2 run ros_mqtt_bridge ros_mqtt_bridge_node"
