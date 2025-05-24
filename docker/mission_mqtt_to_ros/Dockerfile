FROM ros:humble AS builder
RUN apt-get update && apt-get install -y python3-pip && \
    pip install -U colcon-common-extensions "asyncio-mqtt==0.16.2" "paho-mqtt<2.0"
WORKDIR /ws
COPY src/mission_to_ros/ ./src/mission_mqtt_to_ros/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select mission_mqtt_to_ros --install-base /opt/install

FROM ros:humble AS runtime
ENV PYTHONUNBUFFERED=1
COPY --from=builder /opt/install /opt/install
COPY --from=builder /usr/local /usr/local
WORKDIR /opt/install
CMD bash -c "source /opt/ros/humble/setup.bash && \
             source setup.bash && \
             ros2 run mission_mqtt_to_ros mission_mqtt_to_ros_node"
