FROM osrf/ros:humble-desktop-full
# доп. пакеты: модель Iris, bridge с ROS 2
RUN apt-get update && \
    apt-get install -y ros-humble-ros-ign-gazebo \
                       ros-humble-ros-ign-bridge \
                       ros-humble-ros-ign-gazebo-demos && \
    apt-get clean

# копируем world + launch
WORKDIR /sim
COPY docker/gazebo/iris_world.sdf .
COPY docker/gazebo/launch_iris.sh .

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["bash", "launch_iris.sh"]
