FROM ros:humble-ros-base AS builder
RUN apt-get update && apt-get install -y \
      ros-humble-ros-ign-gazebo \
      ros-humble-ros-ign-bridge \
      ignition-gazebo6 \
      ignition-fuel-tools6 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# копируем world + launch
WORKDIR /sim
COPY docker/gazebo/iris_world.sdf .
COPY docker/gazebo/launch_iris.sh .

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["bash", "launch_iris.sh"]
