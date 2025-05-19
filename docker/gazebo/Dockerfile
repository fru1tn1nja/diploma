FROM ros:humble-ros-base

# Gazebo Sim + мосты + демо-миры
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim-demos \
        ros-humble-ros-gz-bridge && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# пример мира из демо-пакета
ENV GZ_WORLD=/opt/ros/humble/share/ros_gz_sim_demos/worlds/shapes.sdf
# чтоб Gazebo сначала искал ресурсы локально, а не на Fuel
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/humble/share

ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/humble/setup.bash && \
     gz sim -r ${GZ_WORLD}"]
