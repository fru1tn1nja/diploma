FROM ros:humble AS builder

RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install -U colcon-common-extensions

WORKDIR /ws

# копируем только ROS‑пакет, чтобы кешировать слои
COPY src/backend/shared_control_switch/ ./src/shared_control_switch/

# сборка с установкой в /opt/install (легковесный runtime)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select shared_control_switch \
                    --install-base /opt/install

# ---------- Runtime stage ----------
FROM ros:humble AS runtime

# переменные среды из .env будут проброшены через compose
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# copy built install tree
COPY --from=builder /opt/install /opt/install

WORKDIR /opt/install

CMD ["bash", "-c", \
        "source /opt/ros/humble/setup.bash && \
        source setup.bash && \
        ros2 run shared_control_switch switch_logic"]
    