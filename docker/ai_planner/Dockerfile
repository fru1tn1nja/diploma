FROM ros:humble AS builder

# инструменты сборки + python-deps
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install -U colcon-common-extensions numpy

WORKDIR /ws

# копируем только пакет ai_planner
# (структура: src/ai/ai_planner/{package.xml, setup.py, ...})
COPY src/ai/ ./src/ai_planner/

# собираем и ставим в /opt/install (чистый runtime-директории)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ai_planner \
                    --install-base /opt/install

# ---------- Runtime stage ----------
FROM ros:humble AS runtime
ENV PYTHONUNBUFFERED=1
WORKDIR /opt/install

# переносим собранный пакет и все python-зависимости
COPY --from=builder /opt/install /opt/install
COPY --from=builder /usr/local /usr/local

# запускаем узел
CMD ["bash", "-c", \
        "source /opt/ros/humble/setup.bash && \
        source setup.bash && \
        ros2 run ai_planner ai_planner_node"]
    