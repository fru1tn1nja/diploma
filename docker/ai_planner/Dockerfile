FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip && \
    pip install --no-cache-dir paho-mqtt

WORKDIR /app
COPY src/ai/ai_planner/ai_planner_node.py .

ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/humble/setup.bash && python3 /app/ai_planner_node.py"]
