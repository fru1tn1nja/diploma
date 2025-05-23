FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip && \
    pip install --no-cache-dir paho-mqtt



COPY src/mode/requirements.txt .
RUN pip install --user --no-cache-dir -r requirements.txt


WORKDIR /app
COPY src/mode/mode_db_writer.py .

ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/humble/setup.bash && python3 /app/mode_db_writer.py"]
