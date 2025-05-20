FROM python:3.12-slim
WORKDIR /app
RUN pip install --no-cache-dir paho-mqtt
COPY src/generator/telemetry_generator.py .
ENTRYPOINT ["python", "telemetry_generator.py"]