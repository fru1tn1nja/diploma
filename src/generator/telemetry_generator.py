#!/usr/bin/env python3
"""
Публикует каждые N миллисекунд пакет JSON в MQTT `telemetry/{DEVICE_ID}`.

env-параметры:
  DEVICE_ID      = 1
  MQTT_HOST      = emqx
  MQTT_PORT      = 1883
  FREQ_HZ        = 2           (частота публикации)
  RADIUS_METERS  = 100         (радиус «круга»)
"""
import os, json, time, math
import paho.mqtt.client as mqtt

# ---- конфигурация -----------------------------------------------
did     = int(os.getenv("DEVICE_ID", 1))
host    = os.getenv("MQTT_HOST", "emqx")
port    = int(os.getenv("MQTT_PORT", 1883))
hz      = float(os.getenv("FREQ_HZ", 2))
radius  = float(os.getenv("RADIUS_METERS", 100))
topic   = f"telemetry/{did}"

# ---- MQTT sync client --------------------------------------------
client = mqtt.Client()
client.connect(host, port, keepalive=30)

print(f"[generator] publishing {hz} Hz to {topic} at {host}:{port}")
t0 = time.time()
while True:
    t = time.time() - t0
    # делаем круг по синус/косинус (≈0.001 ° ≈ 100 м при lat≈60)
    lat = 60.0 + 0.001*math.cos(t/(10/radius))
    lon = 30.0 + 0.001*math.sin(t/(10/radius))
    payload = {
        "device_id": did,
        "ts": int(time.time()*1000),
        "lat": lat,
        "lon": lon,
        "alt": 0.0,
        "vel": 2.0,
        "battery": max(0, 100-int(t/6)),
    }
    client.publish(topic, json.dumps(payload), qos=0)
    time.sleep(1/hz)
