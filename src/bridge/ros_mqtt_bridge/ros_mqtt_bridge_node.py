#!/usr/bin/env python3
"""
ROS 2 → MQTT bridge (sync, paho-mqtt 1.x).

 * Подписывается на одометрию (по-умолчанию PX4 SITL «/fmu/vehicle_odometry»)
   и, при наличии, на /battery.
 * Формирует компактный JSON и публикует в topic telemetry/{DEVICE_ID}.

env:
  DEVICE_ID       – идентификатор (default 1)
  ODOM_TOPIC      – топик одометрии (default '/fmu/vehicle_odometry')
  BATTERY_TOPIC   – топик батареи  (default '/battery')   [опц.]
  MQTT_HOST       – emqx / 127.0.0.1  …
  MQTT_PORT       – 1883
"""

from __future__ import annotations
import os, json, time
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("ros_mqtt_bridge")

        # ---- параметры из окружения --------------------------------------
        self.device_id  = int(os.getenv("DEVICE_ID", "1"))
        mqtt_host       = os.getenv("MQTT_HOST", "emqx")
        mqtt_port       = int(os.getenv("MQTT_PORT", "1883"))
        odom_topic      = os.getenv("ODOM_TOPIC", "/fmu/vehicle_odometry")
        batt_topic      = os.getenv("BATTERY_TOPIC", "/battery")
        self.topic      = f"telemetry/{self.device_id}"

        # ---- MQTT sync client --------------------------------------------
        self.mqtt = mqtt.Client()
        self.mqtt.connect(mqtt_host, mqtt_port, keepalive=30)
        self.get_logger().info(f"Connected to MQTT {mqtt_host}:{mqtt_port}")

        # ---- подписки ROS -------------------------------------------------
        self._latest_batt: int | None = None
        self.create_subscription(Odometry,     odom_topic,  self.on_odom,  20)
        # подписка на батарею необязательна
        self.create_subscription(BatteryState, batt_topic,  self.on_batt,  10)

        self.get_logger().info(
            f"Bridge started: → {self.topic} | odom='{odom_topic}' batt='{batt_topic}'"
        )

    # -------- callbacks ---------------------------------------------------
    def on_batt(self, msg: BatteryState) -> None:
        self._latest_batt = int(msg.percentage * 100)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        payload = {
            "device_id": self.device_id,
            "ts": int(time.time() * 1000),
            "lat": p.x,   # для SITL x≈lat, y≈lon (условное преобразование)
            "lon": p.y,
            "alt": p.z,
            "vel": v.x,
            "battery": self._latest_batt,
        }
        # публикация
        info = self.mqtt.publish(self.topic, json.dumps(payload), qos=0)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().warning(f"MQTT publish error rc={info.rc}")

# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
