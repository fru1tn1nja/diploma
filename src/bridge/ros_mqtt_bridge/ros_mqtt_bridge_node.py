#!/usr/bin/env python3
"""
ROS-2 → MQTT bridge.
Берёт /odom (nav_msgs/Odometry) и /battery (sensor_msgs/BatteryState),
упаковывает в компактный JSON и публикует telemetry/{device_id} в MQTT.

env:
  DEVICE_ID     – числовой ID (default 1)
  MQTT_HOST     – брокер (default 'emqx')
  MQTT_PORT     – 1883
"""
from __future__ import annotations
import os, json, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from asyncio_mqtt import Client
import asyncio

class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("ros_mqtt_bridge")

        self.device_id = int(os.getenv("DEVICE_ID", "1"))
        self.mqtt_host = os.getenv("MQTT_HOST", "emqx")
        self.mqtt_port = int(os.getenv("MQTT_PORT", "1883"))
        self.topic = f"telemetry/{self.device_id}"

        # MQTT client
        self.mqtt = Client(self.mqtt_host, self.mqtt_port)

        # подключаемся асинхронно: создаём задачу в текущем event-loop ROS
        asyncio.get_running_loop().create_task(self.mqtt.connect())

        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(BatteryState, "/battery", self.on_batt, 10)

        self.get_logger().info(f"Bridge started → publish to {self.topic}")

    def on_batt(self, msg: BatteryState):
        self._latest_batt = int(msg.percentage * 100)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        payload = {
            "device_id": self.device_id,
            "ts": int(time.time() * 1000),
            "lat": p.x,                 # для симулятора считаем x≈lat, y≈lon
            "lon": p.y,
            "alt": p.z,
            "vel": v.x,
            "battery": self._latest_batt,
        }
        self.mqtt.publish(self.topic, json.dumps(payload), qos=0)

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