#!/usr/bin/env python3
"""ROS → MQTT bridge for mission waypoints."""
from __future__ import annotations
import os, json, rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
import paho.mqtt.client as mqtt

class MissionBridge(Node):
    def __init__(self):
        super().__init__("ros_mission_bridge")

        self.device_id  = int(os.getenv("DEVICE_ID", "1"))
        self.topic      = f"mission/waypoints/{self.device_id}"
        mqtt_host       = "127.0.0.1"
        mqtt_port       = int(os.getenv("MQTT_PORT", "1883"))

        self.mqtt = mqtt.Client()
        self.mqtt.connect(mqtt_host, mqtt_port, keepalive=30)
        self.mqtt.loop_start()

        self.create_subscription(Path, "/mission/waypoints", self.on_path, 10)
        self.get_logger().info(f"mission-bridge → {self.topic}")

    def on_path(self, msg: Path):
        wps = [[p.pose.position.x, p.pose.position.y]
               for p in msg.poses]
        goal = wps[-1] if wps else None
        payload = {"waypoints": wps, "goal": goal}
        self.mqtt.publish(self.topic, json.dumps(payload), qos=0)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MissionBridge())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
