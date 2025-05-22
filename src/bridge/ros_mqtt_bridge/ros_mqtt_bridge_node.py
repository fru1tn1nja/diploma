#!/usr/bin/env python3
"""
ROS-2 → MQTT bridge (sync, paho-mqtt 1.x).

Читает /odom и /battery, публикует JSON в telemetry/{device_id}.
"""
from __future__ import annotations
import os, json, time
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

class Bridge(Node):
    def __init__(self):
        super().__init__("ros_mqtt_bridge")

        self.device_id  = int(os.getenv("DEVICE_ID", "1"))
        mqtt_host       = "127.0.0.1"
        mqtt_port       = int(os.getenv("MQTT_PORT", "1883"))
        self.topic      = f"telemetry/{self.device_id}"

        # MQTT sync client
        self.mqtt = mqtt.Client()
        self.mqtt.connect(mqtt_host, mqtt_port, keepalive=30)

        self._latest_batt = None
        self._latest_yaw = 0.0
        self.create_subscription(Odometry,      "/odom",    self.on_odom,  20)
        self.create_subscription(BatteryState,  "/battery", self.on_batt,  10)

        self.get_logger().info(f"Bridge started → publish to {self.topic}")

    # -------- callbacks ----------------------------------------------------
    def on_batt(self, msg: BatteryState):
        self._latest_batt = int(msg.percentage * 100)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        self._latest_yaw = math.atan2(siny_cosp, cosy_cosp)

        payload = {
            "device_id": self.device_id,
            "ts": int(time.time() * 1000),
            "lat": p.x,          # в симуляторе x≈lat, y≈lon
            "lon": p.y,
            "alt": p.z,
            "vel": v.x,
            "battery": self._latest_batt,
            "yaw": self._latest_yaw, 
        }
        self.mqtt.publish(self.topic, json.dumps(payload), qos=0)

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
