#!/usr/bin/env python3
"""
Генератор: MQTT + ROS /odom одновременно.

env: DEVICE_ID, MQTT_HOST, MQTT_PORT, FREQ_HZ, RADIUS_METERS
"""
from __future__ import annotations
import os, json, time, math
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# ---------- параметры -----------------------------------------------------
did   = int(os.getenv("DEVICE_ID", 1))
host  = os.getenv("MQTT_HOST", "emqx")
port  = int(os.getenv("MQTT_PORT", 1883))
hz    = float(os.getenv("FREQ_HZ", 2))
R     = float(os.getenv("RADIUS_METERS", 100))
topic = f"telemetry/{did}"

# ---------- ROS-узел ------------------------------------------------------
class GenNode(Node):
    def __init__(self):
        super().__init__("fake_gen")
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.t0 = time.time()
        # MQTT
        self.mqtt = mqtt.Client()
        self.mqtt.connect(host, port, keepalive=30)
        # таймер
        self.create_timer(1/hz, self.tick)

    def tick(self):
        t = time.time() - self.t0
        lat = 60.0 + 0.001*math.cos(t/(10/R))
        lon = 30.0 + 0.001*math.sin(t/(10/R))
        vel = 2.0
        # MQTT
        self.mqtt.publish(topic, json.dumps({
            "device_id": did, "ts": int(time.time()*1000),
            "lat": lat, "lon": lon, "alt": 0.0,
            "vel": vel, "battery": max(0, 100-int(t/6))
        }), qos=0)
        # ROS Odometry (x↔lat, y↔lon)
        odom = Odometry()
        odom.pose.pose.position.x = lat
        odom.pose.pose.position.y = lon
        odom.pose.pose.orientation = Quaternion(w=1.0)   # yaw=0
        odom.twist.twist.linear.x = vel
        self.pub_odom.publish(odom)

def main():
    rclpy.init()
    node = GenNode()
    node.get_logger().info(f"Gen → MQTT {host}:{port} & /odom @ {hz} Hz")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
