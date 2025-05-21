#!/usr/bin/env python3
"""
Мини-симулятор ASV.
  • вход:  /cmd_vel    (geometry_msgs/Twist)
  • выход: /odom       (nav_msgs/Odometry)
           /battery    (sensor_msgs/BatteryState)

Модель: простая кинематика на плоскости, dt = 0.1 с.
Батарея линейно падает от 100 % до 0 % за 20 мин.
"""
from __future__ import annotations
import math, time, os, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

DT = 0.1                        # шаг интегрирования (с)
BATTERY_SEC = 1200              # время разряда (20 мин)

class ASVSim(Node):
    def __init__(self):
        super().__init__("asv_sim")
        self.x = self.y = self.yaw = 0.0
        self.v = self.omega = 0.0
        self.start_time = time.time()

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_batt = self.create_publisher(BatteryState, '/battery', 5)

        self.create_timer(DT, self.step)
        self.get_logger().info("ASV simulator started")

    def on_cmd(self, msg: Twist):
        self.v      = msg.linear.x
        self.omega  = msg.angular.z

    def step(self):
        # интегрируем
        self.x   += self.v * math.cos(self.yaw) * DT
        self.y   += self.v * math.sin(self.yaw) * DT
        self.yaw += self.omega * DT

        # публикуем /odom
        odom = Odometry()
        odom.header.frame_id          = 'map'
        odom.header.stamp             = self.get_clock().now().to_msg()
        odom.pose.pose.position.x     = self.x
        odom.pose.pose.position.y     = self.y
        odom.pose.pose.orientation    = Quaternion(
            z = math.sin(self.yaw/2), w = math.cos(self.yaw/2))
        odom.twist.twist.linear.x     = self.v
        odom.twist.twist.angular.z    = self.omega
        self.pub_odom.publish(odom)

        # батарея
        batt = BatteryState()
        batt.percentage = max(0.0,
            1.0 - (time.time()-self.start_time)/BATTERY_SEC)
        self.pub_batt.publish(batt)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ASVSim())
    rclpy.shutdown()

if __name__ == '__main__':
    main()