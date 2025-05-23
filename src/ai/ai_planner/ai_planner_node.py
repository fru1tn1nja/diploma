#!/usr/bin/env python3
"""ROS 2 Humble node: **ai_planner_node** (v0.2)
-------------------------------------------------
Маршрутный ИИ‑модуль (MVP, одометрия + реактивный обход).

Изменения v0.2
--------------
* подписка на **/odom** (nav_msgs/Odometry) → знаем текущую позицию и yaw
* опция *wp_radius* — радиус «достижения» waypoint
* публикация **/path_planned** (nav_msgs/Path) чтобы видеть маршрут в RViz
* переработан `plan_step`: ориентация берётся из одометрии, скорость плавно затухает при приближении к финишу
"""
from __future__ import annotations

import json
import math
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType

Vec2 = np.ndarray  # alias (2‑D)


# --- в ai_planner_node.py --------------------------------
from std_msgs.msg import UInt8
from enum import IntEnum
class ControlMode(IntEnum):
    AI_CONTROL = 0
    OPERATOR_CONTROL = 1
    MIXED = 2
    FAILSAFE = 3

def dist(a: Vec2, b: Vec2) -> float:
    return float(np.linalg.norm(a - b))


class AIPlanner(Node):
    def __init__(self) -> None:
        super().__init__("ai_planner")

        # -------- параметры -------------
        self.safe_dist = self.declare_and_get("safe_dist", 2.0)           # м
        self.cruise_speed = self.declare_and_get("cruise_speed_mps", 2.5)  # м/с
        self.yaw_kp = self.declare_and_get("yaw_kp", 1.2)
        self.wp_radius = self.declare_and_get("wp_radius", 0.5)            # м

        # -------- состояние -------------
        self._waypoints: List[Vec2] = []
        self._current_wp_idx = 0
        self._laser_ranges: Optional[np.ndarray] = None
        self._pose: Optional[Vec2] = None     # x, y
        self._yaw: float = 0.0

        # -------- pub/sub ---------------
        self.create_subscription(Path, "/mission/waypoints", self.on_path, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_evt = self.create_publisher(String, "/ai/events", 10)
        self._pub_path = self.create_publisher(Path, "/path_planned", 10)



        self._mode = ControlMode.AI_CONTROL
        self.create_subscription(UInt8, "/control/mode", self.on_mode, 10)


        # таймер планирования (10 Гц)
        self.create_timer(0.1, self.plan_step)

        self.get_logger().info("AI Planner started → waiting for mission…")

    # ------------------------------------------------------------
    def declare_and_get(self, name: str, default):
        self.declare_parameter(name, default)
        p = self.get_parameter(name).get_parameter_value()
        if p.type == ParameterType.PARAMETER_DOUBLE:
            return p.double_value
        else:
            return p.integer_value
            
    # ------------------------------------------------------------
    # SUBSCRIBE callbacks
    def on_path(self, msg: Path):
        self._waypoints = [np.array([p.pose.position.x, p.pose.position.y]) for p in msg.poses]
        self._current_wp_idx = 0
        self._pub_path.publish(msg)  # отдадим тот же путь для визуализации
        self.get_logger().info("Mission received: %d waypoints", len(self._waypoints))

    def on_scan(self, msg: LaserScan):
        self._laser_ranges = np.array(msg.ranges)

    def on_odom(self, msg: Odometry):
        self._pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])
        # извлекаем yaw из quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_mode(self, msg: UInt8):
        self._mode = ControlMode(msg.data)

    # ------------------------------------------------------------
    def plan_step(self):
        if self._mode != ControlMode.AI_CONTROL:
            return
        if not self._waypoints:
            return
        if self._pose is None:
            return  # нет одометрии yet

        # --- safety check ---
        if self._laser_ranges is not None and self._laser_ranges.min() < self.safe_dist:
            self.publish_event("obstacle", severity=2, confidence=0.3,
                               detail=f"range={self._laser_ranges.min():.2f} m")
            self._pub_cmd.publish(Twist())
            return

        target = self._waypoints[self._current_wp_idx]
        vec = target - self._pose[:2]
        dist_to_wp = dist(self._pose[:2], target)

        if dist_to_wp < self.wp_radius:
            if self._current_wp_idx < len(self._waypoints) - 1:
                self._current_wp_idx += 1
                return
            else:
                self.publish_event("mission_done", severity=1, confidence=1.0, detail="Reached final WP")
                self._pub_cmd.publish(Twist())
                return

        desired_yaw = math.atan2(vec[1], vec[0])
        yaw_error = desired_yaw - self._yaw
        # нормализуем ошибку до [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # скорость уменьшается к финишу
        speed = self.cruise_speed * min(1.0, dist_to_wp / 5.0)

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(self.yaw_kp * yaw_error)
        self._pub_cmd.publish(cmd)

    # ------------------------------------------------------------
    def publish_event(self, etype: str, *, severity: int, confidence: float, detail: str):
        evt = {"type": etype, "severity": severity, "confidence": confidence, "msg": detail}
        self._pub_evt.publish(String(data=json.dumps(evt)))
        self.get_logger().info("evt %s", evt)


# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AIPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AI Planner shutdown by CTRL-C")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
