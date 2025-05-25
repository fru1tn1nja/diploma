#!/usr/bin/env python3
"""ROS 2 Humble node: **ai_planner_node** (v0.4)
-------------------------------------------------
Маршрутный ИИ‑модуль (MVP, статические препятствия с записью в БД).
При старте генерирует N случайных круговых препятствий, сохраняет их в Postgres и строит маршрут, обходя их.
"""
from __future__ import annotations

import os
import time
import json
import math
import random
from enum import IntEnum
from typing import List, Tuple, Optional

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import psycopg2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, UInt8
from rcl_interfaces.msg import ParameterType

# alias векторов
Vec2 = np.ndarray

# Простая заглушка для обхода: замените на полноценный алгоритм при необходимости
def simple_rrt(start: Vec2, goal: Vec2, obstacles: List[Tuple[Vec2, float]]) -> List[Vec2]:
    for (o, r) in obstacles:
        v = goal - start
        vv = np.dot(v, v)
        t = np.dot(o - start, v) / vv
        t = float(np.clip(t, 0.0, 1.0))
        p = start + t * v
        if np.linalg.norm(p - o) < r:
            n = np.array([-v[1], v[0]])
            n /= np.linalg.norm(n)
            margin = 1.0
            detour = o + n * (r + margin)
            return [start, detour, goal]
    return [start, goal]

# Режимы управления
class ControlMode(IntEnum):
    AI_CONTROL       = 0
    OPERATOR_CONTROL = 1
    MIXED            = 2
    FAILSAFE         = 3

def dist(a: Vec2, b: Vec2) -> float:
    return float(np.linalg.norm(a - b))

class AIPlanner(Node):
    def __init__(self) -> None:
        super().__init__("ai_planner")

        # Параметры управления
        self.safe_dist         = self.declare_and_get("safe_dist", 2.0)
        self.cruise_speed      = self.declare_and_get("cruise_speed_mps", 2.5)
        self.yaw_kp            = self.declare_and_get("yaw_kp", 1.2)
        self.wp_radius         = self.declare_and_get("wp_radius", 0.5)
        self.max_angular_speed = self.declare_and_get("max_angular_speed", 1.0)
        self.align_threshold   = self.declare_and_get("align_threshold_rad", 0.3)

        # Параметры препятствий и БД
        self.declare_parameter("num_obstacles", 5)
        self.declare_parameter("obstacle_bounds", [-50.0,50.0,-50.0,50.0])
        self.declare_parameter("device_id", 1)

        num_obs = int(self.get_parameter("num_obstacles").get_parameter_value().integer_value)
        bnds    = list(self.get_parameter("obstacle_bounds").get_parameter_value().double_array_value)
        xmin, xmax, ymin, ymax = bnds
        device_id = int(self.get_parameter("device_id").get_parameter_value().integer_value)

        # Генерируем статические препятствия
        self._obstacles: List[Tuple[Vec2,float]] = []
        for _ in range(num_obs):
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            r = random.uniform(3.0, 8.0)
            self._obstacles.append((np.array([x, y]), r))
        self.get_logger().info(f"Static obstacles: {self._obstacles}")

        # Подключение к Postgres и вставка

        # Состояние
        self._waypoints: List[Vec2] = []
        self._current_wp_idx       = 0
        self._pose: Optional[Vec2]   = None
        self._yaw = 0.0
        self._mode = ControlMode.AI_CONTROL

        self._pub_obst = self.create_publisher(String, "/obstacles", 10)
        self._publish_obstacles()
        self.create_timer(1.0, self._publish_obstacles)
        # Pub/Sub
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/mission/waypoints", self.on_path, qos)
        self.create_subscription(Odometry, "/odom",            self.on_odom, 20)
        self.create_subscription(UInt8, "/control/mode",      self.on_mode, 10)

        
        self._pub_cmd  = self.create_publisher(Twist,  "/cmd_vel",     10)
        self._pub_evt  = self.create_publisher(String, "/ai/events",    10)
        self._pub_path = self.create_publisher(Path,   "/path_planned",10)


        

        self.create_timer(0.1, self.plan_step)

        self.get_logger().info("AI Planner started → waiting for mission…")

    def declare_and_get(self, name: str, default):
        self.declare_parameter(name, default)
        p = self.get_parameter(name).get_parameter_value()
        if p.type == ParameterType.PARAMETER_DOUBLE:
            return p.double_value
        if p.type == ParameterType.PARAMETER_INTEGER:
            return p.integer_value
        return list(p.double_array_value)

    def on_path(self, msg: Path):
        n = len(msg.poses)
        self.get_logger().info(f"on_path(): received {n} waypoints")
        self._waypoints = [np.array([p.pose.position.x, p.pose.position.y]) for p in msg.poses]
        self._current_wp_idx = 0
        self._pub_path.publish(msg)
        self.replan_with_avoidance()

    def on_odom(self, msg: Odometry):
        self._pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self._yaw = math.atan2(siny, cosy)

    def on_mode(self, msg: UInt8):
        self._mode = ControlMode(msg.data)

    def replan_with_avoidance(self):
        start = self._pose.copy()
        goal  = self._waypoints[-1]
        self.get_logger().info("Replanning around obstacles...")
        new_path = simple_rrt(start, goal, self._obstacles)
        self._waypoints = new_path
        self._current_wp_idx = 0
        ros_path = Path(); ros_path.header.frame_id='map'
        for p in new_path:
            ps = PoseStamped(); ps.header.frame_id='map'
            ps.pose.position.x, ps.pose.position.y = p.tolist()
            ros_path.poses.append(ps)
        self._pub_path.publish(ros_path)

    def plan_step(self):
        if self._mode != ControlMode.AI_CONTROL or not self._waypoints or self._pose is None:
            return
        target = self._waypoints[self._current_wp_idx]
        vec    = target - self._pose
        dist_wp= dist(self._pose, target)
        if dist_wp < self.wp_radius:
            if self._current_wp_idx < len(self._waypoints)-1:
                self._current_wp_idx += 1
            else:
                self._pub_cmd.publish(Twist())
            return
        desired = math.atan2(vec[1], vec[0])
        err     = (desired - self._yaw + math.pi) % (2*math.pi) - math.pi
        speed   = self.cruise_speed * min(1.0, dist_wp/5.0)
        lin_spd = 0.0 if abs(err)>self.align_threshold else speed
        ang_spd = max(-self.max_angular_speed, min(self.max_angular_speed, self.yaw_kp*err))
        cmd     = Twist(); cmd.linear.x=float(lin_spd); cmd.angular.z=float(ang_spd)
        self._pub_cmd.publish(cmd)

    def publish_event(self, etype: str, **kw):
        evt = {"type":etype, **kw}
        self._pub_evt.publish(String(data=json.dumps(evt)))
        self.get_logger().info(f"evt {evt}")

    def _publish_obstacles(self):
        arr = [[float(o[0][0]), float(o[0][1]), float(o[1])]
               for o in self._obstacles]
        msg = String(data=json.dumps({"obstacles": arr}))
        self._pub_obst.publish(msg)
        self.get_logger().debug(f"Published obstacles: {arr}")

def main(args=None):
    rclpy.init(args=args)
    node = AIPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
