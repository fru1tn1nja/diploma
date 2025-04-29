import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


class AIPlanner(Node):
    """
    Простой планёр:
      • хранит «текущий целевой waypoint»
      • публикует /cmd_vel_auto (пропорциональный добег)
      • по topic /desired_waypoint geometry_msgs/Point получает новые точки
      • /return_home (std_msgs/Empty) — вернуться в (0,0)
    """

    def __init__(self):
        super().__init__('ai_planner')

        # publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)

        # subscribers
        self.create_subscription(Odometry,         '/odom',             self.odom_cb,     10)
        self.create_subscription(Point,            '/desired_waypoint', self.waypoint_cb, 10)
        self.create_subscription(Empty,            '/return_home',      self.home_cb,     10)

        # state
        self.latest_pose: Optional[Odometry] = None
        self.target: Optional[Point] = None

        # control loop 5 Гц
        self.timer = self.create_timer(0.2, self.control_step)

    # ---------- callbacks ----------
    def odom_cb(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def waypoint_cb(self, msg: Point):
        self.get_logger().info(f"New waypoint: ({msg.x:.1f}, {msg.y:.1f})")
        self.target = msg

    def home_cb(self, _: Empty):
        self.get_logger().info("Return-home requested.")
        self.target = Point(x=0.0, y=0.0, z=0.0)

    # ---------- main loop ----------
    def control_step(self):
        if self.latest_pose is None or self.target is None:
            return

        dx = self.target.x - self.latest_pose.position.x
        dy = self.target.y - self.latest_pose.position.y
        dist = math.hypot(dx, dy)

        cmd = Twist()

        if dist > 0.5:                                  # ещё далеко
            angle_to_goal = math.atan2(dy, dx)
            yaw = self.get_yaw_from_quat(self.latest_pose.orientation)
            heading_error = self.angle_diff(angle_to_goal, yaw)

            # простейший P-контроллер
            cmd.linear.x  = 1.0                         # константная «тяга»
            cmd.angular.z = 1.5 * heading_error
        else:                                           # цель достигнута
            cmd.linear.x = cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ---------- helpers ----------
    @staticmethod
    def get_yaw_from_quat(q):
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_diff(a, b):
        """Вернуть угол a-b в диапазоне [-π, π]."""
        import math
        diff = a - b
        return (diff + math.pi) % (2 * math.pi) - math.pi


def main():
    rclpy.init()
    node = AIPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
