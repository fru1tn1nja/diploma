import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET_X, TARGET_Y = 10.0, 0.0   # simple waypoint

class AIPlanner(Node):
    def __init__(self):
        super().__init__('ai_planner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.2, self.publish_cmd)
        self.latest_pose = None

    def odom_cb(self, msg):
        self.latest_pose = msg.pose.pose

    def publish_cmd(self):
        if self.latest_pose is None:
            return
        dx = TARGET_X - self.latest_pose.position.x
        dy = TARGET_Y - self.latest_pose.position.y
        dist = math.hypot(dx, dy)
        cmd = Twist()
        if dist > 0.5:
            angle = math.atan2(dy, dx)
            cmd.linear.x = 1.0
            cmd.angular.z = angle
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = AIPlanner()
    rclpy.spin(node)
