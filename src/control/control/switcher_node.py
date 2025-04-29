import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Switcher(Node):
    def __init__(self):
        super().__init__('switcher_node')
        self.use_auto = True
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.auto_cb, 10)
        self.create_subscription(Bool, '/override', self.override_cb, 10)
        self.auto_cmd = Twist()
        self.manual_cmd = Twist()

    def manual_cb(self, msg):
        self.manual_cmd = msg
        if not self.use_auto:
            self.pub.publish(self.manual_cmd)

    def auto_cb(self, msg):
        self.auto_cmd = msg
        if self.use_auto:
            self.pub.publish(self.auto_cmd)

    def override_cb(self, msg):
        self.use_auto = not msg.data
        self.get_logger().info(f"Switched to {'AUTO' if self.use_auto else 'MANUAL'}")

def main():
    rclpy.init()
    node = Switcher()
    rclpy.spin(node)
