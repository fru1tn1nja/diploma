import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class Switcher(Node):
    """
    Смешивает команды:
      • /cmd_vel_manual (от оператора)
      • /cmd_vel_auto   (от ИИ)
    Переключение – топик /override Bool:
        True  → MANUAL
        False → AUTO
    Текущий режим публикуется в /current_mode String.
    """
    def __init__(self):
        super().__init__('switcher_node')

        # Pub / Sub
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/current_mode', 10)

        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_cb,   10)
        self.create_subscription(Twist, '/cmd_vel_auto',   self.auto_cb,     10)
        self.create_subscription(Bool,  '/override',       self.override_cb, 10)

        # State
        self.use_auto = True
        self.auto_cmd   = Twist()
        self.manual_cmd = Twist()

        # объявим исходный режим
        self.publish_mode()

    # ---------- callbacks ----------
    def manual_cb(self, msg):
        self.manual_cmd = msg
        if not self.use_auto:
            self.pub_cmd.publish(self.manual_cmd)

    def auto_cb(self, msg):
        self.auto_cmd = msg
        if self.use_auto:
            self.pub_cmd.publish(self.auto_cmd)

    def override_cb(self, msg: Bool):
        # Если пришёл TRUE — оператор просит MANUAL
        self.use_auto = not msg.data
        self.get_logger().info(f"Switched to {'AUTO' if self.use_auto else 'MANUAL'}")
        self.publish_mode()

    # ---------- helper ----------
    def publish_mode(self):
        mode = String()
        mode.data = 'AUTO' if self.use_auto else 'MANUAL'
        self.mode_pub.publish(mode)


def main():
    rclpy.init()
    node = Switcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
