# tools/publish_fake_odom.py
import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.t0  = time.time()
        self.create_timer(0.1, self.tick)     # 10 Гц

    def tick(self):
        t = time.time() - self.t0
        msg = Odometry()
        msg.pose.pose.position.x = 10*math.cos(t/5)
        msg.pose.pose.position.y = 10*math.sin(t/5)
        msg.twist.twist.linear.x = 2.0
        msg.pose.pose.orientation = Quaternion(w=1.0)  # yaw=0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
