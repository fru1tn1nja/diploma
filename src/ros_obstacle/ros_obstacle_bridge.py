#!/usr/bin/env python3
"""
ROS 2 Humble node: ros_obstacle_bridge_node
-------------------------------------------
Bridge ROS `/obstacles` topic to MQTT `obstacles/{device_id}`.
Listens on std_msgs/String JSON: {"obstacles":[[x,y,r],...]}
Publishes exact payload to MQTT.
"""
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class ObstacleBridge(Node):
    def __init__(self) -> None:
        super().__init__('ros_obstacle_bridge')
        # Get device ID and MQTT parameters from environment
        device_id = int(os.getenv('DEVICE_ID', '1'))
        mqtt_host = '127.0.0.1'
        mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self._topic = f'obstacles/{device_id}'

        # Initialize MQTT client
        self._mqtt = mqtt.Client()
        self._mqtt.connect(mqtt_host, mqtt_port, keepalive=30)
        self._mqtt.loop_start()
        self.get_logger().info(f'Connected to MQTT at {mqtt_host}:{mqtt_port}, publishing to "{self._topic}"')

        # Subscribe to ROS topic
        self.create_subscription(String, '/obstacles', self.on_obstacles, 10)
        self.get_logger().info('Subscribed to ROS topic "/obstacles"')

    def on_obstacles(self, msg: String) -> None:
        """
        Callback for ROS /obstacles topic. msg.data is JSON string.
        Forward it to MQTT unchanged.
        """
        try:
            # Validate JSON
            payload = json.loads(msg.data)
            # Re-serialize to ensure valid formatting
            out = json.dumps(payload)
            self._mqtt.publish(self._topic, out, qos=0)
            self.get_logger().debug(f'Forwarded obstacles to MQTT: {out}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON on /obstacles: {e}')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
