#!/usr/bin/env python3
# mission_mqtt_to_ros.py

import os, json
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class MissionMqttToRos(Node):
    def __init__(self):
        super().__init__('mission_mqtt_to_ros')
        # Читаем DEVICE_ID и MQTT_HOST/PORT из окружения
        dev = os.getenv('DEVICE_ID', '1')
        topic = f'mission/commands/{dev}'
        self.get_logger().info(f'Will bridge MQTT→ROS for topic "{topic}"')

        # ROS-публикатор Path
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(Path, '/mission/waypoints', qos)

        # Настраиваем MQTT-клиент
        self._mqtt = mqtt.Client()
        self._mqtt.on_connect = lambda cl, ud, fl, rc: (
            self.get_logger().info(f'MQTT connected, subscribing to {topic}'),
            cl.subscribe(topic)
        )
        self._mqtt.on_message = self.on_mqtt_message
        self._mqtt.connect(
            '127.0.0.1',
            int(os.getenv('MQTT_PORT', '1883')),
            keepalive=30
        )
        self._mqtt.loop_start()

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            wps = data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Bad mission JSON: {e}')
            return

        path = Path()
        path.header.frame_id = 'map'
        for coord in wps:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(coord[0])
            ps.pose.position.y = float(coord[1])
            path.poses.append(ps)

        self._pub.publish(path)
        self.get_logger().info(f'Published Path with {len(wps)} waypoints')

def main():
    rclpy.init()
    node = MissionMqttToRos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()