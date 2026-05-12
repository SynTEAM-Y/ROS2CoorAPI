#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')

        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').value
        if not robot_description:
            raise RuntimeError('robot_description parameter must be set')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(String, '/robot_description', qos)
        self.get_logger().info('Publishing /robot_description as transient local topic')

        self._publish_robot_description()
        self.timer = self.create_timer(1.0, self._publish_robot_description)

    def _publish_robot_description(self):
        msg = String()
        msg.data = self.get_parameter('robot_description').value
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
