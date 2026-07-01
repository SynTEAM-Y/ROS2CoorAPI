#!/usr/bin/env python3
"""
Forward /<ns>/cmd_vel to /model/<ns>/cmd_vel for Gazebo DiffDrive plugin.

The Gazebo DiffDrive plugin subscribes to /model/<ns>/cmd_vel but the
autopilot publishes to /<ns>/cmd_vel. This node bridges the two.

In the multi-robot case, each robot has its own DiffDrive plugin and
its own /model/<ns>/cmd_vel topic. We use the `cmd_topic` and
`gz_cmd_topic` parameters to specify both sides.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('gz_cmd_topic', '/model/x3plus/cmd_vel')
        cmd_topic = self.get_parameter('cmd_topic').value
        gz_cmd_topic = self.get_parameter('gz_cmd_topic').value
        self.sub = self.create_subscription(
            Twist, cmd_topic, self.callback, 10
        )
        self.pub = self.create_publisher(
            Twist, gz_cmd_topic, 10
        )
        self.get_logger().info(
            f'Forwarding {cmd_topic} -> {gz_cmd_topic}')

    def callback(self, msg):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
