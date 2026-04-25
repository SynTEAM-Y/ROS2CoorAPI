#!/usr/bin/env python3
"""
cmd_vel_test — Subscribes to /cmd_vel for a fixed duration and prints each message.

Useful for quickly verifying that a control node is publishing velocity commands.

Usage:
    ros2 run x3plus_examples cmd_vel_test
    ros2 run x3plus_examples cmd_vel_test --duration 10
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelTest(Node):
    def __init__(self, duration: float):
        super().__init__('cmd_vel_test')
        self._count = 0
        self.sub = self.create_subscription(Twist, 'cmd_vel', self._cb, 10)
        self.timer = self.create_timer(duration, self._stop)
        self.get_logger().info(
            f'Listening on /cmd_vel for {duration:.0f} s … (Ctrl-C to stop early)'
        )

    def _cb(self, msg: Twist):
        self._count += 1
        self.get_logger().info(
            f'[{self._count:4d}]  linear.x={msg.linear.x:+.3f}  '
            f'angular.z={msg.angular.z:+.3f}'
        )

    def _stop(self):
        self.get_logger().info(
            f'Done — received {self._count} message(s).'
        )
        raise SystemExit


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--duration', type=float, default=5.0)
    known, remaining = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining)
    node = CmdVelTest(known.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
