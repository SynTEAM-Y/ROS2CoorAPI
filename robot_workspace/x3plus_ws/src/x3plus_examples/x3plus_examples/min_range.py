#!/usr/bin/env python3
"""
min_range — Prints the minimum range value from a LaserScan topic.

Useful for quickly checking that lidar data is arriving and identifying
the closest obstacle.

Usage:
    ros2 run x3plus_examples min_range
    ros2 run x3plus_examples min_range --topic /scan --count 20
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MinRange(Node):
    def __init__(self, topic: str, count: int):
        super().__init__('min_range')
        self._count = count
        self._received = 0
        self.sub = self.create_subscription(
            LaserScan, topic, self._cb, 10)
        self.get_logger().info(
            f'Listening on {topic} '
            f'({"unlimited" if count == 0 else count} messages) …'
        )

    def _cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges
                 if msg.range_min <= r <= msg.range_max]
        min_r = min(valid) if valid else float('inf')
        self._received += 1
        self.get_logger().info(
            f'[{self._received:4d}]  min_range = {min_r:.3f} m '
            f'(from {len(valid)} valid of {len(msg.ranges)} beams)'
        )
        if self._count and self._received >= self._count:
            raise SystemExit


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--topic', default='/scan')
    parser.add_argument('--count', type=int, default=0)
    known, remaining = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining)
    node = MinRange(known.topic, known.count)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
