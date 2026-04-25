#!/usr/bin/env python3
"""
Map Publisher — Loads a ROS-format map YAML and publishes it as nav_msgs/OccupancyGrid.

Publishes to /map (latched — re-published at 1 Hz so late subscribers receive it).

Map YAML format (standard ROS map_server format):
  image: <path-to-pgm>
  resolution: <meters-per-pixel>
  origin: [x, y, yaw]
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.196

Usage:
    ros2 run x3plus_examples map_publisher
    ros2 run x3plus_examples map_publisher --map-path /path/to/map.yaml

If the map file is absent the node publishes a minimal 10×10 m free (empty) map
so that RViz can still display a valid /map topic.
"""

import os
import sys
import math
import struct
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from builtin_interfaces.msg import Time


# Default map path (matches robot_rviz.launch.py)
DEFAULT_MAP_PATH = os.path.expanduser(
    '~/ROS2Coordination/robot_workspace/x3plus_ws/maps/plain_map.yaml'
)


def _load_pgm(path: str):
    """Load a binary or ASCII PGM file and return (width, height, pixels)."""
    with open(path, 'rb') as f:
        # Read magic
        magic = f.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise ValueError(f'Unsupported PGM magic: {magic}')
        # Skip comments
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = map(int, line.split())
        max_val = int(f.readline().strip())
        if magic == b'P5':
            raw = f.read(width * height)
            pixels = list(raw)
        else:
            pixels = list(map(int, f.read().split()))
    return width, height, pixels, max_val


def _load_yaml_map(yaml_path: str):
    """Parse a minimal ROS map YAML and return an OccupancyGrid."""
    import yaml  # available in standard Python 3
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    resolution = float(data.get('resolution', 0.05))
    origin = data.get('origin', [0.0, 0.0, 0.0])
    negate = int(data.get('negate', 0))
    occ_thresh = float(data.get('occupied_thresh', 0.65))
    free_thresh = float(data.get('free_thresh', 0.196))
    image_file = data.get('image', '')

    # Resolve image path relative to yaml
    if not os.path.isabs(image_file):
        image_file = os.path.join(os.path.dirname(yaml_path), image_file)

    width, height, pixels, max_val = _load_pgm(image_file)

    grid_data = []
    for p in pixels:
        value = p / max_val
        if negate:
            value = 1.0 - value
        if value >= occ_thresh:
            grid_data.append(100)   # occupied
        elif value <= free_thresh:
            grid_data.append(0)     # free
        else:
            grid_data.append(-1)    # unknown

    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = 0.0
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    msg.info.origin.orientation.z = math.sin(yaw / 2.0)
    msg.info.origin.orientation.w = math.cos(yaw / 2.0)
    msg.data = grid_data
    return msg


def _empty_map(size_m: float = 10.0, resolution: float = 0.05) -> OccupancyGrid:
    """Return a free (all-zero) square map centred at origin."""
    cells = int(size_m / resolution)
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = cells
    msg.info.height = cells
    msg.info.origin.position.x = -size_m / 2.0
    msg.info.origin.position.y = -size_m / 2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * (cells * cells)
    return msg


class MapPublisher(Node):
    """Publishes a static map to /map at 1 Hz (latched-equivalent)."""

    def __init__(self, map_path: str):
        super().__init__('map_publisher')

        # Use transient_local so late subscribers receive the last message
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)

        self.grid = None
        if map_path and os.path.isfile(map_path):
            try:
                self.grid = _load_yaml_map(map_path)
                self.get_logger().info(
                    f'Map loaded: {map_path}  '
                    f'({self.grid.info.width}×{self.grid.info.height} cells, '
                    f'{self.grid.info.resolution} m/cell)'
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Failed to load map from {map_path}: {e} — using empty map'
                )
        else:
            if map_path:
                self.get_logger().warn(
                    f'Map file not found: {map_path} — using empty map'
                )
            else:
                self.get_logger().info('No map path given — publishing empty map')

        if self.grid is None:
            self.grid = _empty_map()

        # Publish immediately, then at 1 Hz for late subscribers
        self._publish()
        self.timer = self.create_timer(1.0, self._publish)

    def _publish(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid.header.frame_id = 'map'
        self.pub.publish(self.grid)


def main(args=None):
    # Parse --map-path before handing off to rclpy
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--map-path', default=DEFAULT_MAP_PATH)
    known, remaining = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining)
    node = MapPublisher(known.map_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
