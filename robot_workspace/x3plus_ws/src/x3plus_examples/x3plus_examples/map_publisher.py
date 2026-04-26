#!/usr/bin/env python3
"""
Map Publisher Node

Publishes a static map for RViz visualization.
Used in RViz-only mode when Gazebo is not running.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Declare parameter for map path (a nav2-style .yaml referencing a .pgm)
        self.declare_parameter('map_path', '')

        # Use TRANSIENT_LOCAL + RELIABLE QoS — this is what RViz's Map display
        # and nav2 expect for /map. Without it RViz logs:
        #   'incompatible QoS ... DURABILITY_QOS_POLICY'
        # and never shows the map.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        map_path = self.get_parameter('map_path').value
        if map_path and os.path.exists(map_path):
            self.load_map(map_path)
        else:
            if map_path:
                self.get_logger().warning(
                    f"map_path '{map_path}' does not exist, falling back to empty 10x10 m map")
            self.create_empty_map()

        # Latched publish (TRANSIENT_LOCAL) means a single publish is enough,
        # but we re-publish at 1 Hz so a late-joining subscriber that uses
        # VOLATILE QoS still sees the map.
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Map Publisher initialized')
        # Publish once immediately so the latched subscriber gets it without
        # waiting up to a second for the timer.
        self.publish_map()

    def create_empty_map(self):
        """Create a simple empty map for visualization"""
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = 0.05
        self.map_msg.info.width = 200
        self.map_msg.info.height = 200
        self.map_msg.info.origin.position.x = -5.0
        self.map_msg.info.origin.position.y = -5.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = [0] * (self.map_msg.info.width * self.map_msg.info.height)
        self.get_logger().info('Created empty 10x10m map')

    def load_map(self, map_path):
        """Load a nav2-style map: yaml metadata + .pgm occupancy image."""
        try:
            with open(map_path, 'r') as f:
                meta = yaml.safe_load(f)

            image_field = meta.get('image')
            if not image_field:
                raise ValueError("yaml is missing required 'image' field")
            image_path = image_field
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(map_path), image_path)
            if not os.path.exists(image_path):
                raise FileNotFoundError(f"map image '{image_path}' not found")

            resolution = float(meta.get('resolution', 0.05))
            origin = meta.get('origin', [0.0, 0.0, 0.0])
            negate = int(meta.get('negate', 0))
            occupied_thresh = float(meta.get('occupied_thresh', 0.65))
            free_thresh = float(meta.get('free_thresh', 0.196))

            # Lazy import so the node still starts on a system without numpy/PIL
            # — it would just fall back to the empty map below.
            from PIL import Image
            import numpy as np

            img = Image.open(image_path)
            # Convert to greyscale; flip vertically because image origin is
            # top-left while ROS map origin is bottom-left.
            img = img.convert('L').transpose(Image.FLIP_TOP_BOTTOM)
            arr = np.array(img, dtype=np.uint8)
            height, width = arr.shape

            # Standard nav2 conversion: p = (255 - pixel) / 255 (or pixel/255 if negate)
            if negate:
                p = arr.astype(np.float32) / 255.0
            else:
                p = (255 - arr.astype(np.float32)) / 255.0

            data = np.full(arr.shape, -1, dtype=np.int8)  # unknown
            data[p > occupied_thresh] = 100               # occupied
            data[p < free_thresh] = 0                     # free

            self.map_msg = OccupancyGrid()
            self.map_msg.header.frame_id = 'map'
            self.map_msg.info.resolution = resolution
            self.map_msg.info.width = width
            self.map_msg.info.height = height
            self.map_msg.info.origin.position.x = float(origin[0])
            self.map_msg.info.origin.position.y = float(origin[1])
            self.map_msg.info.origin.position.z = 0.0
            self.map_msg.info.origin.orientation.w = 1.0
            self.map_msg.data = data.flatten().tolist()

            self.get_logger().info(
                f'Loaded map from {map_path} '
                f'({width}x{height} cells @ {resolution} m/cell, '
                f'origin=({origin[0]:.2f}, {origin[1]:.2f}))')
        except Exception as e:
            self.get_logger().warning(f'Failed to load map ({e}); using empty map')
            self.create_empty_map()

    def publish_map(self):
        """Publish the map"""
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
