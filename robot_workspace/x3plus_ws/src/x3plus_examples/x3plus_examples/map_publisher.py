#!/usr/bin/env python3
"""
Map Publisher Node

Publishes a static map for RViz visualization.
Used in RViz-only mode when Gazebo is not running.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import os


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        # Declare parameter for map path
        self.declare_parameter('map_path', '')
        
        # Create publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Create timer to publish map periodically (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Load map if path provided
        map_path = self.get_parameter('map_path').value
        
        if map_path and os.path.exists(map_path):
            self.load_map(map_path)
        else:
            # Create a simple empty map if no path provided
            self.create_empty_map()
        
        self.get_logger().info('Map Publisher initialized')
    
    def create_empty_map(self):
        """Create a simple empty map for visualization"""
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        
        # 10x10 meter map with 0.05m resolution
        self.map_msg.info.resolution = 0.05
        self.map_msg.info.width = 200
        self.map_msg.info.height = 200
        
        self.map_msg.info.origin.position.x = -5.0
        self.map_msg.info.origin.position.y = -5.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        
        # Initialize with free space (0)
        self.map_msg.data = [0] * (self.map_msg.info.width * self.map_msg.info.height)
        
        self.get_logger().info('Created empty 10x10m map')
    
    def load_map(self, map_path):
        """Load map from YAML file"""
        try:
            with open(map_path, 'r') as f:
                map_data = yaml.safe_load(f)
            
            self.get_logger().info(f'Loaded map from {map_path}')
            # TODO: Implement full map loading from image file
            # For now, just create empty map
            self.create_empty_map()
            
        except Exception as e:
            self.get_logger().warning(f'Failed to load map: {e}')
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
