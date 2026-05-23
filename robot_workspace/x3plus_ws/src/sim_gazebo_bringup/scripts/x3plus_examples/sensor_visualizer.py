#!/usr/bin/env python3
"""Sensor Visualizer — show camera and lidar data in OpenCV windows.

Ignition Gazebo (Fortress) does not automatically pop up image windows for
camera sensors.  This node subscribes to the bridged ROS topics and displays
them with cv2, and prints lidar range statistics to the console.

Usage:
  ros2 run sim_gazebo_bringup sensor_visualizer

You can selectively disable viewers with parameters:
  --ros-args -p show_mono:=false -p show_depth:=false -p show_lidar:=false
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np


class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')

        self.declare_parameter('show_mono', True)
        self.declare_parameter('show_depth', True)
        self.declare_parameter('show_lidar', True)
        self.declare_parameter('mono_topic', '/mono_camera/image_raw')
        self.declare_parameter('depth_topic', '/depth_camera/depth_image')
        self.declare_parameter('lidar_topic', '/scan')

        self._bridge = CvBridge()
        self._show_mono = self.get_parameter('show_mono').value
        self._show_depth = self.get_parameter('show_depth').value
        self._show_lidar = self.get_parameter('show_lidar').value

        if self._show_mono:
            self._sub_mono = self.create_subscription(
                Image,
                self.get_parameter('mono_topic').value,
                self._on_mono,
                1,
            )
            cv2.namedWindow('Mono Camera', cv2.WINDOW_NORMAL)
            self.get_logger().info('Mono camera viewer enabled')

        if self._show_depth:
            self._sub_depth = self.create_subscription(
                Image,
                self.get_parameter('depth_topic').value,
                self._on_depth,
                1,
            )
            cv2.namedWindow('Depth Camera', cv2.WINDOW_NORMAL)
            self.get_logger().info('Depth camera viewer enabled')

        if self._show_lidar:
            self._sub_lidar = self.create_subscription(
                LaserScan,
                self.get_parameter('lidar_topic').value,
                self._on_lidar,
                1,
            )
            self.get_logger().info('Lidar console printer enabled')

    def _on_mono(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Mono Camera', cv_img)
            cv2.waitKey(1)
        except cv2.error:
            pass
        except Exception as e:
            self.get_logger().warn(f'Mono display error: {e}')

    def _on_depth(self, msg: Image) -> None:
        try:
            # Depth images from Ignition are usually 32-bit float (metres)
            if msg.encoding == '32FC1':
                depth = self._bridge.imgmsg_to_cv2(msg, '32FC1')
            elif msg.encoding == '16UC1':
                depth = self._bridge.imgmsg_to_cv2(msg, '16UC1').astype(np.float32) * 0.001
            else:
                depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            valid = depth[np.isfinite(depth) & (depth > 0)]
            if len(valid) == 0:
                return

            # Normalise to 0-255 for display
            dmin, dmax = valid.min(), valid.max()
            display = np.clip((depth - dmin) / (dmax - dmin + 1e-6), 0, 1)
            display = (display * 255).astype(np.uint8)
            cv2.imshow('Depth Camera', display)
            cv2.waitKey(1)
        except cv2.error:
            # Headless / no display available – silently ignore
            pass
        except Exception as e:
            self.get_logger().warn(f'Depth display error: {e}')

    def _on_lidar(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid:
            self.get_logger().info('Lidar: no valid ranges')
            return
        self.get_logger().info(
            f'Lidar: angle {msg.angle_min:.2f}→{msg.angle_max:.2f} rad  |  '
            f'ranges {min(valid):.2f}–{max(valid):.2f} m  |  '
            f'{len(valid)}/{len(msg.ranges)} valid'
        )


def main():
    rclpy.init()
    node = SensorVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
