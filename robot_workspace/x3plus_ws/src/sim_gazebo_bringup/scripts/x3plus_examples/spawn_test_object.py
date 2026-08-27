#!/usr/bin/env python3
"""Spawn a colored block in Gazebo for pick-and-place testing.

Usage:
  ros2 run sim_gazebo_bringup spawn_test_object --ros-args \
    -p x:=0.5 -p y:=0.0 -p z:=0.03 -p world:=empty
"""

import sys
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class TestObjectSpawner(Node):
    def __init__(self):
        super().__init__('spawn_test_object')

        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.03)
        self.declare_parameter('R', 0.0)
        self.declare_parameter('P', 0.0)
        self.declare_parameter('Y', 0.0)
        self.declare_parameter('world', 'empty')
        self.declare_parameter('name', 'test_block')
        self.declare_parameter('model_path', '')

        model_path = self.get_parameter('model_path').value
        if not model_path:
            try:
                pkg_dir = get_package_share_directory('sim_gazebo_bringup')
                model_path = os.path.join(pkg_dir, 'models', 'test_block', 'model.sdf')
            except Exception:
                model_path = os.path.join(
                    os.path.dirname(__file__), '..', '..', 'models',
                    'test_block', 'model.sdf')

        world = self.get_parameter('world').value
        name = self.get_parameter('name').value
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        R = self.get_parameter('R').value
        P = self.get_parameter('P').value
        Y = self.get_parameter('Y').value

        self.get_logger().warning(
            f'=== SPAWN PARAMS: name={name}, x={x}, y={y}, z={z} ===')

        self.get_logger().info(
            f'Spawning {name} at ({x}, {y}, {z}) in world "{world}"')

        spawn_cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', world,
            '-file', model_path,
            '-name', name,
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-R', str(R), '-P', str(P), '-Y', str(Y),
        ]

        cmd_str = ' '.join(spawn_cmd)
        self.get_logger().info(f'Running: {cmd_str}')

        import subprocess
        result = subprocess.run(spawn_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info(f'Spawned {name} successfully')
        else:
            self.get_logger().error(
                f'Failed to spawn {name}: {result.stderr.strip()}')
        # NOTE: Do NOT call rclpy.shutdown() here — it destroys the context
        # before spin() can run. The node will exit naturally after __init__
        # because we don't start a timer or subscription.


def main():
    rclpy.init()
    node = TestObjectSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
