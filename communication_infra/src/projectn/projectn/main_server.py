#!/usr/bin/env python3
# ProjectN • Server entrypoint (ROS 2)
import rclpy
from rclpy.executors import MultiThreadedExecutor
from projectn.core.tree_node import TreeNode

def main():
    # Let rclpy parse ROS args (includes your --ros-args -p name:=... etc.)
    rclpy.init()
    node = None
    exec = None
    try:
        # TreeNode reads 'name', 'is_root', 'parent' from ROS parameters
        node = TreeNode()
        exec = MultiThreadedExecutor(num_threads=8)
        exec.add_node(node)
        exec.spin()
    finally:
        if exec is not None:
            try:
                exec.shutdown()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()

