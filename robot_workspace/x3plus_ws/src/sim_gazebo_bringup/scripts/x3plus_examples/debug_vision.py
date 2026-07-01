#!/usr/bin/env python3
"""Quick diagnostic for vision detection issues."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class VisionDebugger(Node):
    def __init__(self):
        super().__init__('vision_debugger')
        
        # Subscribe to all relevant topics
        self.create_subscription(
            Image, '/wrist_mono_camera/image_raw', self._on_wrist_image, 1)
        self.create_subscription(
            PoseStamped, '/detected_object_pose', self._on_pose, 1)
        
        print("\n" + "="*60)
        print("VISION DEBUG MONITOR")
        print("="*60)
        print("Listening for:")
        print("  - /wrist_mono_camera/image_raw (wrist camera)")
        print("  - /detected_object_pose (cube detection)")
        print("\nPress Ctrl+C to stop.\n")
        
        self.last_pose_time = None
        self.pose_count = 0
        self.image_count = 0
        
    def _on_wrist_image(self, msg):
        self.image_count += 1
        if self.image_count % 10 == 0:
            print(f"✓ Wrist camera image received (count={self.image_count})")
    
    def _on_pose(self, msg: PoseStamped):
        self.pose_count += 1
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        print(f"✓ DETECTION #{self.pose_count}: cube at ({x:.3f}, {y:.3f}, {z:.3f}) in frame '{msg.header.frame_id}'")
        self.last_pose_time = self.get_clock().now()

def main():
    rclpy.init()
    node = VisionDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nDEBUG SUMMARY:")
        print(f"  Wrist images received: {node.image_count}")
        print(f"  Cube detections received: {node.pose_count}")
        if node.pose_count == 0:
            print("\n⚠️  NO DETECTIONS! Check:")
            print("     1. Is object_detector running?")
            print("     2. Is the cube visible in camera view?")
            print("     3. Are HSV ranges correct? (check config/hsv_colors.yaml)")
            print("\n  Quick fix - run object_detector in calibration mode:")
            print("     ros2 run sim_gazebo_bringup object_detector --ros-args -p calibrate_mode:=true")
        print()

if __name__ == '__main__':
    main()
