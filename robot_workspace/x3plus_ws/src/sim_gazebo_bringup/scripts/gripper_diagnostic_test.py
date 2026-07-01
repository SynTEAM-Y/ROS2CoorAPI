#!/usr/bin/env python3
"""
Gripper Diagnostic Test - Verify each component individually
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import time
import sys

class GripperDiagnostics(Node):
    def __init__(self):
        super().__init__('gripper_diagnostic')
        self.get_logger().info("[INIT] Gripper Diagnostic Test Starting")
        
        # Publishers for arm joints
        self.j1_pub = self.create_publisher(Float64, '/arm_joint1_cmd_pos', 10)
        self.j2_pub = self.create_publisher(Float64, '/arm_joint2_cmd_pos', 10)
        self.j3_pub = self.create_publisher(Float64, '/arm_joint3_cmd_pos', 10)
        self.j4_pub = self.create_publisher(Float64, '/arm_joint4_cmd_pos', 10)
        self.j5_pub = self.create_publisher(Float64, '/arm_joint5_cmd_pos', 10)
        self.grip_pub = self.create_publisher(Float64, '/grip_joint_cmd_pos', 10)
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Poses
        self.REACH_DOWN = [0.0, -1.45, -0.524, -1.21, 0.0]
        self.GRIPPER_OPEN = -1.54
        self.GRIPPER_CLOSE = 0.0
        self.TEST_GRIP = -0.50
        
        # State machine
        self.state = 'INIT'
        self.step_time = 0
        
        # Create timer for state machine
        self.create_timer(0.1, self.state_machine)
    
    def set_arm_joints(self, joints):
        """Publish arm joint commands"""
        self.j1_pub.publish(Float64(data=joints[0]))
        self.j2_pub.publish(Float64(data=joints[1]))
        self.j3_pub.publish(Float64(data=joints[2]))
        self.j4_pub.publish(Float64(data=joints[3]))
        self.j5_pub.publish(Float64(data=joints[4]))
    
    def set_gripper(self, value):
        """Publish gripper command"""
        self.grip_pub.publish(Float64(data=value))
        rad = value
        deg = (rad * 180 / 3.14159) + 90
        self.get_logger().info(f"[GRIPPER] Commanding grip_joint = {rad:.4f} rad ({deg:.1f}°)")
    
    def check_tf(self, frame_name):
        """Check if TF frame exists"""
        try:
            trans = self.tf_buffer.lookup_transform('map', frame_name, rclpy.time.Time())
            z = trans.transform.translation.z
            self.get_logger().info(f"[TF] {frame_name}: Z = {z:.4f}m")
            return z
        except Exception as e:
            self.get_logger().warn(f"[TF] {frame_name}: NOT FOUND ({str(e)})")
            return None
    
    def state_machine(self):
        """Main diagnostic state machine"""
        self.step_time += 1
        elapsed = self.step_time * 0.1
        
        if self.state == 'INIT':
            if elapsed > 2:
                self.get_logger().info(f"\n[STEP 1] Checking TF frames...")
                self.state = 'CHECK_TF'
                self.step_time = 0
        
        elif self.state == 'CHECK_TF':
            if self.step_time == 1:
                self.check_tf('x3plus')
                self.check_tf('test_block')
                cube_z = self.check_tf('test_block')
                if cube_z is not None:
                    self.get_logger().info(f"✓ Test cube found at Z={cube_z:.4f}m")
                else:
                    self.get_logger().error("✗ Test cube NOT FOUND in TF!")
            if elapsed > 2:
                self.get_logger().info(f"\n[STEP 2] Testing arm movement to REACH_DOWN...")
                self.set_arm_joints(self.REACH_DOWN)
                self.state = 'ARM_REACH'
                self.step_time = 0
        
        elif self.state == 'ARM_REACH':
            if elapsed > 3.5:  # Wait for arm to settle
                self.get_logger().info(f"[ARM] Arm should be at REACH_DOWN now")
                self.get_logger().info(f"\n[STEP 3] Testing gripper OPEN command...")
                self.set_gripper(self.GRIPPER_OPEN)
                self.state = 'GRIPPER_OPEN'
                self.step_time = 0
        
        elif self.state == 'GRIPPER_OPEN':
            if elapsed > 1:
                self.get_logger().info(f"\n[STEP 4] Testing gripper CLOSE command...")
                self.set_gripper(self.GRIPPER_CLOSE)
                self.state = 'GRIPPER_CLOSE'
                self.step_time = 0
        
        elif self.state == 'GRIPPER_CLOSE':
            if elapsed > 1:
                self.get_logger().info(f"\n[STEP 5] Testing intermediate grip value...")
                self.set_gripper(self.TEST_GRIP)
                self.state = 'GRIPPER_TEST'
                self.step_time = 0
        
        elif self.state == 'GRIPPER_TEST':
            if elapsed > 1:
                self.get_logger().info(f"\n[STEP 6] Checking cube position after gripper close...")
                cube_z = self.check_tf('test_block')
                if cube_z is not None:
                    if cube_z > 0.05:
                        self.get_logger().info(f"✓ Cube lifted! Z = {cube_z:.4f}m")
                    else:
                        self.get_logger().warn(f"✗ Cube NOT lifted, still on ground (Z = {cube_z:.4f}m)")
                self.state = 'DONE'
                self.step_time = 0
        
        elif self.state == 'DONE':
            if elapsed > 1:
                self.get_logger().info(f"\n[DONE] Diagnostic complete.")
                self.get_logger().info(f"\nSummary:")
                self.get_logger().info(f"  - TF frames: Check if x3plus and test_block visible")
                self.get_logger().info(f"  - Arm movement: Should have moved")
                self.get_logger().info(f"  - Gripper commands: Sent to /grip_joint_cmd_pos")
                self.get_logger().info(f"  - Cube lift: Did gripper lift the cube?")
                rclpy.shutdown()

def main():
    rclpy.init()
    diagnostics = GripperDiagnostics()
    rclpy.spin(diagnostics)

if __name__ == '__main__':
    main()
