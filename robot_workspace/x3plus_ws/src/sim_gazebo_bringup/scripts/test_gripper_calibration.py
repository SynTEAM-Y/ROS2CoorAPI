#!/usr/bin/env python3
"""
Gripper Calibration Test for 2cm Cube

Purpose:
  Systematically test different grip_joint values to find the optimal
  GRIPPER_HOLD setting that allows the robot to successfully pick up
  the 2cm × 2cm × 2cm test cube.

Workflow:
  1. Launch Gazebo with empty world
  2. Spawn 2cm test cube at (2.0, 0.0, 0.03)
  3. Move arm to REACH_DOWN pose (gripper above cube)
  4. Test grip values from OPEN to CLOSE in steps
  5. At each value: close gripper, wait, check if cube lifted (z > 0.10m)
  6. Record success/failure and print calibration table
  7. Output recommended GRIPPER_HOLD value

Expected Results:
  - Low grip values (< -1.0): Gripper too open, cube not lifted (z ≈ 0.01m)
  - Mid values (-0.7 to -0.3): Possible contact range
  - High values (> -0.2): Gripper may over-squeeze
  - SUCCESS: First value where cube lifts to z > 0.10m after grip
"""

import math
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from tf2_ros import TransformListener, Buffer, TransformException
import subprocess
import signal


# Arm poses (from vision_autopilot_simple.py)
HOME        = [0.0,   0.0,    0.0,    0.0,   0.0]
REACH_DOWN  = [0.0,  -1.45,  -0.524, -1.21,  0.0]  # Gripper vertical over cube

# Gripper test range
GRIPPER_OPEN  = -1.54
GRIPPER_CLOSE = 0.0
TEST_STEP     = 0.05   # Test in 0.05 rad increments (~2.9°)


class GripperCalibrator(Node):
    def __init__(self):
        super().__init__('gripper_calibrator')
        
        # TF listener for cube pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint publishers
        self.joint_publishers = {}
        joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'grip_joint']
        for name in joint_names:
            self.joint_publishers[name] = self.create_publisher(Float64, f'/{name}_cmd_pos', 10)
        
        # Command velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/x3plus/cmd_vel', 10)
        
        # Test results
        self.results = []
        self.current_grip_value = None
        self.test_active = False
        self.cube_z_after_grip = None
        
        # State machine
        self.state = 'INIT'
        self.state_timer = self.create_timer(0.1, self.timer_callback)
        self.init_time = None
        self.state_start_time = None
        
        self.get_logger().info('Gripper Calibrator initialized')

    def publish_arm_pose(self, arm_pos, grip_pos):
        """Publish arm and gripper joint targets."""
        names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        for name, pos in zip(names, arm_pos):
            msg = Float64()
            msg.data = float(pos)
            self.joint_publishers[name].publish(msg)
        
        gmsg = Float64()
        gmsg.data = float(grip_pos)
        self.joint_publishers['grip_joint'].publish(gmsg)

    def get_cube_pose(self):
        """Get cube position from TF."""
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'test_block', rclpy.time.Time())
            return trans.transform.translation
        except TransformException:
            return None

    def sim_sleep(self, dur_s):
        """Sleep for sim time (not wall time)."""
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < dur_s * 1e9:
            time.sleep(0.05)

    def timer_callback(self):
        """Main state machine."""
        if self.state == 'INIT':
            # Wait for Gazebo to fully start
            if self.init_time is None:
                self.init_time = self.get_clock().now()
            
            elapsed = (self.get_clock().now() - self.init_time).nanoseconds / 1e9
            if elapsed > 35:  # Wait for cube and TF relays to be ready
                self.get_logger().info('[INIT] Gazebo ready, moving to REACH_DOWN pose')
                # Start moving arm to REACH_DOWN
                for _ in range(25):  # Interpolate over 25 steps
                    self.publish_arm_pose(REACH_DOWN, GRIPPER_OPEN)
                    self.sim_sleep(0.1)
                self.state = 'REACH_DOWN_SETTLE'
                self.state_start_time = self.get_clock().now()
        
        elif self.state == 'REACH_DOWN_SETTLE':
            # Wait for arm to settle
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            if elapsed > 2:
                self.get_logger().info('[REACH_DOWN_SETTLE] Arm settled, starting grip tests')
                self.state = 'TEST_NEXT_GRIP'
                self.current_grip_value = GRIPPER_OPEN
        
        elif self.state == 'TEST_NEXT_GRIP':
            # Move to next grip value
            # Start from OPEN (-1.54) and step TOWARD CLOSE (0.0)
            if self.current_grip_value < GRIPPER_CLOSE - 0.01:
                self.get_logger().info(f'[TEST_NEXT_GRIP] Testing grip_joint = {self.current_grip_value:.3f} rad ({math.degrees(self.current_grip_value-math.pi/2):.1f}°)')
                self.state = 'GRIP_CLOSE'
                self.state_start_time = self.get_clock().now()
            else:
                # All tests done
                self.state = 'PRINT_RESULTS'
        
        elif self.state == 'GRIP_CLOSE':
            # Close gripper at current test value
            self.publish_arm_pose(REACH_DOWN, self.current_grip_value)
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            if elapsed > 1.5:  # Wait for gripper to settle
                self.state = 'GRIP_CHECK'
                self.state_start_time = self.get_clock().now()
        
        elif self.state == 'GRIP_CHECK':
            # Check if cube was lifted
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            
            cube_pose = self.get_cube_pose()
            if cube_pose:
                z = cube_pose.z
                success = z > 0.10
                
                self.results.append({
                    'grip_joint': self.current_grip_value,
                    'grip_deg': math.degrees(self.current_grip_value) - 90,
                    'cube_z': z,
                    'success': success
                })
                
                status = '✓ SUCCESS' if success else f'✗ FAILED (z={z:.4f}m)'
                self.get_logger().info(f'  → Grip {self.current_grip_value:.3f}: {status}')
            
            if elapsed > 0.5:  # Wait a bit to ensure measurement is valid
                # Reset gripper to open for next test
                self.publish_arm_pose(REACH_DOWN, GRIPPER_OPEN)
                self.current_grip_value += TEST_STEP  # ADD to move from -1.54 toward 0.0
                self.state = 'TEST_NEXT_GRIP'
                self.state_start_time = self.get_clock().now()
                self.sim_sleep(0.5)  # Wait for gripper to open
        
        elif self.state == 'PRINT_RESULTS':
            self.print_calibration_results()
            self.state = 'DONE'
        
        elif self.state == 'DONE':
            self.get_logger().info('[DONE] Test complete. Shutting down...')
            rclpy.shutdown()

    def print_calibration_results(self):
        """Print calibration table and recommendations."""
        self.get_logger().info('')
        self.get_logger().info('=' * 80)
        self.get_logger().info('GRIPPER CALIBRATION RESULTS FOR 2CM CUBE')
        self.get_logger().info('=' * 80)
        self.get_logger().info('')
        self.get_logger().info(f'{"Grip Value (rad)":<20} {"Servo Angle (°)":<20} {"Cube Z (m)":<20} {"Status":<15}')
        self.get_logger().info('-' * 75)
        
        # Print results in order
        for r in sorted(self.results, key=lambda x: x['grip_joint'], reverse=True):
            grip_str = f"{r['grip_joint']:.3f}"
            deg_str = f"{r['grip_deg']:+.1f}"
            z_str = f"{r['cube_z']:.5f}"
            status_str = '✓ PICKED' if r['success'] else '✗ NOT LIFTED'
            
            self.get_logger().info(f'{grip_str:<20} {deg_str:<20} {z_str:<20} {status_str:<15}')
        
        self.get_logger().info('')
        self.get_logger().info('-' * 75)
        
        # Find optimal value (first successful)
        successful = [r for r in self.results if r['success']]
        if successful:
            # Get the least closed (most open) successful value
            optimal = max(successful, key=lambda x: x['grip_joint'])
            self.get_logger().info('')
            self.get_logger().info('🎯 CALIBRATION RESULT:')
            self.get_logger().info(f'   Optimal GRIPPER_HOLD = {optimal["grip_joint"]:.3f} rad')
            self.get_logger().info(f'   Servo angle: {optimal["grip_deg"]:+.1f}°')
            self.get_logger().info(f'   Cube lifted to: {optimal["cube_z"]:.5f} m')
            self.get_logger().info('')
            self.get_logger().info('📋 UPDATE vision_autopilot_simple.py:')
            self.get_logger().info(f'   GRIPPER_HOLD = {optimal["grip_joint"]:.2f}')
            self.get_logger().info('')
        else:
            self.get_logger().error('❌ No successful grip values found!')
            self.get_logger().info('   This indicates a fundamental issue with:')
            self.get_logger().info('   1. Arm pose (REACH_DOWN j4 angle may still be wrong)')
            self.get_logger().info('   2. Gripper joint range (may not be closing at all)')
            self.get_logger().info('   3. Cube physics (may be too small/light)')
        
        self.get_logger().info('')
        self.get_logger().info('=' * 80)


def main():
    rclpy.init()
    node = GripperCalibrator()
    
    # Run with MultiThreadedExecutor to handle TF updates in parallel
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
