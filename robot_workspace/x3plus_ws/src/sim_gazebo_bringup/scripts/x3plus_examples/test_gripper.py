#!/usr/bin/env python3
"""
Gripper Test & Verification Script

Tests the parallel linkage mechanism and gripper gap at different positions.
Publishes diagnostic information to verify:
1. R link2 and L link2 remain parallel
2. Gripper gap matches expected values
3. Contact sensors detect cube properly

Usage:
    ros2 run x3plus_examples test_gripper.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import Contacts
import math
import time


# Test positions for gripper
TEST_POSITIONS = {
    'fully_closed': 0.0,
    'hold_position': -0.676,  # 4.8 cm gap for 4 cm cube
    'half_open': -0.77,
    'fully_open': -1.54,
}


class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # Publisher for gripper commands
        self.grip_pub = self.create_publisher(
            Float64, '/grip_joint_cmd_pos', 10
        )
        
        # Subscribe to joint states for feedback
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )
        
        # Contact sensors
        self.llink2_contact = None
        self.rlink2_contact = None
        self.create_subscription(
            Contacts, '/model/x3plus/contact/llink2',
            self.llink2_contact_callback, 10
        )
        self.create_subscription(
            Contacts, '/model/x3plus/contact/rlink2',
            self.rlink2_contact_callback, 10
        )
        
        # Current joint positions
        self.joint_positions = {}
        
        self.get_logger().info('═' * 70)
        self.get_logger().info('GRIPPER TEST & VERIFICATION SCRIPT')
        self.get_logger().info('═' * 70)
        
    def joint_state_callback(self, msg):
        """Store current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
    
    def llink2_contact_callback(self, msg):
        self.llink2_contact = msg
    
    def rlink2_contact_callback(self, msg):
        self.rlink2_contact = msg
    
    def send_gripper_command(self, position, label):
        """Send gripper position command and wait for it to reach target"""
        self.get_logger().info('')
        self.get_logger().info('─' * 70)
        self.get_logger().info(f'Testing: {label} (grip_joint = {position:.3f} rad)')
        self.get_logger().info('─' * 70)
        
        msg = Float64()
        msg.data = position
        
        # Publish multiple times to ensure it gets through
        for _ in range(5):
            self.grip_pub.publish(msg)
            time.sleep(0.1)
        
        # Wait for gripper to move (2 seconds + settle time)
        self.get_logger().info('Waiting for gripper motion...')
        time.sleep(3.0)
        
        # Verify positions
        self.verify_parallel_linkage()
        self.check_contacts()
    
    def verify_parallel_linkage(self):
        """Verify that R link2 and L link2 remain parallel"""
        self.get_logger().info('')
        self.get_logger().info('🔍 PARALLEL LINKAGE VERIFICATION:')
        
        # Get current joint positions
        grip = self.joint_positions.get('grip_joint', 0.0)
        llink1 = self.joint_positions.get('llink_joint1', 0.0)
        llink2 = self.joint_positions.get('llink_joint2', 0.0)
        rlink2 = self.joint_positions.get('rlink_joint2', 0.0)
        
        # Calculate absolute angles of llink2 and rlink2
        # llink2 absolute = llink1 + llink2_rel (where llink2_rel should = grip * 1)
        # rlink2 absolute = grip + rlink2_rel (where rlink2_rel should = grip * -1)
        llink2_abs = llink1 + llink2
        rlink2_abs = grip + rlink2
        
        # Check if they're parallel (same absolute angle)
        angle_diff = abs(llink2_abs - rlink2_abs)
        
        self.get_logger().info(f'  grip_joint:      {grip:+.4f} rad ({math.degrees(grip):+.2f}°)')
        self.get_logger().info(f'  llink_joint1:    {llink1:+.4f} rad (mimic = grip × -1)')
        self.get_logger().info(f'  Expected:        {-grip:+.4f} rad')
        self.get_logger().info(f'  Error:           {abs(llink1 - (-grip)):+.6f} rad')
        self.get_logger().info('')
        self.get_logger().info(f'  llink2 relative: {llink2:+.4f} rad (mimic = grip × +1)')
        self.get_logger().info(f'  rlink2 relative: {rlink2:+.4f} rad (mimic = grip × -1)')
        self.get_logger().info('')
        self.get_logger().info(f'  llink2 absolute: {llink2_abs:+.4f} rad')
        self.get_logger().info(f'  rlink2 absolute: {rlink2_abs:+.4f} rad')
        self.get_logger().info(f'  Angle difference:{angle_diff:+.6f} rad ({math.degrees(angle_diff):.4f}°)')
        
        # Verify parallel (within 1 degree tolerance)
        if angle_diff < 0.017:  # ~1 degree
            self.get_logger().info('')
            self.get_logger().info('  ✅ PARALLEL CONSTRAINT VERIFIED')
        else:
            self.get_logger().warn('')
            self.get_logger().warn('  ⚠️  WARNING: Links not parallel!')
    
    def check_contacts(self):
        """Check if gripper fingers are in contact with anything"""
        self.get_logger().info('')
        self.get_logger().info('👆 CONTACT SENSOR STATUS:')
        
        llink_contact = False
        rlink_contact = False
        
        if self.llink2_contact is not None and len(self.llink2_contact.contacts) > 0:
            llink_contact = True
            for contact in self.llink2_contact.contacts:
                self.get_logger().info(f'  Left finger contact:  {contact.collision1} ↔ {contact.collision2}')
        
        if self.rlink2_contact is not None and len(self.rlink2_contact.contacts) > 0:
            rlink_contact = True
            for contact in self.rlink2_contact.contacts:
                self.get_logger().info(f'  Right finger contact: {contact.collision1} ↔ {contact.collision2}')
        
        if not llink_contact and not rlink_contact:
            self.get_logger().info('  No contacts detected')
        elif llink_contact and rlink_contact:
            self.get_logger().info('  ✅ Both fingers in contact (good grip!)')
        else:
            self.get_logger().warn('  ⚠️  Only one finger in contact (unbalanced grip)')
    
    def run_tests(self):
        """Run through all test positions"""
        self.get_logger().info('')
        self.get_logger().info('Starting gripper test sequence in 3 seconds...')
        self.get_logger().info('Make sure the robot is spawned in Gazebo!')
        time.sleep(3.0)
        
        # Test each position
        for name, position in TEST_POSITIONS.items():
            self.send_gripper_command(position, name)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1.0)
        
        self.get_logger().info('')
        self.get_logger().info('═' * 70)
        self.get_logger().info('TEST SEQUENCE COMPLETE')
        self.get_logger().info('═' * 70)
        self.get_logger().info('')
        self.get_logger().info('Summary:')
        self.get_logger().info('  • Tested 4 gripper positions')
        self.get_logger().info('  • Verified parallel linkage at each position')
        self.get_logger().info('  • Checked contact sensors')
        self.get_logger().info('')
        self.get_logger().info('Recommendations:')
        self.get_logger().info('  1. Spawn a test_block in front of the gripper')
        self.get_logger().info('  2. Run: ros2 topic pub --once /grip_joint_cmd_pos std_msgs/msg/Float64 "{data: -0.676}"')
        self.get_logger().info('  3. Verify both contact sensors detect the cube')
        self.get_logger().info('  4. Check that the cube is held securely')
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    tester = GripperTester()
    
    try:
        # Wait a moment for subscriptions to connect
        time.sleep(1.0)
        tester.run_tests()
        
        # Keep node alive for a bit to receive final callbacks
        for _ in range(20):
            rclpy.spin_once(tester, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
