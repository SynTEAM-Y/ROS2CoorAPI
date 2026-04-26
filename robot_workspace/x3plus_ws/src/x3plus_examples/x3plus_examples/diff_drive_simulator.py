#!/usr/bin/env python3
"""
Differential Drive Simulator Node

This node simulates a differential drive robot's odometry in software,
used for RViz-only mode when Gazebo is not running.

Subscribes to /cmd_vel and publishes /odom, /joint_states and broadcasts TF.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
import math


class DiffDriveSimulator(Node):
    def __init__(self):
        super().__init__('diff_drive_simulator')
        
        # Robot parameters (matching X3plus specs)
        self.wheel_separation = 0.2128  # meters
        self.wheel_radius = 0.04  # meters
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        # Create subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Create publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Initialize joint positions (all zeros for default pose).
        # Gripper starts CLOSED (grip_joint = 0.0 for this URDF mesh).
        # Mimic joints follow grip_joint with multipliers (-1, +1, -1, +1, -1).
        _grip_init = 0.0
        self.joint_positions = {
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'back_left_wheel_joint': 0.0,
            'back_right_wheel_joint': 0.0,
            'arm_joint1': 0.0,
            'arm_joint2': 0.0,
            'arm_joint3': 0.0,
            'arm_joint4': 0.0,
            'arm_joint5': 0.0,
            'grip_joint': _grip_init,
            'rlink_joint2': -_grip_init,
            'rlink_joint3':  _grip_init,
            'llink_joint1': -_grip_init,
            'llink_joint2':  _grip_init,
            'llink_joint3': -_grip_init,
        }
        
        # Subscribe to arm joint command topics published by arm_controller.py
        arm_joints = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        for joint in arm_joints:
            self.create_subscription(
                Float64,
                f'/{joint}_cmd_pos',
                lambda msg, j=joint: self._arm_cmd_cb(msg, j),
                10
            )
        self.create_subscription(
            Float64,
            '/grip_joint_cmd_pos',
            self._grip_cmd_cb,
            10
        )

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for odometry updates (200 Hz)
        self.timer = self.create_timer(0.005, self.update_odometry)
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Differential Drive Simulator initialized')
        self.get_logger().info(f'  Wheel Separation: {self.wheel_separation} m')
        self.get_logger().info(f'  Wheel Radius: {self.wheel_radius} m')
    
    def _arm_cmd_cb(self, msg, joint_name):
        """Update arm joint position from arm_controller command"""
        self.joint_positions[joint_name] = msg.data

    def _grip_cmd_cb(self, msg):
        """Update gripper and mimic joints from grip_joint command"""
        pos = msg.data
        self.joint_positions['grip_joint'] = pos
        # Mimic joints follow grip_joint with their multipliers
        self.joint_positions['rlink_joint2'] = -pos   # multiplier -1
        self.joint_positions['rlink_joint3'] = pos    # multiplier 1
        self.joint_positions['llink_joint1'] = -pos   # multiplier -1
        self.joint_positions['llink_joint2'] = pos    # multiplier 1
        self.joint_positions['llink_joint3'] = -pos   # multiplier -1

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    
    def update_odometry(self):
        """Update robot pose based on velocity commands"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update pose using differential drive kinematics
        # Only update if there's actual velocity to reduce jitter when stationary
        if abs(self.vx) > 1e-6 or abs(self.vth) > 1e-6:
            delta_x = self.vx * math.cos(self.theta) * dt
            delta_y = self.vx * math.sin(self.theta) * dt
            delta_theta = self.vth * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom_msg)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish joint states for robot visualization
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = list(self.joint_positions.keys())
        joint_state.position = list(self.joint_positions.values())
        # Set velocities to zero (could calculate wheel velocities from cmd_vel if needed)
        joint_state.velocity = [0.0] * len(joint_state.name)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
