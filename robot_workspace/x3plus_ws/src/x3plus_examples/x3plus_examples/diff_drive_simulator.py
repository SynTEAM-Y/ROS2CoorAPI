#!/usr/bin/env python3
"""
Differential Drive Simulator for RViz-Only Mode

Subscribes to /cmd_vel (geometry_msgs/Twist) and integrates position using
differential drive kinematics. Publishes:
  - /odom           (nav_msgs/Odometry)
  - /joint_states   (sensor_msgs/JointState)  — wheel joint angles
  - TF: odom → base_footprint

Robot parameters (X3plus):
  Wheel separation  L = 0.2128 m
  Wheel radius      r = 0.04   m

Kinematics (per time-step dt):
  v   = (v_R + v_L) / 2          linear velocity
  ω   = (v_R - v_L) / L          angular velocity
  x  += v * cos(θ) * dt
  y  += v * sin(θ) * dt
  θ  += ω * dt

Usage:
    ros2 run x3plus_examples diff_drive_simulator
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a Quaternion (2-D, z-axis only)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DiffDriveSimulator(Node):
    """Pure-software differential drive odometry for RViz simulation."""

    # ---- Robot geometry (must match URDF and manual_control.py) ----
    WHEEL_SEPARATION = 0.2128   # m
    WHEEL_RADIUS = 0.04         # m
    UPDATE_RATE = 20.0          # Hz
    ARM_JOINTS = [
        'arm_joint1',
        'arm_joint2',
        'arm_joint3',
        'arm_joint4',
        'arm_joint5',
        'grip_joint',
    ]

    def __init__(self):
        super().__init__('diff_drive_simulator')

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        # Accumulated wheel angles (for joint_states)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # Cached arm state so RViz always has complete joint transforms.
        self.arm_positions = {joint: 0.0 for joint in self.ARM_JOINTS}

        # ROS interfaces
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10)

        dt = 1.0 / self.UPDATE_RATE
        self.timer = self.create_timer(dt, self._update)

        self.get_logger().info(
            f'diff_drive_simulator started '
            f'(L={self.WHEEL_SEPARATION} m, r={self.WHEEL_RADIUS} m, '
            f'{self.UPDATE_RATE} Hz)'
        )

    def _cmd_cb(self, msg: Twist):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    def _joint_state_cb(self, msg: JointState):
        # Merge arm/gripper values from any external publisher (e.g. arm_controller).
        if not msg.name or not msg.position:
            return
        max_i = min(len(msg.name), len(msg.position))
        for i in range(max_i):
            joint = msg.name[i]
            if joint in self.arm_positions:
                self.arm_positions[joint] = msg.position[i]

    def _update(self):
        dt = 1.0 / self.UPDATE_RATE
        now = self.get_clock().now().to_msg()

        # Integrate pose
        self.theta += self.w_cmd * dt
        self.x += self.v_cmd * math.cos(self.theta) * dt
        self.y += self.v_cmd * math.sin(self.theta) * dt

        # Wheel angular velocities → accumulated angles
        v_right = self.v_cmd + self.w_cmd * self.WHEEL_SEPARATION / 2.0
        v_left = self.v_cmd - self.w_cmd * self.WHEEL_SEPARATION / 2.0
        self.right_wheel_angle += (v_right / self.WHEEL_RADIUS) * dt
        self.left_wheel_angle += (v_left / self.WHEEL_RADIUS) * dt

        q = euler_to_quaternion(self.theta)

        # --- Publish TF: odom → base_footprint ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf_msg)

        # --- Publish /odom ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v_cmd
        odom.twist.twist.angular.z = self.w_cmd
        self.odom_pub.publish(odom)

        # --- Publish /joint_states (wheels + cached arm state) ---
        # Keep wheel motion in sync with odom while preserving latest arm commands.
        js = JointState()
        js.header.stamp = now
        js.name = [
            'left_wheel_joint',
            'right_wheel_joint',
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'arm_joint4',
            'arm_joint5',
            'grip_joint',
        ]
        js.position = [
            self.left_wheel_angle,
            self.right_wheel_angle,
            self.arm_positions['arm_joint1'],
            self.arm_positions['arm_joint2'],
            self.arm_positions['arm_joint3'],
            self.arm_positions['arm_joint4'],
            self.arm_positions['arm_joint5'],
            self.arm_positions['grip_joint'],
        ]
        self.joint_pub.publish(js)


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
