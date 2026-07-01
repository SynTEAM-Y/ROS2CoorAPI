#!/usr/bin/env python3

import math
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


class DiffDriveSimulator(Node):
    def __init__(self):
        super().__init__('diff_drive_simulator')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        q = quaternion_from_yaw(self.yaw)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_publisher.publish(odom)

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'back_left_wheel_joint', 'back_right_wheel_joint',
            'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5',
            'grip_joint', 'rlink_joint3', 'llink_joint1', 'llink_joint2', 'llink_joint3',
            'rlink_joint1', 'rlink_joint2'
        ]
        joint_state.position = [0.0] * len(joint_state.name)
        self.joint_state_publisher.publish(joint_state)


def main():
    rclpy.init()
    node = DiffDriveSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
