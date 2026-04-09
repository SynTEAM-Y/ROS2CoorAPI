#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdomIntegrator(Node):
    """A simple ROS 2 node that integrates /cmd_vel commands into odometry data
    and publishes both /odom and the corresponding TF transform.
    """

    def __init__(self):
        super().__init__('odom_integrator')

        # Declare and retrieve frame parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Initialize robot pose (x, y, yaw) and velocities (vx, vw)
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.vx = 0.0; self.vw = 0.0
        self.last = self.get_clock().now()  # Timestamp for integration

        # Subscribe to velocity commands and publish odometry output
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster for publishing odom → base_link transform
        self.br  = TransformBroadcaster(self)

        # Create a timer for integration loop (50 Hz)
        self.create_timer(0.02, self.on_timer)

    def on_twist(self, msg: Twist):
        """Callback for incoming /cmd_vel messages."""
        self.vx = float(msg.linear.x)
        self.vw = float(msg.angular.z)

    def on_timer(self):
        """Periodic callback to integrate velocities into pose and publish odometry."""
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now
        if dt <= 0.0:
            return  # Skip invalid time deltas

        # Integrate linear and angular motion
        self.yaw += self.vw * dt
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += self.vx * c * dt
        self.y += self.vx * s * dt

        # Compute quaternion from yaw (2D rotation)
        qz, qw = math.sin(self.yaw * 0.5), math.cos(self.yaw * 0.5)

        # --- Publish TF transform (odom → base_link) ---
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

        # --- Publish Odometry message ---
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id  = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x  = self.vx
        od.twist.twist.angular.z = self.vw
        self.pub.publish(od)

def main():
    """Initialize and spin the odometry integrator node."""
    rclpy.init()
    rclpy.spin(OdomIntegrator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
