#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdomIntegrator(Node):
    """
    Integrates namespaced cmd_vel into odometry and TF.
    Topics are RELATIVE so launching in a namespace works:
      <ns>/cmd_vel  -> subscribe
      <ns>/odom     -> publish
      TF: odom_frame -> base_frame (both passed as params)

    Now supports strafing via linear.y.
    """

    def __init__(self):
        super().__init__('odom_integrator')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Relative topics
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)

        self.br = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.last = self.get_clock().now()

        self.create_timer(0.02, self.on_timer)  # 50 Hz

    def on_twist(self, msg: Twist):
        self.vx = float(msg.linear.x)   # forward/backward (robot frame)
        self.vy = float(msg.linear.y)   # left/right (robot frame)
        self.vw = float(msg.angular.z)  # yaw rate

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now
        if dt <= 0.0:
            return

        # Integrate heading
        self.yaw += self.vw * dt
        c, s = math.cos(self.yaw), math.sin(self.yaw)

        # Rotate body-frame (vx, vy) into world-frame and integrate position
        self.x += ( self.vx * c - self.vy * s) * dt
        self.y += ( self.vx * s + self.vy * c) * dt

        # Yaw quaternion (z,w for planar)
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        # TF: odom_frame -> base_frame
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

        # Odometry message
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id  = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.x = 0.0
        od.pose.pose.orientation.y = 0.0
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x  = self.vx
        od.twist.twist.linear.y  = self.vy
        od.twist.twist.linear.z  = 0.0
        od.twist.twist.angular.x = 0.0
        od.twist.twist.angular.y = 0.0
        od.twist.twist.angular.z = self.vw
        self.pub.publish(od)

def main():
    rclpy.init()
    rclpy.spin(OdomIntegrator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
