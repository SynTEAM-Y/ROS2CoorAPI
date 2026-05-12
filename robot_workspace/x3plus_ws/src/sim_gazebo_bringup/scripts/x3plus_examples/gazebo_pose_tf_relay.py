#!/usr/bin/env python3
"""
gazebo_pose_tf_relay
====================

Re-publishes robot pose from /odom as a clean `odom -> base_footprint`
transform on /tf.

WHY: the URDF tree is rooted at `base_footprint`, while Ignition publishes
model odometry separately from the URDF. This node reuses the bridge-provided
`/odom` message and republishes its pose on /tf so RViz can connect `map ->
odom -> base_footprint`.

This keeps RViz visually identical to Gazebo (no wheel-odometry slip drift),
which fixes the "press 2 -> Gazebo turns 90deg, RViz turns 2.3 revolutions"
mismatch caused by running an independent kinematic simulator alongside.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class GazeboPoseTfRelay(Node):
    def __init__(self):
        super().__init__('gazebo_pose_tf_relay')

        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('input_topic', '/odom')

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        input_topic = self.get_parameter('input_topic').value

        # /tf publisher: best-effort, volatile (matches tf2_ros default).
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)

        qos = QoSProfile(depth=100)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self.sub = self.create_subscription(
            Odometry, input_topic, self._on_odometry, qos
        )

        self.get_logger().info(
            f"Relaying odometry pose from '{input_topic}' -> /tf as "
            f"'{self.parent_frame}'->'{self.child_frame}'"
        )

    def _on_odometry(self, msg: Odometry) -> None:
        child_frame = self.child_frame or msg.child_frame_id
        if not child_frame:
            self.get_logger().warning(
                'Odometry message child_frame_id is empty; skipping TF publish.'
            )
            return

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.parent_frame
        out.child_frame_id = child_frame
        out.transform.translation = msg.pose.pose.position
        out.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[out]))


def main():
    rclpy.init()
    node = GazeboPoseTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
