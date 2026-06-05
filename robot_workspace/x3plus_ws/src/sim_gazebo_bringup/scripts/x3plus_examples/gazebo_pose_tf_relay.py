#!/usr/bin/env python3
"""
gazebo_pose_tf_relay
====================

Re-publishes the robot's GROUND-TRUTH pose as a clean `odom -> base_footprint`
transform on /tf.

WHY: the URDF tree is rooted at `base_footprint`, while Ignition publishes
model pose separately from the URDF. Ignition's wheel-integrated `/odom`
over-reports yaw badly under skid (4 driven wheels), so using it as the base
TF makes the whole tree's heading wrong — the autopilot then faces slightly
off and misses the cube laterally, and 90 deg turns toward the drop zone are
grossly wrong. Instead we relay the ground-truth pose from the Ignition
PosePublisher (bridged to `/gz_pose_tf` as a TFMessage whose `child_frame_id`
is the model name, e.g. `x3plus`) and republish it as `odom -> base_footprint`.

A legacy mode is kept: if `input_topic` carries nav_msgs/Odometry (e.g. the old
`/odom` wiring) the node still works by reading the Odometry pose. The message
type is auto-selected from the `input_type` parameter.
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
        # Ground-truth source: TFMessage on /gz_pose_tf, pick the transform
        # whose child_frame_id == source_child (the Ignition model name).
        self.declare_parameter('input_topic', '/gz_pose_tf')
        self.declare_parameter('input_type', 'tf')      # 'tf' | 'odom'
        self.declare_parameter('source_child', 'x3plus')

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.source_child = self.get_parameter('source_child').value
        input_topic = self.get_parameter('input_topic').value
        input_type = self.get_parameter('input_type').value

        # /tf publisher: best-effort, volatile (matches tf2_ros default).
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)

        qos = QoSProfile(depth=100)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability = QoSDurabilityPolicy.VOLATILE

        if input_type == 'odom':
            self.sub = self.create_subscription(
                Odometry, input_topic, self._on_odometry, qos)
            self.get_logger().info(
                f"Relaying ODOMETRY pose from '{input_topic}' -> /tf as "
                f"'{self.parent_frame}'->'{self.child_frame}'")
        else:
            self.sub = self.create_subscription(
                TFMessage, input_topic, self._on_tf, qos)
            self.get_logger().info(
                f"Relaying GROUND-TRUTH pose from '{input_topic}' "
                f"(child '{self.source_child}') -> /tf as "
                f"'{self.parent_frame}'->'{self.child_frame}'")

    def _publish(self, stamp, translation, rotation) -> None:
        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.transform.translation.x = translation.x
        out.transform.translation.y = translation.y
        out.transform.translation.z = translation.z
        out.transform.rotation.x = rotation.x
        out.transform.rotation.y = rotation.y
        out.transform.rotation.z = rotation.z
        out.transform.rotation.w = rotation.w
        self.tf_pub.publish(TFMessage(transforms=[out]))

    def _on_tf(self, msg: TFMessage) -> None:
        # The PosePublisher emits the model pose plus every link pose. We want
        # the model root, identified by child_frame_id == source_child.
        for tr in msg.transforms:
            child = tr.child_frame_id.split('/')[-1]
            if child == self.source_child:
                self._publish(tr.header.stamp,
                              tr.transform.translation,
                              tr.transform.rotation)
                return

    def _on_odometry(self, msg: Odometry) -> None:
        self._publish(msg.header.stamp, msg.pose.pose.position,
                      msg.pose.pose.orientation)



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
