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


class _OnceLog:
    def __init__(self, node):
        self.node = node
        self.seen = set()

    def log(self, key: str, msg: str):
        if key not in self.seen:
            self.seen.add(key)
            self.node.get_logger().info(msg)


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

        # Use reliable QoS for the input topic so the relay does not miss
        # messages if it starts after the bridge.
        qos = QoSProfile(depth=100)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        self._once = _OnceLog(self)

        # Keep the last known pose and re-publish it periodically.  This keeps
        # the transform alive in the TF buffer even if the Ignition
        # PosePublisher slows down or stops, and protects the autopilot from
        # transient message drops.
        self._last_transform = None
        self.create_timer(0.1, self._republish_last)

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
        self.get_logger().info('TF relay node initialized')

    @staticmethod
    def _child_matches(child_frame_id: str, source_child: str) -> bool:
        """
        Check whether child_frame_id identifies the source model.
        Ignition may emit bare model names, scoped names (model/link), or
        nested names (model::link), so we accept any of those forms.
        """
        if not child_frame_id:
            return False
        if child_frame_id == source_child:
            return True
        # Last token after '/' (e.g. "yellow_object/base_link" -> "base_link").
        if child_frame_id.split('/')[-1] == source_child:
            return True
        # First token before '/' or '::' (e.g. "yellow_object/base_link").
        for sep in ('/', '::'):
            if sep in child_frame_id:
                if child_frame_id.split(sep)[0] == source_child:
                    return True
        return False

    def _publish(self, stamp, translation, rotation) -> None:
        out = TransformStamped()
        # The Ignition world-pose topic often arrives with header.stamp = 0.
        # robot_state_publisher and TF lookups use sim time, so stamp the
        # relayed transform with the current node clock instead.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.transform.translation.x = translation.x
        out.transform.translation.y = translation.y
        out.transform.translation.z = translation.z
        out.transform.rotation.x = rotation.x
        out.transform.rotation.y = rotation.y
        out.transform.rotation.z = rotation.z
        out.transform.rotation.w = rotation.w
        self._last_transform = out
        self.tf_pub.publish(TFMessage(transforms=[out]))
        self._once.log(
            'published_first',
            f"[{self.child_frame}] published {self.parent_frame} -> {self.child_frame} "
            f"@ t={out.header.stamp.sec}.{out.header.stamp.nanosec:09d} "
            f"p=({out.transform.translation.x:.3f}, {out.transform.translation.y:.3f}, "
            f"{out.transform.translation.z:.3f})")

    def _republish_last(self) -> None:
        if self._last_transform is not None:
            self._last_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_pub.publish(TFMessage(transforms=[self._last_transform]))

    def _on_tf(self, msg: TFMessage) -> None:
        try:
            # The PosePublisher emits the model pose plus every link pose. We want
            # the model root, identified by child_frame_id == source_child.
            self._once.log('recv_first', f"[{self.child_frame}] first /gz_pose_tf received ({len(msg.transforms)} transforms)")
            for tr in msg.transforms:
                if self._child_matches(tr.child_frame_id, self.source_child):
                    self._once.log('match_first', f"[{self.child_frame}] matched source_child '{self.source_child}' "
                                                   f"(child_frame_id='{tr.child_frame_id}'), publishing to /tf")
                    self._publish(tr.header.stamp,
                                  tr.transform.translation,
                                  tr.transform.rotation)
                    return
            # Throttled debug to see which child_frame_ids are being ignored.
            children = [t.child_frame_id for t in msg.transforms[:8]]
            self._once.log('no_match_first',
                           f"[{self.child_frame}] no source_child '{self.source_child}' in /gz_pose_tf "
                           f"(children: {children})")
        except Exception as e:
            self.get_logger().error(f'Error processing TFMessage: {e}')

    def _on_odometry(self, msg: Odometry) -> None:
        try:
            self._publish(msg.header.stamp, msg.pose.pose.position,
                          msg.pose.pose.orientation)
        except Exception as e:
            self.get_logger().error(f'Error processing Odometry: {e}')



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
