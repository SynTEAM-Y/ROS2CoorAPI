#!/usr/bin/env python3
"""
Gripper Mimic Relay — Relays grip_joint position to the 5 linkage mimic joints.

In Gazebo (Ignition), URDF <mimic> tags are not enforced by the physics engine.
This node subscribes to /joint_states, reads grip_joint, and republishes the
mimic joint commands so the gripper linkage moves visually.

Mimic joint mapping (from URDF):
  rlink_joint2 : multiplier = -1  (mirrors grip_joint)
  rlink_joint3 : multiplier =  1
  llink_joint1 : multiplier = -1
  llink_joint2 : multiplier =  1
  llink_joint3 : multiplier = -1

Usage:
    ros2 run x3plus_examples gripper_mimic_relay
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


GRIP_JOINT = 'grip_joint'

# {joint_name: multiplier}
MIMIC_JOINTS = {
    'rlink_joint2': -1.0,
    'rlink_joint3':  1.0,
    'llink_joint1': -1.0,
    'llink_joint2':  1.0,
    'llink_joint3': -1.0,
}


class GripperMimicRelay(Node):
    """Republishes grip_joint position to all mimic gripper joints."""

    def __init__(self):
        super().__init__('gripper_mimic_relay')

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(
            JointState, 'joint_states', self._cb, 10)

        self.get_logger().info('gripper_mimic_relay started')

    def _cb(self, msg: JointState):
        if GRIP_JOINT not in msg.name:
            return
        idx = msg.name.index(GRIP_JOINT)
        grip_pos = msg.position[idx] if msg.position else 0.0

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for joint, mult in MIMIC_JOINTS.items():
            out.name.append(joint)
            out.position.append(grip_pos * mult)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GripperMimicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
