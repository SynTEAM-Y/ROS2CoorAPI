#!/usr/bin/env python3
"""
Gripper Mimic Joint State Filter

Ignition Gazebo's JointStatePublisher emits physics positions for ALL joints,
including the passive mimic finger joints (llink_joint1-3, rlink_joint2-3)
which are frozen at 0 in physics (gravity disabled, no controller).

robot_state_publisher (RSP) in ROS2 honours URDF <mimic> tags ONLY when the
mimic joint is NOT present in the incoming /joint_states message — if the joint
is present it uses the value directly, bypassing the mimic computation.

This node:
  - Subscribes to /joint_states_raw  (raw Ignition bridge output)
  - Strips the 5 mimic joint entries from the message
  - Republishes to /joint_states

RSP then computes each mimic joint position from grip_joint via the URDF <mimic>
relationship, so the fingers visually open/close correctly in RViz and Gazebo
without any physics controller on the fragile (1e-7 kg.m²) finger links.

Mimic multipliers (from URDF):
  rlink_joint2  = grip_joint × -1
  rlink_joint3  = grip_joint × +1
  llink_joint1  = grip_joint × -1
  llink_joint2  = grip_joint × +1
  llink_joint3  = grip_joint × -1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


# Joint names that are pure kinematic followers — strip them so RSP computes
# their positions via the URDF <mimic> relationship from grip_joint.
MIMIC_JOINTS = {
    'rlink_joint2',
    'rlink_joint3',
    'llink_joint1',
    'llink_joint2',
    'llink_joint3',
}


class GripperMimicRelay(Node):
    def __init__(self):
        super().__init__('gripper_mimic_relay')

        # Use SensorDataQoS (BEST_EFFORT, KEEP_LAST 5) on both ends so the
        # 200 Hz Ignition stream isn't queued up / dropped under load and RViz
        # gets the freshest joint states with minimal latency — this is what
        # makes the arm look smooth in RViz.
        self.pub = self.create_publisher(
            JointState, '/joint_states', qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self._callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Gripper mimic relay active: /joint_states_raw → /joint_states '
            f'(stripping {len(MIMIC_JOINTS)} mimic joints so RSP computes them via URDF)'
        )

    def _callback(self, msg: JointState):
        out = JointState()
        out.header = msg.header

        for i, name in enumerate(msg.name):
            if name in MIMIC_JOINTS:
                continue  # skip — RSP will compute from grip_joint via <mimic>
            out.name.append(name)
            if i < len(msg.position):
                out.position.append(msg.position[i])
            if i < len(msg.velocity):
                out.velocity.append(msg.velocity[i])
            if i < len(msg.effort):
                out.effort.append(msg.effort[i])

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
