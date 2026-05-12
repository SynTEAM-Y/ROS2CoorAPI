#!/usr/bin/env python3
"""
Gripper Mimic Joint Bridge

Ignition Gazebo Fortress does NOT enforce URDF <mimic>. This node bridges the
gap two ways:

1. /joint_states_raw  ->  /joint_states
   Strips the 5 mimic finger joints (llink_joint1-3, rlink_joint2-3) from the
   raw Ignition joint_states so robot_state_publisher computes them from
   grip_joint via the URDF <mimic> tag. This makes the gripper move correctly
   in RViz / TF.

2. /grip_joint_cmd_pos  ->  /<mimic>_cmd_pos x5
   Fans out the gripper command to a JointPositionController on each mimic
   joint (multiplier baked in) so the fingers also physically open/close in
   Gazebo, not just visually in RViz. Without this the URDF <mimic> tag is
   ignored by Ignition physics and the fingers stay frozen in Gazebo.

Mimic multipliers (from URDF):
  llink_joint1  = grip_joint x -1
  llink_joint2  = grip_joint x +1
  llink_joint3  = grip_joint x -1
  rlink_joint2  = grip_joint x -1
  rlink_joint3  = grip_joint x +1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# Mimic joint -> URDF multiplier (must match yahboomcar_X3plus.urdf.xacro).
MIMIC_MULTIPLIERS = {
    'llink_joint1': -1.0,
    'llink_joint2': +1.0,
    'llink_joint3': -1.0,
    'rlink_joint2': -1.0,
    'rlink_joint3': +1.0,
}


class GripperMimicRelay(Node):
    def __init__(self):
        super().__init__('gripper_mimic_relay')

        # ---- 1) joint_states filter (so RSP/RViz uses URDF mimic) ----
        self.pub_js = self.create_publisher(
            JointState, '/joint_states', qos_profile_sensor_data
        )
        self.sub_js = self.create_subscription(
            JointState, '/joint_states_raw', self._js_callback,
            qos_profile_sensor_data,
        )

        # ---- 2) grip_joint command fan-out (so Gazebo physics fingers move) ----
        # Master grip_joint controller subscribes to GZ topic /grip_master_target
        # (renamed in URDF to break the input/output cycle on /grip_joint_cmd_pos).
        self._master_pub = self.create_publisher(Float64, '/grip_master_target', 10)
        self._mimic_pubs = {
            name: self.create_publisher(Float64, f'/{name}_cmd_pos', 10)
            for name in MIMIC_MULTIPLIERS
        }
        self.sub_grip_cmd = self.create_subscription(
            Float64, '/grip_joint_cmd_pos', self._grip_cmd_callback, 10
        )
        # Rate-limited target ramp: avoids step inputs that cause PID jolt + detach.
        self._target = 0.0      # latest user setpoint
        self._current = 0.0     # currently published (ramped) setpoint
        self._rate = 0.6        # rad/s ramp speed (~1 s for full open)
        self._dt = 0.02         # 50 Hz publish
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            'Gripper mimic relay active: '
            '/joint_states_raw -> /joint_states (filter) and '
            '/grip_joint_cmd_pos -> %d mimic _cmd_pos topics (fan-out)'
            % len(self._mimic_pubs)
        )

    def _js_callback(self, msg: JointState):
        # Strip the 5 mimic finger joints from the raw Ignition joint_states.
        # robot_state_publisher then computes those finger positions from the
        # actual gripper joint and the URDF <mimic> relationships.
        filtered = JointState()
        filtered.header = msg.header
        filtered.name = []
        filtered.position = []
        filtered.velocity = []
        filtered.effort = []

        for idx, name in enumerate(msg.name):
            if name in MIMIC_MULTIPLIERS:
                continue
            filtered.name.append(name)
            if idx < len(msg.position):
                filtered.position.append(msg.position[idx])
            if idx < len(msg.velocity):
                filtered.velocity.append(msg.velocity[idx])
            if idx < len(msg.effort):
                filtered.effort.append(msg.effort[idx])

        self.pub_js.publish(filtered)

    def _grip_cmd_callback(self, msg: Float64):
        # Just record latest target; ramping happens in _tick.
        self._target = float(msg.data)

    def _tick(self):
        # Move _current toward _target by at most rate*dt per tick.
        max_step = self._rate * self._dt
        delta = self._target - self._current
        if delta > max_step:
            self._current += max_step
        elif delta < -max_step:
            self._current -= max_step
        else:
            self._current = self._target
        # Publish smoothed value to the master joint AND every mimic.
        master_msg = Float64()
        master_msg.data = self._current
        self._master_pub.publish(master_msg)
        for name, mult in MIMIC_MULTIPLIERS.items():
            out = Float64()
            out.data = self._current * mult
            self._mimic_pubs[name].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GripperMimicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
