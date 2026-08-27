#!/usr/bin/env python3
"""
Gripper Mimic Joint Bridge for Gazebo Sim 10 (Ionic) + ROS 2 Lyrical.

Gazebo Sim 10 does NOT enforce URDF <mimic>. This node bridges the gap
two ways:

1. /joint_states_raw  ->  /joint_states
   Strips the 5 mimic finger joints (llink_joint1-3, rlink_joint2-3) from
   the raw Gazebo joint_states so robot_state_publisher computes them from
   grip_joint via the URDF <mimic> tag. This makes the gripper move correctly
   in RViz / TF.

2. /grip_joint_cmd_pos  ->  /<mimic>_cmd_pos x5
   Fans out the gripper command to a JointPositionController on each mimic
   joint (multiplier baked in) so the fingers also physically open/close in
   Gazebo, not just visually in RViz. Without this the URDF <mimic> tag is
   ignored by Gazebo physics and the fingers stay frozen.

Mimic multipliers (from URDF):
  llink_joint1  = grip_joint x -1
  llink_joint2  = grip_joint x +1  (parallelogram: same sign as crank → pad stays parallel)
  llink_joint3  = grip_joint x -1
  rlink_joint2  = grip_joint x -1  (parallelogram: opposite sign to crank → pad stays parallel)
  rlink_joint3  = grip_joint x +1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# Mimic joint -> URDF multiplier (must match yahboomcar_X3plus.urdf.xacro).
# From the URDF (xacro file), the mimic relationships are:
#   r_joint2:  mimic=-1  (r link2 = -grip_joint)  [pad counter-rotates → net orientation 0]
#   l_joint1:  mimic=-1  (l link1 = -grip_joint)  [left crank mirrors right]
#   l_joint2:  mimic=+1  (l link2 =  grip_joint)  [left pad counter-rotates]
#   l_joint3:  mimic=-1  (l link3 = -grip_joint)  [left rocker mirrors]
#   r_joint3:  mimic=+1  (r link3 =  grip_joint)  [right rocker parallel to crank]
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

        # Optional namespace for multi-robot operation.  When empty the node
        # behaves exactly like the original single-robot version.  When set to
        # e.g. "robot1" it listens/publishes on /robot1/joint_states,
        # /robot1/grip_joint_cmd_pos, etc.
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').value
        self.ns = ns.rstrip('/') if ns else ''
        self._prefix = f'/{self.ns}' if self.ns else ''

        # ---- 1) joint_states filter (so RSP/RViz uses URDF mimic) ----
        self.pub_js = self.create_publisher(
            JointState, f'{self._prefix}/joint_states', qos_profile_sensor_data
        )
        self.sub_js = self.create_subscription(
            JointState, f'{self._prefix}/joint_states_raw', self._js_callback,
            qos_profile_sensor_data,
        )

        # ---- 2) grip_joint command fan-out (so Gazebo physics fingers move) ----
        # Master grip_joint controller subscribes to GZ topic /grip_master_target
        # (renamed in URDF to break the input/output cycle on /grip_joint_cmd_pos).
        self._master_pub = self.create_publisher(
            Float64, f'{self._prefix}/grip_master_target', 10)
        self._mimic_pubs = {
            name: self.create_publisher(
                Float64, f'{self._prefix}/{name}_cmd_pos', 10)
            for name in MIMIC_MULTIPLIERS
        }
        self.sub_grip_cmd = self.create_subscription(
            Float64, f'{self._prefix}/grip_joint_cmd_pos',
            self._grip_cmd_callback, 10
        )
        # Rate-limited target ramp: 5 rad/s (≈0.3 s for full open) is fast
        # enough that the gripper pads reach target in well under one
        # arm-pose segment, but smooth enough to avoid the step-input PID
        # jolt on the JointPositionController.  Slow ramps (≤0.5 rad/s) in
        # combination with the controller's high p_gain cause a 1-2 px
        # "buzz" in the wrist camera as the reference creeps into the
        # controller's dead zone.
        #
        # Init at 0.0 (CLOSED) — matches the manufacturer's URDF spawn pose.
        # The gripper starts with pads together, ready to receive a cube.
        self._target = 0.0       # latest user setpoint (CLOSED at startup)
        self._current = 0.0      # currently published (ramped) setpoint
        self._rate = 5.0         # rad/s ramp speed (~0.3 s for full open)
        self._dt = 0.02          # 50 Hz publish
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            'Gripper mimic relay active [%s]: '
            '%s/joint_states_raw -> %s/joint_states (filter) and '
            '%s/grip_joint_cmd_pos -> %d mimic _cmd_pos topics (fan-out)'
            % (self.ns or 'global',
               self._prefix or '', self._prefix or '',
               self._prefix or '', len(self._mimic_pubs))
        )

    def _js_callback(self, msg: JointState):
        # Strip the 5 mimic finger joints from the raw Ignition joint_states.
        # robot_state_publisher then computes those finger positions from the
        # actual gripper joint and the URDF <mimic> relationships.
        filtered = JointState()
        # The raw Ignition Model->JointState bridge leaves header.stamp at 0,
        # which makes robot_state_publisher emit zero-stamped TF frames and
        # breaks later lookups.  Stamp with the relay's current clock time.
        filtered.header.stamp = self.get_clock().now().to_msg()
        filtered.name = []
        filtered.position = []
        filtered.velocity = []
        filtered.effort = []
        ns_prefix = f'{self.ns}_' if self.ns else ''

        for idx, name in enumerate(msg.name):
            # Multi-robot: joint names are prefixed (e.g. robot_1_llink_joint1).
            # Strip the robot namespace so mimic filtering works for any robot.
            bare_name = name[len(ns_prefix):] if ns_prefix and name.startswith(ns_prefix) else name
            if bare_name in MIMIC_MULTIPLIERS:
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
