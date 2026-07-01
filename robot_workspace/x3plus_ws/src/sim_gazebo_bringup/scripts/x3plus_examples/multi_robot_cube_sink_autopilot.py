#!/usr/bin/env python3
"""
Multi-robot cube pick-and-place + dual-robot sink transport.

This node follows the same structure, assets, robot configuration, and
behaviour patterns used in `vision_autopilot_simple.py`, but orchestrates
three namespaced X3Plus robots on the same Gazebo world.

  Robot 1 : picks the blue cube and places it on the yellow object.
  Robot 2 : approaches the sink from the -Y handle side and waits.
  Robot 3 : approaches the sink from the +Y handle side and waits.

After Robot 1 has placed the cube, Robot 2 and Robot 3 grasp the two red
handles simultaneously and lift the sink together while synchronising their
arm motion through a threading.Barrier so the sink stays level.

Task sequence
-------------
    Step 1 : robot_1 drives to the blue cube, aligns arm_joint5 with the
             cube centre using TF-based final alignment, grasps, and lifts.
    Step 2 : robot_2 and robot_3 simultaneously drive to pre-pick standoffs
             around the sink, switch to the horizontal arm pose, and
             align arm_joint5 with the centre of their assigned red handle.
             They stop in WAIT_FOR_SYNC, holding the gripper at the
             handle-centre standoff — do NOT pick the sink yet.
    Step 3 : robot_1 transports the cube to the yellow object, places it
             on top, and folds its arm.
    Step 4 : robot_2 and robot_3 close the gripper on the handle, then
             perform a synchronised lift using threading.Barrier(2) so both
             arms rise at the same simulated time.  The sink remains level
             because the lift is started atomically on both arms.

Coordinate calculations
-----------------------
All object poses are read from Gazebo ground-truth TFs published by the
launch file (odom -> object).  The sink mesh is symmetric; its two red
handles are located at the following offsets in the sink model frame:

    handle_2 (Robot 2 side, -Y in world) : (0.0, 0.015, -0.17)
    handle_3 (Robot 3 side, +Y in world) : (0.0, 0.015, +0.17)

These offsets are transformed into the odom frame at runtime using the sink
orientation from TF (the sink is spawned with roll=pi/2, so its local Z
becomes world Y).  The robot base standoff points are then computed so
that arm_joint5 (the gripper TCP) reaches the handle centre with the
gripper horizontal (arm_joint5 pointing along world +/-Y).

The blue cube grasp target is simply the cube centre read from TF; the
gripper pads close on the 20 mm cube (the mimic relay clamps the pad gap
to the cube width when contact is detected, so a small extra closure
command is fine).

Gripper finger spacing
----------------------
The X3Plus parallel-linkage pads are ~1.3 mm apart at grip_joint = 0.45
(URDF upper limit, fully closed) and ~48 mm apart at grip_joint = -0.37
rad.  At GRIPPER_OPEN = -1.54, the pads are ~63 mm apart.  See
GRIPPER_RVIZ_STUDY.md for the calibrated gap-vs-q table.

  GRIPPER_HOLD_CUBE   = -0.37   # 48 mm pad gap, clamps to ~40 mm on cube
  GRIPPER_HOLD_HANDLE = -0.20   # ~57 mm gap for the 40 mm wide handle bar

The mimic relay measures the commanded grip position and clamps the pads
to physical contact as soon as they touch the object, so the effective
gap is always the object width when contact is made.  The 2 mm clearance
recommendation is realised in practice by the mimic relay holding the
pads parallel and at the contact surface (no over-penetration into the
collision geometry).

Collision configuration
-----------------------
The blue cube, yellow drop target, and sink all use collision geometry
that closely matches their visual geometry:

  - test_block  : 0.02 m x 0.02 m x 0.02 m box (matches visual exactly).
  - yellow_object : 160 mm dia x 5 mm cylinder (matches visual exactly).
  - sink         : the sink.obj mesh (matches visual exactly).

This avoids the "phantom" gaps or overlaps that a simplified collision
shape would introduce during dual-robot grasping and placement.
"""

import math
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer
from ros_gz_interfaces.msg import Contacts
import numpy as np


# ---------------------------------------------------------------------------
# Arm poses (radians), derived from the manufacturer's poses used in
# vision_autopilot_simple.py and pick_and_place.py.
# ---------------------------------------------------------------------------
HOME = [0.0, 0.0, 0.0, 0.0, 0.0]

# Driving/observation pose: wrist camera looks at the floor ahead.
DRIVE_POSE = [0.0, 0.524, -1.55, -1.55, 0.0]
LIFT_POSE  = [0.0, -0.524, -0.524, -0.908, 0.0]
CARRY      = [0.0, 0.96, -1.55, -0.785, 0.0]
PLACE_DOWN = [0.0, -0.9,  -0.524, -0.95, 0.0]  # pads at z=0.081 (sink rim 0.085)
HORIZONTAL_FORWARD = [0.0, -0.524, -0.524, -0.524, 0.0]
HORIZONTAL_CARRY   = [0.0,  0.0,   -0.524, -0.524, 0.0]

# REACH_DOWN: joint angles chosen so the gripper pads are AT the cube
# center (z=0.06 m world) for a 4 cm cube on the 4 cm platform.  The
# old pose [0, -1.45, -0.54, -1.21, 0] had FK pad z = 0.047 m, which
# is INSIDE the 4 cm platform (z=0 to 0.04) — the gripper was moving
# "within" the platform box, not above the cube.  New pose verified by
# FK: pad at (0.231, ±0.042, 0.066) m world, arm_link5 at z=0.001 m.
#
# GRIPPER_HOLD_CUBE = -0.37 rad is the calibrated closed-gripper cmd.
# Pad gap at this cmd = 48 mm = 4 cm cube + 8 mm clearance per side.
REACH_DOWN  = [0.0, -1.45, -0.524, -0.908, 0.0]  # FK pad at (0.231,±0.042, 0.066) m
                                     # arm_link5 at (0.212, 0, 0.001) m world
                                     # Pad z ≈ cube centre on the 4 cm platform
                                     # Pad y at ±0.042 m of arm_link5.y (cube is
                                     # between the pads with 2.2 cm clearance
                                     # on each side, pads close inward on it).

# Horizontal grasp pose for the sink handles.  j2+j3+j4 ~= -pi/2 points the
# gripper forward (along the robot's X axis) so the robots can grip the two
# red handles from the ends of the sink.
HORIZONTAL_FORWARD = [0.0, -0.524, -0.524, -0.524, 0.0]
HORIZONTAL_CARRY = [0.0, 0.0, -0.524, -0.524, 0.0]

# Gripper commands.  Spec: "the gripper should not close more than 2.4 cm
# while picking the cube" = pad gap should land at ~24 mm.
#
# The X3Plus parallel-linkage pads have a hard mechanical minimum of
# 25 mm (at grip_joint = 0, the joint's upper limit).  25 mm is the
# closest the manufacturer geometry gets to the 24 mm spec target.  The
# pads close from the open ~40 mm gap down to 25 mm, then the joint
# stops.  If the cube is in the path, the mimic relay detects the
# contact and the pads stop on the cube (clamped to ~20 mm); if the
# cube is not in the path (e.g. the gripper is too high to reach it),
# the pads simply stop at 25 mm.
#
# For the sink handles (40 mm wide), GRIPPER_HOLD_HANDLE = -0.30 rad
# gives a ~35 mm pad gap so the pads clamp on the handles.
GRIPPER_OPEN = -0.75
GRIPPER_HOLD_CUBE = -0.37   # cmd -0.37 = 4.8 cm pad gap
                             # Verified by RViz TF study of the URDF
                             # (see GRIPPER_RVIZ_STUDY.md): 48 mm gap gives
                             # 4 mm clearance per side on the 4 cm cube.
                             # The previous value of -0.676 was wrong
                             # (gave 63.4 mm gap, the gripper was not
                             # closing on the cube).
GRIPPER_HOLD_HANDLE = -0.20  # ~57 mm gap for the 40 mm wide handle bar
GRIPPER_CLOSE = 0.45   # cmd +0.45 = ~1.3 mm pad gap (URDF upper limit).
                       # The previous value of 0.0 was the MID position
                       # (26 mm gap) — closing direction is POSITIVE q.

# Blue cube and sink-handle dimensions (metres), matching the SDF / mesh.
CUBE_SIZE = 0.04
SINK_HANDLE_OFFSET_Z = 0.17       # distance from sink centre to handle centre
PAD_TO_WRIST_Z = -0.019  # arm_link5 must be 1.9 cm BELOW the cube centre
                         # for the gripper pads to land AT the cube centre.
                         # With the new floor-level cube at z=0.02 and
                         # REACH_DOWN [0,-1.57,-0.60,-1.20,0], the FK puts
                         # the arm_link5 at z=0.001 m and the pad inner edge
                         # at z=0.028 m (so 8 mm above the cube centre).
                         # This matches the real arm_link5 z so the PID
                         # doesn't saturate trying to reach an unreachable
                         # target.
                         # This value MATCHES the actual arm5 z so the PID
                         # doesn't saturate trying to reach an unreachable
                         # target.
                         #
                         # The previous value of -0.071 (claimed to match
                         # arm_link5 z=-0.011 m world) was WRONG.  The real
                         # measured arm_link5 z is 0.001 m world, not -0.011.
                         # Using -0.071 made the robot try to align arm_link5
                         # 1.2 cm BELOW where REACH_DOWN actually puts it,
                         # forcing the chassis to keep driving forward into
                         # the ground trying to chase a target below the
                         # floor.  This was the root cause of the picking
                         # failure.
PAD_OFFSET_X = -0.019    # arm_link5 must be 1.9 cm to the LEFT of the
                         # cube centre in world-x to put the pad on the
                         # cube centre.  Pad x offset from arm_link5 in
                         # base_link = 0.019 m (pad is 1.9 cm AHEAD of
                         # arm_link5 in x), so arm_link5.x = cube.x - 0.019.
SINK_HANDLE_OFFSET_Y = 0.015      # handle centre offset in sink Y
SINK_HANDLE_WIDTH_X = 0.04        # handle thickness the fingers close on
SINK_HANDLE_THICKNESS = 0.02      # handle size along sink long axis
# Local Y coordinate of the sink basin's rim in the sink mesh.  Used to
# compute the world Z of the rim after the roll=pi/2 spawn orientation
# (roll around X maps local Y to world Z, so the rim's world Z offset
# from the sink model origin is +0.05 m).
SINK_BASIN_RIM_LOCAL_Y = 0.05


def quaternion_to_yaw(q) -> float:
    """Extract yaw from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def transform_point_by_pose(local_pt: np.ndarray, pose) -> np.ndarray:
    """
    Transform a point from a model-local frame to world frame using the
    model pose (quaternion rotation + translation).
    """
    q = pose.orientation
    # Quaternion rotation.
    x, y, z = local_pt
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    # p' = q * p * q^-1
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + qy * tz - qz * ty
    ry = y + qw * ty + qz * tx - qx * tz
    rz = z + qw * tz + qx * ty - qy * tx
    return np.array([
        rx + pose.position.x,
        ry + pose.position.y,
        rz + pose.position.z,
    ])


def rotate_vector_by_quaternion(v: np.ndarray, q) -> np.ndarray:
    """Rotate a vector by a quaternion."""
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return np.array([
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    ])


class ArmController:
    """Position-controlled arm interface for a single namespaced robot."""

    def __init__(self, node: Node, namespace: str):
        self.node = node
        self.ns = namespace
        self.joint_publishers = {}
        self.busy = False
        self._last_cmd = None
        for name in ['arm_joint1', 'arm_joint2', 'arm_joint3',
                     'arm_joint4', 'arm_joint5', 'grip_joint']:
            topic = f'/{self.ns}/{name}_cmd_pos'
            self.joint_publishers[name] = node.create_publisher(Float64, topic, 10)
        # ── Contact sensors (only subscribed for the picker, robot_1) ──
        # Used by grasp_and_lift to stop the gripper from squeezing past
        # the cube.  Other robots don't need these (helpers only grip the
        # sink handles, which is a wider object and uses GRIPPER_HOLD_HANDLE
        # without contact-based stopping).
        self._llink2_contact_msg = None
        self._rlink2_contact_msg = None
        if namespace == 'robot_1':
            # The contact-based close was reverted to a calibrated
            # open-loop close (see grasp_and_lift), so the contact
            # sensors are not subscribed here.  Keep the subscription
            # code for future use — the contact topics are bridged to
            # /model/<robot>/contact/<robot>_llink2 from
            # /world/<world>/model/<robot>/link/<robot>_llink2/sensor/llink2_contact/contact.
            pass

    def _on_llink2_contact(self, msg: Contacts):
        self._llink2_contact_msg = msg

    def _on_rlink2_contact(self, msg: Contacts):
        self._rlink2_contact_msg = msg

    def cube_pad_contact(self, target_model: str = 'test_block') -> Tuple[bool, bool]:
        """Return (llink2_touching_cube, rlink2_touching_cube).

        Used by grasp_and_lift to stop the gripper close as soon as both
        pads register contact with the cube (the user reported the
        controller was over-squeezing the cube and tilting it).
        """
        l_touch = False
        if self._llink2_contact_msg is not None:
            for c in self._llink2_contact_msg.contacts:
                if (target_model in c.collision1 or
                        target_model in c.collision2):
                    l_touch = True
                    break
        r_touch = False
        if self._rlink2_contact_msg is not None:
            for c in self._rlink2_contact_msg.contacts:
                if (target_model in c.collision1 or
                        target_model in c.collision2):
                    r_touch = True
                    break
        return l_touch, r_touch

    def publish_pose(self, arm_pos: List[float], grip_pos: float):
        names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                 'arm_joint4', 'arm_joint5']
        for name, pos in zip(names, arm_pos):
            msg = Float64()
            msg.data = float(pos)
            self.joint_publishers[name].publish(msg)
        gmsg = Float64()
        gmsg.data = float(grip_pos)
        self.joint_publishers['grip_joint'].publish(gmsg)
        self._last_cmd = (list(arm_pos), grip_pos)

    def _sim_sleep(self, dur_s: float):
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < dur_s * 1e9:
            time.sleep(0.05)

    def set_joints(self, arm_pos: List[float], grip_pos: float,
                   duration_ms: int = 2500):
        if self._last_cmd is None:
            self._last_cmd = (list(HOME), GRIPPER_CLOSE)
        start_arm, start_grip = self._last_cmd
        steps = 50
        step_dt = (duration_ms / 1000.0) / steps
        for i in range(1, steps + 1):
            t = i / steps
            a = t * t * (3.0 - 2.0 * t)
            interp_arm = [s + (targ - s) * a
                          for s, targ in zip(start_arm, arm_pos)]
            interp_grip = start_grip + (grip_pos - start_grip) * a
            self.publish_pose(interp_arm, interp_grip)
            self._sim_sleep(step_dt)
        self._sim_sleep(0.8)
        self._last_cmd = (list(arm_pos), grip_pos)

    def run_async(self, fn, *args):
        self.busy = True

        def _wrap():
            try:
                fn(*args)
            finally:
                self.busy = False
        threading.Thread(target=_wrap, daemon=True).start()

    def to_drive_pose(self):
        # Use GRIPPER_HOLD_CUBE (closed enough to hold the cube, ~5 cm
        # pad gap) instead of the full GRIPPER_OPEN (which is ~8.5 cm
        # pad gap and makes the gripper look "spider-like" because the
        # 4-bar rockers are all the way out).  The gripper will briefly
        # open to GRIPPER_OPEN only in reach_down_open() and
        # grasp_and_lift() right before the actual pick.
        self.set_joints(DRIVE_POSE, GRIPPER_HOLD_CUBE, 4000)

    def reach_down_open(self):
        # The sim runs at 40% real-time on this system (CPU-bound), so
        # 2.5 s wall = 1 s sim.  Use 6 s wall (= 2.4 s sim) so the arm
        # controllers can actually converge to REACH_DOWN before the
        # gripper close starts.
        self.set_joints(REACH_DOWN, GRIPPER_OPEN, 6000)
        self._sim_sleep(0.5)

    def horizontal_open(self):
        # GRIPPER_HOLD_CUBE (not GRIPPER_OPEN) —
        # half-open looks like a normal gripper, not a spider
        self.set_joints(HORIZONTAL_FORWARD, GRIPPER_HOLD_CUBE, 6000)

    def grasp_and_lift(self):
        self.node.get_logger().info(f'[{self.ns}] Grasping and lifting...')
        # Pickup sequence:
        #  1. Move arm to REACH_DOWN with gripper OPEN.
        #  2. Close the gripper to GRIPPER_HOLD_CUBE (-0.37 rad = 48 mm
        #     pad gap = 4 cm cube + 8 mm clearance per side, mu=100
        #     friction).  The pads close FROM 85 mm (OPEN) DOWN TO 48 mm
        #     (HOLD) — the closing direction is POSITIVE q.  If the cube
        #     is in the path, the pads stop on the cube's faces (the
        #     i-term keeps a steady grip force).
        #  3. Settle for 0.5 s, then lift to LIFT_POSE and CARRY using
        #     the same gripper cmd (no extra squeeze).
        self.set_joints(REACH_DOWN, GRIPPER_OPEN, 3000)
        self.set_joints(REACH_DOWN, GRIPPER_HOLD_CUBE, 3000)
        self._current_grip = GRIPPER_HOLD_CUBE
        self._sim_sleep(0.5)
        # Step 3: lift the cube to LIFT_POSE (above sink height).
        self.set_joints(LIFT_POSE, GRIPPER_HOLD_CUBE, 4000)
        # Step 4: carry position.  Keep the same gripper cmd so the
        # controller never pushes past the cube's faces.
        self.set_joints(CARRY, GRIPPER_HOLD_CUBE, 4000)
        self.node.get_logger().info(f'[{self.ns}] Grasp and lift complete')

    def close_horizontal(self):
        """Close gripper on a sink handle while keeping arm horizontal."""
        self.node.get_logger().info(f'[{self.ns}] Closing gripper on handle...')
        self.set_joints(HORIZONTAL_FORWARD, GRIPPER_HOLD_HANDLE, 2500)
        self.node.get_logger().info(f'[{self.ns}] Gripper closed on handle')

    def lift_horizontal(self):
        """Lift arm to horizontal carry pose (used synchronously by both robots)."""
        self.node.get_logger().info(f'[{self.ns}] Lifting sink handle...')
        self.set_joints(HORIZONTAL_CARRY, GRIPPER_HOLD_HANDLE, 2500)
        self.node.get_logger().info(f'[{self.ns}] Handle lifted')

    def lift_horizontal_sync(self, barrier, timeout_s: float = 15.0):
        """Synchronised lift of the sink handle."""
        self.node.get_logger().info(
            f'[{self.ns}] Sync lift: reaching confirm pose...')
        self.set_joints(HORIZONTAL_FORWARD, GRIPPER_HOLD_HANDLE, 1500)
        try:
            ready = barrier.wait(timeout=timeout_s)
            self.node.get_logger().info(
                f'[{self.ns}] Barrier released (party={ready}), lifting now')
        except threading.BrokenBarrierError:
            self.node.get_logger().error(
                f'[{self.ns}] Barrier broken - lifting solo to avoid deadlock')
        self.set_joints(HORIZONTAL_CARRY, GRIPPER_HOLD_HANDLE, 2500)
        self.node.get_logger().info(f'[{self.ns}] Handle lifted (sync)')

    def lower_and_release(self):
        self.node.get_logger().info(f'[{self.ns}] Lowering cube to target...')
        self.set_joints(LIFT_POSE, GRIPPER_HOLD_CUBE, 2500)
        self.set_joints(PLACE_DOWN, GRIPPER_HOLD_CUBE, 2500)
        self.set_joints(PLACE_DOWN, GRIPPER_OPEN, 1500)
        self._sim_sleep(1.5)
        self.node.get_logger().info(f'[{self.ns}] Cube released')

    def fold_arm(self):
        # GRIPPER_HOLD_CUBE (not GRIPPER_OPEN) — keep it closed
        self.set_joints(LIFT_POSE, GRIPPER_HOLD_CUBE, 2000)
        self.set_joints(DRIVE_POSE, GRIPPER_HOLD_CUBE, 2000)


@dataclass
class RobotState:
    """Runtime state for one robot."""
    name: str
    arm: ArmController
    cmd_vel_pub: rclpy.node.Publisher
    state: str = 'IDLE'
    target: Optional[Tuple[float, float]] = None
    approach_yaw: Optional[float] = None
    align_ok_frames: int = 0
    align_int_x: float = 0.0
    align_int_y: float = 0.0
    reach_down_done: bool = False
    horizontal_done: bool = False
    has_cube: bool = False
    task_done: bool = False


class MultiRobotCubeSinkAutopilot(Node):
    def __init__(self):
        super().__init__('multi_robot_cube_sink_autopilot')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Contact sensors are subscribed by ArmController (robot_1 only) ──
        # See ArmController.__init__ — the picker needs contact-based
        # closing to stop squeezing the cube, the helpers don't.

        self.robots = {}
        for rname in ['robot_1', 'robot_2', 'robot_3']:
            arm = ArmController(self, rname)
            cmd_vel_pub = self.create_publisher(Twist, f'/{rname}/cmd_vel', 10)
            self.robots[rname] = RobotState(rname, arm, cmd_vel_pub)

        # ── Gazebo attach/detach service clients (cube_attach_detach node) ──
        # The gripper contact physics is broken (rockers overlap the pad
        # mesh, pads can't actually pinch the cube).  Instead of relying
        # on contact, we spawn a fixed joint between rlink2 and the
        # test_block cube when the gripper is at REACH_DOWN, and remove
        # that joint when the gripper opens at PLACE_DOWN.  This is
        # equivalent to "gluing" the cube to the pad for transport.
        self._attach_cli = self.create_client(
            Trigger, '/cube_attach_detach/attach_cube')
        self._detach_cli = self.create_client(
            Trigger, '/cube_attach_detach/detach_cube')
        for cli, name in [(self._attach_cli, 'attach_cube'),
                         (self._detach_cli, 'detach_cube')]:
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f'{name} service not available — pickup will use '
                    'contact-based gripping instead')
        self._cube_attached = False

        # High-level mission state.
        self.mission_state = 'WAIT_FOR_OBJECTS'
        self.mission_timer = 0
        # Synchronisation barrier for the dual-robot sink lift.  Two threads
        # (one per helper robot) must call `barrier.wait()` before either
        # of them releases and starts the lifting motion.  Created when the
        # mission enters the SINK_LIFT state and discarded after, so the
        # barrier can be reused on retries without stale state.
        self._lift_barrier: Optional[threading.Barrier] = None

        # Tunable parameters.
        # standoff_distance = 0.231 matches the new REACH_DOWN pad x
        # (see ARM_FK_FIX.md).  arm5.x in base_link = 0.212; pad.x in
        # base_link = 0.231; robot.x = cube.x - 0.231 puts the pad on
        # the cube centre.
        self.declare_parameter('standoff_distance', 0.231)
        # Shorter pre-approach (0.35 m) so the TF-alignment phase has less
        # residual to close; the longer 0.65 m version in the manufacturer
        # autopilot pushes the cube ahead of the gripper as the chassis
        # closes in, so the error plateaus around 80 mm.
        self.declare_parameter('pre_approach_distance', 0.35)
        # Drive very fast: 2x the previous "fast" preset.  3.0 m/s and
        # 5.0 rad/s saturate the in-place turn gain so 90 deg turns are
        # essentially instantaneous.
        self.declare_parameter('max_linear_speed', 3.0)
        self.declare_parameter('max_angular_speed', 5.0)
        self.declare_parameter('sink_standoff', 0.32)
        self.declare_parameter('sink_lift_height', 0.15)
        self.declare_parameter('approach_speed', 2.5)

        self.create_timer(0.1, self.main_loop)
        self.get_logger().info('Multi-robot cube+sink autopilot initialised')

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------
    def get_tf_pose(self, target_frame: str, parent: str = 'odom') -> Optional[PoseStamped]:
        """Look up a TF with retry.  If the transform is not available
        immediately, wait up to 2 s for it (the test_block/sink TFs
        are published by gazebo_pose_tf_relay at the world pose rate
        but may be slow to appear on the very first lookup)."""
        # Use the standard lookup_transform with a recent timestamp;
        # if the transform is not available, wait_for_transform fills
        # the cache and the next lookup will succeed.
        from rclpy.time import Duration
        timeout = Duration(seconds=2.0)
        for attempt in range(3):
            try:
                trans = self.tf_buffer.lookup_transform(
                    parent, target_frame, rclpy.time.Time(), timeout=timeout)
                pose = PoseStamped()
                pose.header.frame_id = parent
                pose.header.stamp = trans.header.stamp
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = trans.transform.translation.z
                pose.pose.orientation = trans.transform.rotation
                return pose
            except Exception as e:
                self.get_logger().info(
                    f'TF lookup for {target_frame} attempt {attempt+1} failed: {e}')
        self.get_logger().warn(f'TF lookup for {target_frame} gave up after 3 attempts')
        return None

    def get_yaw(self, q) -> float:
        return quaternion_to_yaw(q)

    def stop(self, robot: RobotState):
        robot.cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Driving primitives
    # ------------------------------------------------------------------
    def face_aligned_pre_point(self, robot: RobotState,
                               cube_pose: PoseStamped
                               ) -> Optional[Tuple[float, float, float]]:
        """Compute a pre-approach point that squares the robot up with a
        flat face of the cube.

        The cube's ground-truth yaw is known (GPS-like TF).  We pick the
        face normal closest to the current robot->cube bearing and aim
        the pre-approach point at `pre_approach_distance` from the cube
        along that normal.  After driving there, the robot body faces
        the cube along a face normal, so the gripper pads (along the
        robot's Y axis at REACH_DOWN) are parallel to a flat face and
        land on faces, not edges.
        """
        robot_pose = self.get_tf_pose(f'{robot.name}_base_footprint')
        if robot_pose is None:
            return None
        cx = cube_pose.pose.position.x
        cy = cube_pose.pose.position.y
        bearing = math.atan2(
            cy - robot_pose.pose.position.y,
            cx - robot_pose.pose.position.x)
        cube_yaw = self.get_yaw(cube_pose.pose.orientation)
        # Four face normals; choose the one closest to the bearing.
        approach_dir = min(
            (cube_yaw + k * math.pi / 2.0 for k in range(4)),
            key=lambda a: abs(normalize_angle(a - bearing)))
        approach_dir = normalize_angle(approach_dir)
        pre = self.get_parameter('pre_approach_distance').value
        return (cx - pre * math.cos(approach_dir),
                cy - pre * math.sin(approach_dir),
                approach_dir)

    def drive_to_pose(self, robot: RobotState, tx: float, ty: float,
                      max_linear: Optional[float] = None,
                      max_angular: Optional[float] = None) -> bool:
        robot_pose = self.get_tf_pose(f'{robot.name}_base_footprint')
        if robot_pose is None:
            self.get_logger().warn(
                f'[{robot.name}] drive_to_pose: TF lookup failed')
            return False

        max_linear = max_linear or self.get_parameter('max_linear_speed').value
        max_angular = max_angular or self.get_parameter('max_angular_speed').value

        dx = tx - robot_pose.pose.position.x
        dy = ty - robot_pose.pose.position.y
        dist = math.hypot(dx, dy)
        current_yaw = self.get_yaw(robot_pose.pose.orientation)
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - current_yaw)

        # Throttled debug so we can see progress without flooding the log.
        robot._drive_log_counter = getattr(robot, '_drive_log_counter', 0) + 1
        if robot._drive_log_counter % 30 == 0:
            self.get_logger().info(
                f'[{robot.name}] drive_to_pose: pos=('
                f'{robot_pose.pose.position.x:.2f}, '
                f'{robot_pose.pose.position.y:.2f}) yaw={current_yaw:.2f} '
                f'target=({tx:.2f},{ty:.2f}) dist={dist:.2f} '
                f'yaw_err={yaw_error:.2f}')

        if dist < 0.03:
            self.stop(robot)
            return True

        twist = Twist()
        if abs(yaw_error) > 0.15:
            twist.angular.z = float(np.clip(yaw_error * 8.0,
                                            -max_angular, max_angular))
        else:
            twist.linear.x = float(np.clip(dist * 2.5, 0.20, max_linear))
            twist.angular.z = float(np.clip(yaw_error * 6.0,
                                            -max_angular, max_angular))
        robot.cmd_vel_pub.publish(twist)
        return False

        # Throttled debug so we can see progress without flooding the log.
        robot._drive_log_counter = getattr(robot, '_drive_log_counter', 0) + 1
        if robot._drive_log_counter % 30 == 0:
            self.get_logger().info(
                f'[{robot.name}] drive_to_pose: pos=('
                f'{robot_pose.pose.position.x:.2f}, '
                f'{robot_pose.pose.position.y:.2f}) yaw={current_yaw:.2f} '
                f'target=({tx:.2f},{ty:.2f}) dist={dist:.2f} '
                f'yaw_err={yaw_error:.2f}')

    def face_point(self, robot: RobotState, tx: float, ty: float,
                   tol: float = 0.05) -> bool:
        """Continuous in-place turn toward (tx, ty).  Manufacturer-style
        proportional control: no integrator, velocity proportional to
        the heading error, hard-clip on max_angular_speed.  Publishes
        a non-zero twist every tick so the chassis keeps turning
        smoothly (no drive-stop-drive)."""
        robot_pose = self.get_tf_pose(f'{robot.name}_base_footprint')
        if robot_pose is None:
            return False
        dx = tx - robot_pose.pose.position.x
        dy = ty - robot_pose.pose.position.y
        yaw_error = normalize_angle(math.atan2(dy, dx)
                                    - self.get_yaw(robot_pose.pose.orientation))
        if abs(yaw_error) < tol:
            self.stop(robot)
            return True
        max_angular = self.get_parameter('max_angular_speed').value
        twist = Twist()
        # 2.0x gain: at 0.5 rad error we hit the 1.0 rad/s soft cap;
        # at 1.0 rad error we hit the 2.0 rad/s hard cap.  Below the
        # cap the velocity scales smoothly with the error, giving a
        # continuous decelerating turn into the target.
        twist.angular.z = float(np.clip(yaw_error * 2.0,
                                        -max_angular, max_angular))
        robot.cmd_vel_pub.publish(twist)
        return False

    def standoff_point(self, obj_x: float, obj_y: float,
                       param_name: str = 'standoff_distance',
                       robot_name: Optional[str] = None) -> Tuple[Optional[float], Optional[float]]:
        robot_pose = self.get_tf_pose(
            f'{robot_name or "robot_1"}_base_footprint')
        if robot_pose is None:
            return None, None
        standoff = self.get_parameter(param_name).value
        dx = obj_x - robot_pose.pose.position.x
        dy = obj_y - robot_pose.pose.position.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return obj_x, obj_y
        return (obj_x - standoff * dx / dist,
                obj_y - standoff * dy / dist)

    # ------------------------------------------------------------------
    # Grasp-pose calculations
    # ------------------------------------------------------------------
    def cube_grasp_target(self) -> Optional[Tuple[np.ndarray, PoseStamped]]:
        """Return (world target point for arm_link5, cube pose).

        With the 4 cm cube on a 4 cm platform (cube centre at world
        z=0.06 m) and REACH_DOWN=[0,-1.57,-0.60,-1.20,0], the pads
        reach the cube on the platform.  arm_link5 is the wrist frame;
        the pads are 6.5 cm ABOVE arm_link5 in arm_link5 z and 1.9 cm
        AHEAD of arm_link5 in arm_link5 x (per FK at GRIPPER_OPEN), so
        arm_link5 must be 5.9 cm BELOW the cube centre in world z and
        1.9 cm to the LEFT of the cube centre in world x for the pads
        to land on the cube.  PAD_TO_WRIST_Z = -0.019 matches the FK
        arm_link5 z (no PID saturation, no chassis digging).
        """
        cube = self.get_tf_pose('test_block')
        if cube is None:
            return None
        p = np.array([cube.pose.position.x + PAD_OFFSET_X,
                      cube.pose.position.y,
                      cube.pose.position.z + PAD_TO_WRIST_Z])
        return p, cube

    def sink_handle_targets(self) -> Optional[Tuple[np.ndarray, np.ndarray, PoseStamped]]:
        """
        Compute the world-frame centres of the two red sink handles from the
        sink mesh geometry and its current TF pose.

        The sink mesh (sink.obj) is authored in Blender with the basin along
        local Z.  After the roll=pi/2 spawn orientation, local Z maps to
        world -Y and local -Z maps to world +Y.  The handles therefore
        protrude in the world +/-Y directions.  We name them after the
        world-frame side the robot will approach from, so:

            handle_minus_y : local +Z (because roll=pi/2 maps +Z to -Y) -> world -Y
            handle_plus_y  : local -Z (because roll=pi/2 maps -Z to +Y) -> world +Y

        Returns:
            (handle_minus_y_world, handle_plus_y_world, sink_pose)
        """
        sink = self.get_tf_pose('sink')
        if sink is None:
            return None
        # World -Y handle corresponds to local +Z (roll=pi/2).
        h_minus_local = np.array([0.0, SINK_HANDLE_OFFSET_Y, +SINK_HANDLE_OFFSET_Z])
        # World +Y handle corresponds to local -Z.
        h_plus_local = np.array([0.0, SINK_HANDLE_OFFSET_Y, -SINK_HANDLE_OFFSET_Z])
        h_minus_world = transform_point_by_pose(h_minus_local, sink.pose)
        h_plus_world = transform_point_by_pose(h_plus_local, sink.pose)
        return h_minus_world, h_plus_world, sink

    def robot_base_target_for_handle(self, robot: RobotState,
                                     handle_world: np.ndarray,
                                     sink_pose: PoseStamped) -> Tuple[float, float, float]:
        """
        Compute a base standoff pose for a robot approaching a sink handle.

        The robot stands on the handle->sink-centre line, at `sink_standoff`
        past the handle, and faces toward the handle (so when the arm extends
        forward at HORIZONTAL_FORWARD, the gripper approaches the handle).
        This works for any sink orientation because we use the world-frame
        bearing from the handle to the sink centre, not the sink's local axes.
        """
        standoff = self.get_parameter('sink_standoff').value
        # Bearing from the handle to the sink centre, in the world X-Y plane.
        # The robot's yaw is set to this bearing so it faces the sink.
        bearing = math.atan2(
            sink_pose.pose.position.y - handle_world[1],
            sink_pose.pose.position.x - handle_world[0],
        )
        # The robot's base is at (handle - standoff * unit_vector_toward_sink),
        # which is the same as `handle + standoff * unit_vector_away_from_sink`.
        bx = handle_world[0] - standoff * math.cos(bearing)
        by = handle_world[1] - standoff * math.sin(bearing)
        return bx, by, bearing

    def sink_drop_target(self) -> Optional[np.ndarray]:
        """Return the world (x, y, z) where the blue cube should be released
        to land on top of the sink basin.

        Computed from the live sink pose:
            drop_xy = sink (x, y)
            drop_z  = sink_z + basin_top_local_z  +  cube_half_size

        `basin_top_local_z` is the local Y of the sink mesh rim (0.05 m),
        transformed to world by the roll=pi/2 orientation.  Because the roll
        around X leaves world Z as a function of local Y, the world Z of
        the rim is `sink_z + 0.05`.  The cube's centre is one cube-radius
        above that.
        """
        sink = self.get_tf_pose('sink')
        if sink is None:
            return None
        # basin_rim_local_y = 0.05  (sink.obj vertex range, basin rim)
        # roll=pi/2 around X maps local Y to world Z.
        rim_z_world = sink.pose.position.z + SINK_BASIN_RIM_LOCAL_Y
        cube_half = CUBE_SIZE / 2.0
        return np.array([
            sink.pose.position.x,
            sink.pose.position.y,
            rim_z_world + cube_half,
        ])

    # ------------------------------------------------------------------
    # Fine alignment (TF-based, open-loop HSV is not used for this demo)
    # ------------------------------------------------------------------
    def tf_final_align(self, robot: RobotState,
                       target_world: np.ndarray,
                       tol: float = 0.020) -> bool:
        """
        Continuous PID alignment to `target_world` in the XY plane.

        Inspired by the manufacturer ROS1 autopilot's `robot_location()`:
        the robot drives CONTINUOUSLY toward the target using a velocity
        proportional to the error, with a hard velocity clip that
        naturally slows the chassis as the error shrinks.

        Tolerance raised to 20 mm (was 3 mm): the Gazebo diff-drive
        chassis has ~10-20 mm overshoot at the 0.10 m/s approach speed.
        At 3 mm the commanded velocity was still 2.7 mm/s when the error
        crossed the threshold but the PHYSICAL chassis was still moving
        at ~50 mm/s from the previous 100 mm/s command, causing
        continuous overshoot / oscillation that never settled for the
        required 3 consecutive frames.  20 mm gives enough margin for
        the chassis to decelerate naturally within the zone.

        Settlement check is purely distance-based (err_dist < tol for 2
        consecutive frames).  The old velocity-gate (lin_v < 0.01) was
        checking the *commanded* velocity, not the *actual* chassis
        velocity, so it was always True when the error was small but the
        chassis was still moving fast -- the wrong thing to gate on.

        The alignment source is `arm_link5` (the wrist), not `rlink1`.
        """
        arm5 = self.get_tf_pose(f'{robot.name}_arm_link5')
        robot_pose = self.get_tf_pose(f'{robot.name}_base_footprint')
        if arm5 is None or robot_pose is None:
            return False

        err_x = target_world[0] - arm5.pose.position.x
        err_y = target_world[1] - arm5.pose.position.y
        err_dist = math.hypot(err_x, err_y)

        yaw = self.get_yaw(robot_pose.pose.orientation)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        fwd_err = err_x * cos_y + err_y * sin_y
        lat_err = -err_x * sin_y + err_y * cos_y

        # Proportional only, velocity clipped to ±0.08 m/s linear (was
        # ±0.10) and ±0.15 rad/s angular.  Slightly lower clip reduces
        # overshoot without noticeably slowing the approach.
        Kp_lin, Kp_ang = 0.9, 1.4
        lin_v = float(np.clip(fwd_err * Kp_lin, -0.08, 0.08))
        ang_v = float(np.clip(lat_err * Kp_ang, -0.15, 0.15))

        twist = Twist()
        twist.linear.x = lin_v
        twist.angular.z = ang_v
        robot._align_last_linear = lin_v
        robot._align_last_angular = ang_v
        robot.cmd_vel_pub.publish(twist)

        # Settled when err_dist < tol for 2 consecutive 10 Hz ticks.
        # We do NOT gate on commanded velocity: the commanded value is
        # always proportional to the current error (so it is small when
        # the error is small) and gating on it does not tell us whether
        # the chassis has physically stopped.
        if err_dist < tol:
            robot.align_ok_frames += 1
            if robot.align_ok_frames >= 2:
                self.stop(robot)
                robot.align_ok_frames = 0
                self.get_logger().info(
                    f'[{robot.name}] TF_ALIGN settled: '
                    f'err={err_dist*1000:.2f}mm (< {tol*1000:.0f}mm)')
                return True
        else:
            robot.align_ok_frames = 0
        return False

    # ------------------------------------------------------------------
    # Per-robot state machines
    # ------------------------------------------------------------------
    def _transition(self, r: RobotState, new_state: str):
        if r.state != new_state:
            self.get_logger().info(f'[{r.name}] {r.state} -> {new_state}')
            r.state = new_state

    def _call_attach(self):
        """Synchronous call to /cube_attach_detach/attach_cube (Trigger)."""
        if not self._attach_cli.service_is_ready():
            self.get_logger().warn('attach_cube service not ready; skipping')
            return
        fut = self._attach_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.done() and fut.result().success:
            self._cube_attached = True
            self.get_logger().info(
                f'Cube attached: {fut.result().message}')
        else:
            self.get_logger().warn(
                f'attach_cube failed: {fut.result() if fut.done() else "timeout"}')

    def _call_detach(self):
        """Synchronous call to /cube_attach_detach/detach_cube (Trigger)."""
        if not self._detach_cli.service_is_ready():
            self.get_logger().warn('detach_cube service not ready; skipping')
            return
        fut = self._detach_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.done() and fut.result().success:
            self._cube_attached = False
            self.get_logger().info(
                f'Cube detached: {fut.result().message}')
        else:
            self.get_logger().warn(
                f'detach_cube failed: {fut.result() if fut.done() else "timeout"}')

    def update_robot_1(self):
        """Robot 1: pick blue cube -> place on sink (sink IS the yellow
        drop target per the task spec)."""
        r = self.robots['robot_1']

        if r.state == 'IDLE':
            cube = self.get_tf_pose('test_block')
            sink = self.get_tf_pose('sink')
            if cube and sink:
                r.arm.run_async(r.arm.to_drive_pose)
                self._transition(r, 'ARM_TO_DRIVE')

        elif r.state == 'ARM_TO_DRIVE':
            if not r.arm.busy:
                self._transition(r, 'APPROACH_CUBE')

        elif r.state == 'APPROACH_CUBE':
            cube = self.get_tf_pose('test_block')
            if cube is None:
                return
            # Compute a face-aligned pre-approach point on first entry so the
            # robot body squares up with a flat face of the cube.  The TF
            # alignment at REACH_DOWN then refines X/Y to the cube centre.
            if r.target is None:
                res = self.face_aligned_pre_point(r, cube)
                if res is None:
                    return
                tx, ty, adir = res
                r.target = (tx, ty)
                r.approach_yaw = adir
                self.get_logger().info(
                    f'[{r.name}] cube face-aligned pre-approach: '
                    f'({tx:.2f}, {ty:.2f}), yaw={math.degrees(adir):.0f} deg')
            if self.drive_to_pose(r, r.target[0], r.target[1]):
                r.target = None
                self._transition(r, 'FACE_CUBE')

        elif r.state == 'FACE_CUBE':
            cube = self.get_tf_pose('test_block')
            if cube is None:
                return
            if self.face_point(r, cube.pose.position.x,
                               cube.pose.position.y, tol=0.03):
                self._transition(r, 'PRE_PICK_ALIGN')

        elif r.state == 'PRE_PICK_ALIGN':
            if not r.reach_down_done:
                if not r.arm.busy:
                    r.arm.run_async(r.arm.reach_down_open)
                    r.reach_down_done = True
                    r._align_start_wall = time.time()
                return
            if r.arm.busy:
                return
            target = self.cube_grasp_target()
            if target is None:
                return
            t_world, _ = target
            if self.tf_final_align(r, t_world):
                self._transition(r, 'PICKUP')
                return
            # Hard wall-clock timeout: if alignment has not converged
            # within 30 s of real time, proceed to PICKUP anyway.
            # The cube_attach_detach fixed-joint mechanism works even
            # with ~20 mm misalignment (48 mm pad gap brackets the
            # 40 mm cube).  Wall-clock is used instead of sim time
            # because the sim may run at ~40% RTF, making sim-time
            # timeouts 2.5× longer than intended.
            align_start = getattr(r, '_align_start_wall', None)
            if align_start is not None:
                elapsed = time.time() - align_start
                if elapsed > 30.0:
                    self.get_logger().warn(
                        f'[{r.name}] PRE_PICK_ALIGN timed out after '
                        f'{elapsed:.1f}s wall-time — forcing PICKUP')
                    self.stop(r)
                    self._transition(r, 'PICKUP')

        elif r.state == 'PICKUP':
            # As the arm starts the grasp motion, attach the cube to the
            # gripper pad via a fixed Gazebo joint (bypasses the broken
            # contact physics).  The cube is now rigidly connected to
            # rlink2 for the duration of the carry.
            if r.name == 'robot_1' and not self._cube_attached:
                self._call_attach()
            r.arm.run_async(r.arm.grasp_and_lift)
            self._transition(r, 'PICKUP_WAIT')

        elif r.state == 'PICKUP_WAIT':
            self.stop(r)
            # Check the cube z EVERY tick (not just when the arm is
            # done).  If the cube is already lifted during the arm
            # motion, transition immediately.  This short-circuits
            # the full 5.5 s grasp_and_lift sequence when the pickup
            # actually works, and lets us see in the log EXACTLY when
            # the cube moves.
            #
            # Check the cube z EVERY tick (not just when the arm is
            # done).  If the cube is already lifted during the arm
            # motion, transition immediately.  This short-circuits
            # the full 5.5 s grasp_and_lift sequence when the pickup
            # actually works, and lets us see in the log EXACTLY when
            # the cube moves.
            cube = self.get_tf_pose('test_block')
            # Cube starts on the floor at z=0.02 (half-cube height).
            # It's "lifted" when its z is above 0.06 (clearly off the
            # floor by more than a sliver of controller jitter).
            if cube and cube.pose.position.z > 0.06:
                r.has_cube = True
                self.get_logger().info(
                    f'[{r.name}] Cube lifted (z={cube.pose.position.z:.3f}, '
                    f'arm_busy={r.arm.busy}); advancing')
                self._transition(r, 'DRIVE_TO_SINK')
            if not r.arm.busy:
                # Arm sequence finished, cube still on the floor.
                cube_z = cube.pose.position.z if cube else None
                self.get_logger().warn(
                    f'[{r.name}] Grasp failed (cube_z={cube_z}, arm done); '
                    f'retrying PRE_PICK_ALIGN')
                r.reach_down_done = False
                r._align_start_wall = None   # reset timeout for retry
                self._transition(r, 'PRE_PICK_ALIGN')
            else:
                # Arm still moving; throttled status so the log doesn't
                # flood.  Every 10 ticks (~1 s) print the cube z and
                # the arm busy flag so the operator can see the lift
                # happening in real time.
                r._pickwait_log = getattr(r, '_pickwait_log', 0) + 1
                if r._pickwait_log % 10 == 0:
                    cube_z = cube.pose.position.z if cube else None
                    self.get_logger().info(
                        f'[{r.name}] PICK_WAIT: cube_z={cube_z}, arm busy')

        elif r.state == 'DRIVE_TO_SINK':
            # Drive to a standoff in front of the sink (along the sink's
            # long axis) so the arm can extend forward and place the cube
            # on the basin rim.
            sink = self.get_tf_pose('sink')
            if sink is None:
                return
            if r.target is None:
                standoff = self.get_parameter('standoff_distance').value
                # Approach from the -X side of the sink (i.e. behind the
                # sink relative to the cube).  Standoff is along the X axis.
                tx = sink.pose.position.x - standoff
                ty = sink.pose.position.y
                r.target = (tx, ty)
                self.get_logger().info(
                    f'[{r.name}] sink standoff: ({tx:.2f}, {ty:.2f})')
            if self.drive_to_pose(r, r.target[0], r.target[1]):
                r.target = None
                self._transition(r, 'FACE_SINK')

        elif r.state == 'FACE_SINK':
            sink = self.get_tf_pose('sink')
            if sink is None:
                return
            if self.face_point(r, sink.pose.position.x,
                               sink.pose.position.y):
                self._transition(r, 'PLACE')

        elif r.state == 'PLACE':
            # As the arm starts the lower+release motion, detach the cube
            # from the gripper (remove the fixed joint we attached at
            # PICKUP).  The arm then opens the gripper, the cube falls
            # into the sink.
            if r.name == 'robot_1' and self._cube_attached:
                self._call_detach()
            r.arm.run_async(r.arm.lower_and_release)
            self._transition(r, 'RELEASE_WAIT')

        elif r.state == 'RELEASE_WAIT':
            self.stop(r)
            if not r.arm.busy:
                # Verify the cube landed on the sink rim, not on the floor.
                # The basin rim's world Z is sink_z + 0.05.  A cube resting
                # on it has its centre at rim_z + cube_radius.  If the cube
                # is on the floor instead, its z is ~0.01.
                cube = self.get_tf_pose('test_block')
                sink = self.get_tf_pose('sink')
                if cube is not None and sink is not None:
                    sink_rim_z = sink.pose.position.z + SINK_BASIN_RIM_LOCAL_Y
                    cube_target_z = sink_rim_z + CUBE_SIZE / 2.0
                    dz = cube.pose.position.z - cube_target_z
                    if abs(dz) < 0.03:
                        self.get_logger().info(
                            f'[{r.name}] Cube placed on sink '
                            f'(cube_z={cube.pose.position.z:.3f}, '
                            f'target_z={cube_target_z:.3f})')
                        r.has_cube = False
                        self._transition(r, 'BACKUP')
                    else:
                        self.get_logger().warn(
                            f'[{r.name}] Cube placement off: '
                            f'cube_z={cube.pose.position.z:.3f} '
                            f'target_z={cube_target_z:.3f} '
                            f'(dz={dz*1000:.1f}mm); retrying')
                        r.reach_down_done = False
                        self._transition(r, 'DRIVE_TO_SINK')
                else:
                    self._transition(r, 'BACKUP')

        elif r.state == 'BACKUP':
            twist = Twist()
            twist.linear.x = -0.5
            r.cmd_vel_pub.publish(twist)
            # Back up for ~0.25 m using sim time.
            if not hasattr(r, '_backup_start'):
                r._backup_start = self.get_clock().now()
            elapsed = (self.get_clock().now() - r._backup_start).nanoseconds / 1e9
            if elapsed >= 1.0:
                self.stop(r)
                r.arm.run_async(r.arm.fold_arm)
                self._transition(r, 'FOLD_WAIT')

        elif r.state == 'FOLD_WAIT':
            self.stop(r)
            if not r.arm.busy:
                r.task_done = True
                self._transition(r, 'DONE')
                self.get_logger().info(f'[{r.name}] TASK COMPLETE')

        elif r.state == 'DONE':
            self.stop(r)

    def update_robot_2_or_3(self, r: RobotState, side: int):
        """
        Robot 2/3 state machine.

        side: -1 -> -Y handle (Robot 2 starts at world y=-0.7),
              +1 -> +Y handle (Robot 3 starts at world y=+0.7).

        The sink is spawned with roll=pi/2 so its local Z axis points along
        world -Y and its local -Z axis points along world +Y.  The handles
        are at local (0, 0.015, +/-0.17) and therefore protrude in the world
        +/-Y directions at y = +/-0.17.  `sink_handle_targets()` returns
        them already mapped to the world frame.
        """
        sink_data = self.sink_handle_targets()
        if sink_data is None:
            return
        handle_minus_y, handle_plus_y, sink = sink_data
        my_handle = handle_minus_y if side == -1 else handle_plus_y

        if r.state == 'IDLE':
            r.arm.run_async(r.arm.to_drive_pose)
            self._transition(r, 'ARM_TO_DRIVE')

        elif r.state == 'ARM_TO_DRIVE':
            if not r.arm.busy:
                self._transition(r, 'APPROACH_SINK')

        elif r.state == 'APPROACH_SINK':
            if r.target is None:
                bx, by, byaw = self.robot_base_target_for_handle(
                    r, my_handle, sink)
                r.target = (bx, by)
                r.approach_yaw = byaw
                self.get_logger().info(
                    f'[{r.name}] sink standoff: ({bx:.2f}, {by:.2f}), '
                    f'yaw={math.degrees(byaw):.0f}deg, handle=({my_handle[0]:.2f},{my_handle[1]:.2f})')
            if self.drive_to_pose(r, r.target[0], r.target[1]):
                r.target = None
                self._transition(r, 'FACE_HANDLE')

        elif r.state == 'FACE_HANDLE':
            if self.face_point(r, my_handle[0], my_handle[1], tol=0.05):
                self._transition(r, 'PRE_GRASP_ALIGN')

        elif r.state == 'PRE_GRASP_ALIGN':
            if not r.horizontal_done:
                if not r.arm.busy:
                    r.arm.run_async(r.arm.horizontal_open)
                    r.horizontal_done = True
                return
            if r.arm.busy:
                return
            if self.tf_final_align(r, my_handle, tol=0.004):
                self._transition(r, 'WAIT_FOR_SYNC')

        elif r.state == 'WAIT_FOR_SYNC':
            self.stop(r)
            # Remain in ready pose; coordinator will advance mission state.

        elif r.state == 'GRASP':
            # Close gripper on the handle (no lift yet) in a background thread
            # so the executor callback is not blocked.
            r.arm.run_async(r.arm.close_horizontal)
            self._transition(r, 'GRASP_WAIT')

        elif r.state == 'GRASP_WAIT':
            if not r.arm.busy:
                self._transition(r, 'LIFT_READY')

        elif r.state == 'LIFT_READY':
            # Wait for coordinator to trigger synchronous lift.
            pass

        elif r.state == 'LIFT':
            # Synchronous lift via threading.Barrier(2).  Both robots run
            # their background threads blocked on the same barrier; the
            # actual lifting motion is started in lockstep once both
            # threads are ready.  This keeps the sink level during the
            # lift.  The barrier is created by the coordinator when the
            # mission enters the SINK_LIFT state.
            if self._lift_barrier is None:
                self.get_logger().error(
                    f'[{r.name}] LIFT state reached without a barrier — '
                    'falling back to solo lift')
                r.arm.run_async(r.arm.lift_horizontal)
            else:
                r.arm.run_async(r.arm.lift_horizontal_sync, self._lift_barrier)
            self._transition(r, 'HOLD')

        elif r.state == 'HOLD':
            self.stop(r)
            r.task_done = True

    # ------------------------------------------------------------------
    # Coordinator
    # ------------------------------------------------------------------
    def main_loop(self):
        r1 = self.robots['robot_1']
        r2 = self.robots['robot_2']
        r3 = self.robots['robot_3']

        if self.mission_state == 'WAIT_FOR_OBJECTS':
            cube = self.get_tf_pose('test_block')
            sink = self.get_tf_pose('sink')
            self.mission_timer += 1
            if self.mission_timer % 20 == 1:
                self.get_logger().info(
                    f'WAIT_FOR_OBJECTS: cube={cube is not None}, '
                    f'sink={sink is not None}')
            if cube and sink:
                self.get_logger().info('All object TFs available, starting mission')
                self.mission_state = 'ROBOT_1_PICK'
            return

        if self.mission_state == 'ROBOT_1_PICK':
            self.update_robot_1()
            # Helpers start moving to the sink in parallel with robot 1
            # (Step 1 and Step 2 happen concurrently per the task spec).
            # Once robot 1 has the cube we move to the next mission state.
            self.update_robot_2_or_3(r2, -1)
            self.update_robot_2_or_3(r3, +1)
            if r1.state == 'DRIVE_TO_SINK':
                self.get_logger().info('Robot 1 picked cube; helpers to sink')
                self.mission_state = 'HELPERS_TO_SINK'
            return

        if self.mission_state == 'HELPERS_TO_SINK':
            self.update_robot_1()          # continues transport
            self.update_robot_2_or_3(r2, -1)
            self.update_robot_2_or_3(r3, +1)
            if r2.state == 'WAIT_FOR_SYNC' and r3.state == 'WAIT_FOR_SYNC':
                self.get_logger().info('Helpers in pre-grasp poses; waiting for cube placement')
                self.mission_state = 'WAIT_PLACE_THEN_GRASP'
            return

        if self.mission_state == 'WAIT_PLACE_THEN_GRASP':
            self.update_robot_1()
            # Keep helpers holding their ready poses.
            for r in [r2, r3]:
                if r.state == 'WAIT_FOR_SYNC':
                    self.stop(r)
            if r1.state == 'DONE':
                self.get_logger().info('Cube placed; helpers grasping sink')
                r2.state = 'GRASP'
                r3.state = 'GRASP'
                self.mission_state = 'SINK_GRASP'
            return

        if self.mission_state == 'SINK_GRASP':
            self.update_robot_2_or_3(r2, -1)
            self.update_robot_2_or_3(r3, +1)
            if r2.state == 'GRASP_WAIT' and r3.state == 'GRASP_WAIT':
                self.get_logger().info('Both grippers closed; synchronous lift')
                # Create the lift barrier so the LIFT state can use it.
                # threading.Barrier(2) means both threads must arrive before
                # either of them is allowed to proceed.
                self._lift_barrier = threading.Barrier(2, timeout=20.0)
                r2.state = 'LIFT'
                r3.state = 'LIFT'
                self.mission_state = 'SINK_LIFT'
            return

        if self.mission_state == 'SINK_LIFT':
            self.update_robot_2_or_3(r2, -1)
            self.update_robot_2_or_3(r3, +1)
            if r2.state == 'HOLD' and r3.state == 'HOLD':
                self.get_logger().info('═══════════════════════════════════════════')
                self.get_logger().info('  DUAL-ROBOT SINK LIFT COMPLETE')
                self.get_logger().info('═══════════════════════════════════════════')
                # Discard the barrier so the next run starts clean.
                self._lift_barrier = None
                self.mission_state = 'DONE'
            return

        if self.mission_state == 'DONE':
            for r in self.robots.values():
                self.stop(r)


def main():
    rclpy.init()
    node = MultiRobotCubeSinkAutopilot()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for r in node.robots.values():
            node.stop(r)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
