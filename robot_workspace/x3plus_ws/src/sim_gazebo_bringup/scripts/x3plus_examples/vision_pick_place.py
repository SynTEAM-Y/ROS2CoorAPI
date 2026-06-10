#!/usr/bin/env python3
"""Vision-based autonomous pick-and-place using camera feedback.

Integrates object detection, navigation, and vision-guided arm control
to autonomously pick objects detected by the camera.

State machine:
  IDLE -> DETECT -> NAVIGATE -> APPROACH -> PICK -> TRANSPORT -> PLACE -> IDLE

Usage:
  ros2 launch sim_gazebo_bringup vision_autopilot.launch.py
"""

import sys
import time
import math
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, Point
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState, Image
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf2_geometry_msgs

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# ── Pre-defined arm joint positions (radians) ──────────────────────────
# Manufacturer (ROS1) uses degrees where 90° = neutral.
# URDF convention: 0 rad = neutral → URDF = (deg - 90) × π/180

# ── Manufacturer poses (exact from autopilot_main.py + transport_main.py) ──
# Home:       [90, 120,  0,  0, 90, 30] → URDF [ 0.000,  0.524, -1.571, -1.571,  0.000]
# Pre-pick:   [90, 145,  0, 45, 90, 30] → URDF [ 0.000,  0.960, -1.571, -0.785,  0.000]
# Pick:       [pos1, 7, 60, 38, 90, 30] → URDF [j1_rad, -1.449, -0.524, -0.908,  0.000]
# Pick grip:  [ 0,  7, 60, 38, 90, 140] → URDF [-1.571, -1.449, -0.524, -0.908,  0.000]
# Place:      [90,  2, 60, 40, 90, 30] → URDF [ 0.000, -1.536, -0.524, -0.873,  0.000]

MFR_HOME     = [0.0,   0.524, -1.571, -1.571, 0.0]    # [90, 120,  0,  0, 90]
MFR_PRE_PICK = [0.0,   0.960, -1.571, -0.785, 0.0]    # [90, 145,  0, 45, 90]
MFR_CARRY    = [0.0,  -0.524, -0.524, -0.908, 0.0]    # [90, 60, 60, 38, 90] — lifted carry
MFR_PLACE    = [0.0,  -1.536, -0.524, -0.873, 0.0]    # [90,   2, 60, 40, 90]
MFR_PRE_PLACE = MFR_CARRY                              # same as carry

# Legacy poses (kept for backward compat, not used in manufacturer flow)
ARM_UP       = [0.0,   0.0,    0.0,    0.0,   0.0]
ARM_DOWN     = [0.0,  -1.5708, 0.0,    0.0,   0.0]
ARM_INIT     = [0.0,   0.7854,-1.5708,-1.5708, 0.0]
ARM_APPROACH = ARM_INIT
HOME         = ARM_UP
PRE_PICK     = [0.0,  -0.8,   -0.4,   -0.3,   0.0]
REACH_DOWN   = [0.0,  -1.45,  -0.524,  -0.908,  0.0]
CARRY        = [0.0,  -0.8,   -0.4,   -0.3,   0.0]
PLACE_DOWN   = [0.0,  -1.40,  -0.524, -0.873, 0.0]
PRE_PLACE    = [0.0,  -0.8,   -0.4,   -0.3,   0.0]

# SRDF gripper group_states: open=-1.54, close=0.0
GRIPPER_OPEN  = -1.54
GRIPPER_HOLD  = -0.55   # Secure grip (~4.5 cm finger gap on 4 cm cube)
GRIPPER_CLOSE = 0.0
ARM_JOINT_NAMES = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']

# MoveIt joint_limits.yaml: max_velocity = 1.0 rad/s for all arm joints.
# Use a safety scaling matching default_velocity_scaling_factor = 0.1 ... 1.0.
ARM_MAX_VELOCITY = 1.0  # rad/s (hard URDF/MoveIt limit)
ARM_VEL_SCALE    = 1.0  # full speed — controller handles saturation internally

# MoveIt kinematics.yaml goal_position_tolerance = 0.0001 m. We use a slightly
# looser deadband (50 mm) for closed-loop FK calibration, accounting for measured
# ~41 mm FK errors and gripper system tolerances.
POSITION_TOLERANCE_M = 0.050

# arm_link5 -> grip centre offset (calibrated from TF: arm_link5.x=0.407,
# actual finger centre ≈ 0.292 → offset = -0.115 m).
ARM5_TO_GRIP_CENTER = -0.115

# Empirical FK compensation (m): the trajectory controller settles ~41 mm
# short of REACH_DOWN due to gravity droop (gripper ends up CLOSER to
# base than FK predicts).  This value is SUBTRACTED from the FK-predicted
# reach so the robot stops CLOSER to the cube — exactly where the actual
# gripper lands after settling.  The fingers span a 4.8 cm gap, so the
# gripper centre must be within ~2 cm of the 4 cm cube centre.
# NOTE: 2026-06-02 test showed actual error ~41mm.
FK_SETTLE_COMPENSATION = 0.040


def _safe_traj_duration(current, target, requested_sec):
    """Return a duration that respects ARM_MAX_VELOCITY * ARM_VEL_SCALE.

    `current` and `target` are joint-position lists (rad). If `current` is
    None (no joint state yet) we just return `requested_sec`.
    """
    if current is None:
        return max(requested_sec, 0.5)
    max_delta = max(abs(t - c) for t, c in zip(target, current))
    v_max = ARM_MAX_VELOCITY * ARM_VEL_SCALE
    min_dur = max_delta / v_max if v_max > 0 else requested_sec
    # Add a 25% buffer for acceleration phase.
    min_dur *= 1.25
    return max(requested_sec, min_dur)

# HSV ranges for blue block validation in wrist camera
BLUE_LOWER = np.array([80, 50, 50])
BLUE_UPPER = np.array([120, 255, 255])
VISION_MIN_AREA = 200
WRIST_CAMERA_TIMEOUT = 3.0


class VisionPickPlace(Node):
    """Vision-guided pick-and-place autopilot."""
    
    # State machine states
    STATE_IDLE = 'IDLE'
    STATE_DETECT = 'DETECT'
    STATE_NAVIGATE = 'NAVIGATE'
    STATE_APPROACH = 'APPROACH'
    STATE_PICK = 'PICK'
    STATE_TRANSPORT = 'TRANSPORT'
    STATE_PLACE = 'PLACE'
    
    def __init__(self):
        super().__init__('vision_pick_place')
        self._cb_group = ReentrantCallbackGroup()
        
        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('object_x', 2.0)
        self.declare_parameter('object_y', 0.0)
        self.declare_parameter('object_z', 0.03)
        self.declare_parameter('drop_off_x', 2.0)
        self.declare_parameter('drop_off_y', 1.2)
        # Pad sits north of cube spawn; gripper drops forward of base, so
        # the robot must face +y (math.pi/2) for PLACE_DOWN to centre on pad.
        self.declare_parameter('drop_off_yaw', math.pi / 2.0)
        
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('pose_topic', '/detected_object_pose')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('arm_action', '/arm_group_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_action', '/gripper_group_controller/follow_joint_trajectory')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        # State
        self.state = self.STATE_IDLE
        self._object_pose_map = None
        self._detected_pose_cam = None
        self._detected_pose_map = None
        self._detected_pose_time = None
        
        # TF2
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        # Action clients
        self._arm_traj_ac = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('arm_action').value,
            callback_group=self._cb_group)
        self._gripper_traj_ac = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('gripper_action').value,
            callback_group=self._cb_group)
        
        # Publishers
        self._cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._cmd_vel_ign_pub = self.create_publisher(
            Twist, '/model/x3plus/cmd_vel', 10)
        
        # Subscribers
        self._odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        # Robot pose comes from the GROUND-TRUTH Ignition PosePublisher
        # (bridged to /gz_pose_tf), NOT the wheel /odom topic. Wheel odom yaw
        # over-reports badly under skid, which made the robot face slightly off
        # (missing the cube laterally) and overshoot the 90 deg turn toward the
        # drop zone. /gz_pose_tf is the same source the corrected base TF uses,
        # so robot pose and cube detection stay in one consistent frame.
        self.create_subscription(
            TFMessage, '/gz_pose_tf', self._on_gz_pose, self._odom_qos)
        # Legacy wheel odom kept only as a fallback if ground truth is absent.
        self._have_gt_pose = False
        self.create_subscription(
            Odometry, '/odom', self._on_odom, self._odom_qos)
        
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self._on_detected_object, 10, callback_group=self._cb_group)
        
        self._wrist_image = None
        self._bridge = CvBridge() if CV_AVAILABLE else None
        if CV_AVAILABLE:
            self.create_subscription(
                Image, '/wrist_mono_camera/image_raw',
                self._on_wrist_image, 1, callback_group=self._cb_group)
        
        self._joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, self._odom_qos, callback_group=self._cb_group)
        
        self._llink2_contact = None
        self._rlink2_contact = None
        self.create_subscription(
            Contacts, '/model/x3plus/contact/llink2',
            self._on_llink2_contact, 10, callback_group=self._cb_group)
        self.create_subscription(
            Contacts, '/model/x3plus/contact/rlink2',
            self._on_rlink2_contact, 10, callback_group=self._cb_group)
            
        self.get_logger().info('Vision Pick-and-Place state machine initialized')
        
    def _on_odom(self, msg):
        # Fallback only: used when ground-truth /gz_pose_tf is unavailable.
        if self._have_gt_pose:
            return
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _on_gz_pose(self, msg: TFMessage):
        # Ground-truth robot pose from the Ignition PosePublisher. The model
        # root transform has child_frame_id ending in 'x3plus'.
        for tr in msg.transforms:
            if tr.child_frame_id.split('/')[-1] == 'x3plus':
                self._have_gt_pose = True
                self._odom_x = tr.transform.translation.x
                self._odom_y = tr.transform.translation.y
                q = tr.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self._odom_yaw = math.atan2(siny, cosy)
                return

    def _on_detected_object(self, msg: PoseStamped):
        """Handle detected object pose from camera."""
        self._detected_pose_cam = msg
        self._detected_pose_time = self.get_clock().now()
        # Store the detected object pose in the robot's odom frame whenever
        # possible, because the autopilot uses odom for base motion and the
        # map->odom transform may differ in navigation mode.
        try:
            cam_to_odom = self._tf_buffer.lookup_transform(
                'odom', msg.header.frame_id,
                Time(), Duration(seconds=0.5))
            pose_odom = tf2_geometry_msgs.do_transform_pose_stamped(msg, cam_to_odom)
            self._detected_pose_map = pose_odom
            return
        except Exception as e:
            self.get_logger().warn(
                f'TF camera->odom failed (frame_id={msg.header.frame_id}): {e}')

        try:
            cam_to_map = self._tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                Time(), Duration(seconds=0.5))
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(msg, cam_to_map)
            self._detected_pose_map = pose_map
        except Exception as e:
            self.get_logger().warn(
                f'TF camera->map failed (frame_id={msg.header.frame_id}): {e}')

    def _on_wrist_image(self, msg: Image):
        if self._bridge is None:
            return
        try:
            self._wrist_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Wrist camera cv_bridge failed: {e}')

    def _on_llink2_contact(self, msg: Contacts):
        self._llink2_contact = msg

    def _on_rlink2_contact(self, msg: Contacts):
        self._rlink2_contact = msg

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def run_autopilot(self):
        """Manufacturer-style visual pick-and-place.

        Simple pipeline matching ROS1 autopilot_main.py:
          1. IDLE: Move arm to observe pose
          2. DETECT: Wait for camera to see cube
          3. NAVIGATE: Drive toward cube using pixel PID (no TF)
          4. PICK: When pixel condition met, execute manufacturer arm sequence
          5. TRANSPORT: Drive to drop zone, place cube
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING MANUFACTURER-STYLE VISION PICK-AND-PLACE')
        self.get_logger().info('=' * 60)

        if not self._wait_for_servers():
            return False

        # ── STATE: IDLE ─────────────────────────────────────────────
        self.state = self.STATE_IDLE
        self.get_logger().info('[STATE] IDLE: Moving arm to observe pose')
        self._gripper_open()
        self._sleep_sim(0.5)
        self._move_arm(MFR_HOME, 'mfr_home', duration_sec=2.0)
        time.sleep(0.3)

        # Wait for camera to detect cube
        deadline = time.monotonic() + 180.0
        last_log = time.monotonic()
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._detected_pose_map is not None:
                age = (self.get_clock().now() - self._detected_pose_time).nanoseconds / 1e9
                if age < 3.0:
                    break
            if time.monotonic() - last_log > 10.0:
                self.get_logger().warn(
                    f'[DETECT] Waiting for cube... elapsed={time.monotonic() - (deadline - 180.0):.0f}s')
                last_log = time.monotonic()
            time.sleep(0.1)

        if self._detected_pose_map is None:
            self.get_logger().error('Timeout waiting for vision detection')
            return False

        # ── STATE: DETECT ───────────────────────────────────────────
        self.state = self.STATE_DETECT
        self._object_pose_map = self._detected_pose_map
        self.get_logger().info(
            f'[STATE] DETECT: Cube at ({self._object_pose_map.pose.position.x:.2f}, '
            f'{self._object_pose_map.pose.position.y:.2f})')

        # ── STATE: NAVIGATE (Pixel PID drive to cube) ───────────────
        self.state = self.STATE_NAVIGATE
        self.get_logger().info('[STATE] NAVIGATE: Driving to cube using pixel PID')

        # Fold arm for driving (manufacturer pre-pick pose)
        self._move_arm(MFR_PRE_PICK, 'mfr_pre_pick', duration_sec=2.0)
        time.sleep(0.2)

        # Drive toward cube using pixel-based PID (manufacturer approach)
        nav_ok = self._pixel_pid_navigate(timeout=60.0)
        if not nav_ok:
            self.get_logger().warn('⚠️ Navigation timed out — proceeding anyway')

        # ── STATE: PICK (Manufacturer arm sequence) ─────────────────
        self.state = self.STATE_PICK
        self.get_logger().info('[STATE] PICK: Executing manufacturer arm sequence')

        # Read final pixel position for joint1 computation
        pixel_x, pixel_y = self._get_cube_pixel()
        if pixel_x is not None:
            # Manufacturer linear mapping: [320, 90] → [343.5, 95] in degrees
            pos1_deg = 0.2128 * pixel_x + 21.91
            j1_rad = (pos1_deg - 90.0) * math.pi / 180.0
            self.get_logger().warn(
                f'  Pick pixel=({pixel_x:.0f},{pixel_y:.0f}) → j1={pos1_deg:.1f}°')
        else:
            j1_rad = 0.0
            self.get_logger().warn('  No pixel data — using j1=0')

        # Manufacturer pick approach pose: [pos1, 7°, 60°, 38°, 90°]
        # URDF radians: (deg-90)*π/180
        # NOTE: j4 adjusted from 38°→45° for simulation URDF geometry.
        # The real robot uses j4=38°, but the sim URDF has arm_joint5 rpy=π/2
        # and grip_joint rpy=-π/2 which shifts the gripper orientation.
        # j4=45° (URDF -0.785) makes the gripper fingers point straight down.
        pick_approach = [j1_rad, -1.449, -0.524, -0.785, 0.0]

        # Step 1: Move to pick approach pose with gripper OPEN
        self.get_logger().info('  [MFR] Pick approach pose + gripper OPEN')
        self._gripper_open()
        self._sleep_sim(0.5)
        self._move_arm(pick_approach, 'mfr_pick_approach', duration_sec=3.0)
        self._sleep_sim(1.0)

        # Step 2: Lower arm — j2 down to 60° (manufacturer: id=2, angle=60)
        self.get_logger().info('  [MFR] Lower arm (j2→60°)')
        lower_pose = list(pick_approach)
        lower_pose[1] = -0.524  # j2 = 60° URDF
        self._move_arm(lower_pose, 'mfr_lower_j2', duration_sec=1.5)
        self._sleep_sim(0.5)

        # Step 3: Further lower — j1 to neutral 0° (manufacturer: id=1, angle=0)
        # This is the actual pick grip pose: gripper vertical over cube
        self.get_logger().info('  [MFR] Further lower (j1→0° neutral) — gripper vertical')
        lower_pose[0] = -1.571  # j1 = 0° URDF
        self._move_arm(lower_pose, 'mfr_lower_j1', duration_sec=1.5)
        self._sleep_sim(0.5)

        # Step 4: Close gripper (manufacturer: joints[5]=140 → URDF 0.0)
        self.get_logger().info('  [MFR] Close gripper')
        self._gripper_close()
        self._sleep_sim(1.5)

        # Step 5: Lift to CARRY pose
        self.get_logger().info('  [MFR] Lift with cube')
        self._move_arm(MFR_CARRY, 'mfr_lift', duration_sec=2.0)
        self._sleep_sim(0.5)

        # Verify grasp
        if self._cube_is_lifted():
            self.get_logger().info('✅ GRASP CONFIRMED')
            grasped = True
        else:
            self.get_logger().warn('⚠️ Grasp failed — retrying once')
            self._gripper_open()
            self._sleep_sim(0.5)
            self._move_arm(pick_approach, 'mfr_retry', duration_sec=1.5)
            self._sleep_sim(0.5)
            self._gripper_close()
            self._sleep_sim(1.5)
            self._move_arm(MFR_CARRY, 'mfr_retry_lift', duration_sec=2.0)
            self._sleep_sim(0.5)
            grasped = self._cube_is_lifted()
            if grasped:
                self.get_logger().info('✅ GRASP CONFIRMED on retry')
            else:
                self.get_logger().error('❌ PICK FAILED')

        if not grasped:
            self.get_logger().error('❌ PICK FAILED — aborting')
            self._gripper_open()
            self._sleep_sim(1.0)
            self._move_arm(MFR_HOME, 'fold_home_failed', duration_sec=2.0)
            return False

        # ── STATE: TRANSPORT ────────────────────────────────────────
        self.state = self.STATE_TRANSPORT
        self.get_logger().info('[STATE] TRANSPORT: Driving to drop zone')

        self._move_arm(MFR_HOME, 'mfr_home_transport', duration_sec=2.0)
        time.sleep(0.2)

        # Safety maneuver
        self._backup_and_strafe()

        drop_x = self.get_parameter('drop_off_x').value
        drop_y = self.get_parameter('drop_off_y').value

        gripper_reach = self._gripper_center_x_at_joints(MFR_PLACE) - FK_SETTLE_COMPENSATION
        place_standoff = max(0.20, gripper_reach)

        self.get_logger().info(
            f'  Driving to landing pad ({drop_x:.2f}, {drop_y:.2f}) '
            f'standoff={place_standoff:.2f}m')
        self._drive_to_pose_xy(
            drop_x, drop_y,
            stop_dist=place_standoff,
            max_lin=0.5,
            max_ang=1.0,
            timeout=30.0,
            log_prefix='  Transport',
        )

        # ── STATE: PLACE ────────────────────────────────────────────
        self.state = self.STATE_PLACE
        self.get_logger().info('[STATE] PLACE: Placing cube on landing pad')

        self._move_arm(MFR_PRE_PLACE, 'mfr_pre_place', duration_sec=1.5)
        time.sleep(0.1)
        self._move_arm(MFR_PLACE, 'mfr_place', duration_sec=1.5)
        time.sleep(0.1)

        self._gripper_open()
        self._sleep_sim(2.0)
        self.get_logger().info('Gripper released cube')

        self._move_arm(MFR_HOME, 'mfr_final_home', duration_sec=2.0)
        time.sleep(0.1)

        self.get_logger().info('=' * 60)
        self.get_logger().info('AUTOPILOT PICK-AND-PLACE COMPLETE')
        self.get_logger().info('=' * 60)

        self.state = self.STATE_IDLE
        self._object_pose_map = None
        return True

    # ── Utilities ────────────────────────────────────────────────────
    
    def _wait_for_servers(self):
        self.get_logger().info('Waiting for trajectory action servers (arm + gripper)...')
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            arm_ok = self._arm_traj_ac.wait_for_server(timeout_sec=1.0)
            grip_ok = self._gripper_traj_ac.wait_for_server(timeout_sec=1.0)
            if arm_ok and grip_ok:
                self.get_logger().info('Action servers ready')
                return True
            time.sleep(2.0)
        self.get_logger().error('Action servers not ready after 60 s')
        return False

    def _move_arm(self, joint_positions, label, duration_sec=2.0):
        # Auto-extend duration to respect MoveIt joint_limits.yaml
        # (max_velocity = 1.0 rad/s, scaled by ARM_VEL_SCALE).
        current = None
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            if all(n in js for n in ARM_JOINT_NAMES):
                current = [js[n] for n in ARM_JOINT_NAMES]
        duration_sec = _safe_traj_duration(current, joint_positions, duration_sec)

        self.get_logger().info(
            f'[{label}] Arm -> {["%.2f" % j for j in joint_positions]} (dur={duration_sec:.2f}s)')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        goal.trajectory.points = [point]

        if not self._arm_traj_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{label}] Arm trajectory server not available')
            return False

        future = self._arm_traj_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=duration_sec + 3.0)
        if not future.done() or not future.result().accepted:
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)
        return result_future.done()

    def _move_arm_with_alignment(self, joint_positions, label, duration_sec=2.0,
                                  cube_world_x=None, cube_world_y=None):
        """Move arm to joint_positions while applying concurrent base corrections.

        During the arm trajectory, continuously monitors the cube position via
        the ground-truth ``test_block`` TF and applies small yaw + forward
        corrections so the gripper centre tracks the cube as the arm extends.
        This closes any micro-drift that remained after ``_camera_guided_approach``
        and keeps the gripper aligned with the cube throughout the REACH_DOWN
        motion, enabling a successful pick.

        Only active when cube_world_x/y are provided.  Corrections are kept
        intentionally small (≤0.25 rad/s yaw, ≤0.08 m/s forward) to avoid
        disturbing the arm physics while it moves.
        """
        current = None
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            if all(n in js for n in ARM_JOINT_NAMES):
                current = [js[n] for n in ARM_JOINT_NAMES]
        duration_sec = _safe_traj_duration(current, joint_positions, duration_sec)

        self.get_logger().info(
            f'[{label}] Arm -> {["%.2f" % j for j in joint_positions]} '
            f'(dur={duration_sec:.2f}s, base_align={cube_world_x is not None})')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        goal.trajectory.points = [point]

        if not self._arm_traj_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{label}] Arm trajectory server not available')
            return False

        future = self._arm_traj_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=duration_sec + 3.0)
        if not future.done() or not future.result().accepted:
            return False

        result_future = future.result().get_result_async()

        # Concurrent base alignment while arm moves. Recompute the current
        # gripper reach from live joint state rather than assuming the final
        # pose the entire time, because the arm is still extending during this
        # phase.
        t_arm0 = time.monotonic()
        cx, cy = cube_world_x, cube_world_y
        last_corr_log = 0.0

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.02)

            # Compute the current gripper reach from the most recent joint state.
            current_finger_center = None
            if self._joint_state is not None:
                js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
                if all(n in js for n in ARM_JOINT_NAMES):
                    current_joints = [js[n] for n in ARM_JOINT_NAMES]
                    current_finger_center = (
                        self._gripper_center_x_at_joints(current_joints)
                        - FK_SETTLE_COMPENSATION)
            if current_finger_center is None:
                current_finger_center = (
                    self._gripper_center_x_at_joints(joint_positions)
                    - FK_SETTLE_COMPENSATION)

            # Safety: don't spin longer than duration + 6 s
            if time.monotonic() - t_arm0 > duration_sec + 6.0:
                break

            if cube_world_x is None or cx is None or cy is None:
                time.sleep(0.02)
                continue

            # Refresh cube position from GT TF each cycle
            try:
                ta = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(),
                    Duration(seconds=0.05))
                cx = ta.transform.translation.x
                cy = ta.transform.translation.y
            except Exception:
                pass  # keep last cx, cy

            dx = cx - self._odom_x
            dy = cy - self._odom_y
            
            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)
                 
            forward = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
            dist_err = forward - current_finger_center

            # Only apply if errors are meaningful (avoid noise chatter)
            corr_ang = 0.0
            corr_lin = 0.0
            if abs(yaw_err) > math.radians(1.5):
                corr_ang = max(-0.25, min(0.25, yaw_err * 2.0))
            if abs(yaw_err) < math.radians(8.0) and abs(dist_err) > 0.012:
                corr_lin = max(-0.08, min(0.08, dist_err * 1.2))

            if abs(corr_ang) > 0.01 or abs(corr_lin) > 0.005:
                twist = Twist()
                twist.angular.z = corr_ang
                twist.linear.x = corr_lin
                self._publish_cmd_vel(twist)
                if time.monotonic() - last_corr_log > 0.5:
                    self.get_logger().info(
                        f'  [arm-align] yaw_err={math.degrees(yaw_err):+.1f}° '
                        f'dist_err={dist_err*1000:+.0f}mm '
                        f'cmd=(lin={corr_lin:.3f}, ang={corr_ang:.3f})')
                    last_corr_log = time.monotonic()

            time.sleep(0.02)

        # Full stop after arm completes
        for _ in range(5):
            self._publish_cmd_vel(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

        # Wait for result if trajectory hasn't confirmed completion yet
        if not result_future.done():
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=duration_sec + 5.0)
        return result_future.done()

    def _gripper_open(self):
        self.get_logger().info('Gripper -> OPEN')
        self._gripper(GRIPPER_OPEN)

    def _gripper_close(self):
        self.get_logger().info('Gripper -> HOLD (4.8cm parallelism)')
        self._gripper(GRIPPER_HOLD)

    def _gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['grip_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        goal.trajectory.points = [point]
        self._gripper_traj_ac.send_goal_async(goal)

    def _sleep_sim(self, seconds):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _cube_is_lifted(self):
        """Ground-truth grasp check with contact verification.

        A real grasp carries the cube up with the arm, so its world z rises
        well above the spawn height (~0.0125-0.03 m). An empty close leaves it
        on the ground. 
        
        Also checks finger contact sensors for early detection of grasp failure:
        - Both fingers should report contact (not just one)
        - Symmetry in contact indicates centered grasp
        
        Returns True only if the cube is clearly elevated.
        """
        for attempt in range(6):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.5))
                z = t.transform.translation.z
                lifted = z > 0.10
                
                # Log contact sensor state for diagnostics
                if self._llink2_contact is not None or self._rlink2_contact is not None:
                    lcontact = len(self._llink2_contact.states) if self._llink2_contact else 0
                    rcontact = len(self._rlink2_contact.states) if self._rlink2_contact else 0
                    self.get_logger().info(
                        f'  Cube height check: z={z*1000:.1f} mm (lifted={lifted}) | '
                        f'contacts: L={lcontact} R={rcontact}')
                else:
                    self.get_logger().info(f'  Cube height check: z={z*1000:.1f} mm (lifted={lifted})')
                
                return lifted
            except Exception:
                time.sleep(0.1)
        self.get_logger().warn(
            '  Cube TF unavailable for lift check — trying contact-sensor fallback')
        # Fallback: if BOTH finger pads report active contact the cube is
        # almost certainly between the fingers (empty-air close = no contact).
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        lcontact = (self._llink2_contact is not None
                    and len(self._llink2_contact.contacts) > 0)
        rcontact = (self._rlink2_contact is not None
                    and len(self._rlink2_contact.contacts) > 0)
        self.get_logger().info(
            f'  Contact fallback: L={lcontact} R={rcontact}')
        if lcontact and rcontact:
            self.get_logger().info(
                '  Both finger contacts active → assuming cube is grasped')
            return True
        self.get_logger().warn('  No contact on one or both fingers — NOT grasped')
        return False

    def _compensate_arm_joint1_for_cube_center(self):
        """ROS1-style: adjust arm_joint1 based on cube pixel position to center gripper.

        ROS1 uses a linear mapping from pixel_x to arm_joint1:
          pixel_x=320 → j1=90° (URDF 0.0)
          pixel_x=343.5 → j1=95° (URDF 0.087)
        This compensates for the camera-to-gripper parallax offset.
        """
        if not CV_AVAILABLE or self._bridge is None:
            self.get_logger().warn('  j1 compensation skipped: cv_bridge unavailable')
            return

        self.get_logger().warn('═══════ ARM J1 COMPENSATION (ROS1-style) ══════')

        # Get latest wrist camera frame
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._wrist_image is not None:
                break

        if self._wrist_image is None:
            self.get_logger().warn('  No wrist camera frame — skipping j1 compensation')
            return

        try:
            hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]
            if not large:
                self.get_logger().warn('  No cube in wrist view — skipping j1 compensation')
                return

            largest = max(large, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            cube_px = x + w // 2
            width = self._wrist_image.shape[1]
            img_cx = width // 2  # 320

            self.get_logger().warn(f'  Cube pixel_x={cube_px} (center={img_cx})')

            # ROS1 linear mapping: [320, 90] → [343.5, 95] in degrees
            # a = (95-90)/(343.5-320) = 5/23.5 ≈ 0.2128
            # b = 90 - 0.2128*320 ≈ 21.91
            # pos1_deg = 0.2128 * cube_px + 21.91
            pos1_deg = 0.2128 * cube_px + 21.91
            # Convert to URDF radians: (deg-90)*π/180
            j1_comp = (pos1_deg - 90.0) * math.pi / 180.0

            self.get_logger().warn(f'  j1 compensation: {j1_comp*180/math.pi:+.2f}° (URDF rad)')

            # Apply: move arm to REACH_DOWN with compensated j1
            compensated = list(REACH_DOWN)
            compensated[0] = j1_comp
            self._move_arm(compensated, 'reach_down_j1_comp', duration_sec=1.0)
            time.sleep(0.3)

        except Exception as e:
            self.get_logger().warn(f'  j1 compensation error: {e}')

    def _tf_center_gripper_on_cube(self, timeout=20.0):
        """Closed-loop TF-based centering: align gripper centre onto cube.

        Called after the arm is at REACH_DOWN.  Reads the actual finger-centre
        TF (rlink2 + llink2 midpoint) and the cube TF (test_block), computes
        forward + lateral offset in the robot frame, and drives the base to
        eliminate both.  Runs until convergence or timeout.
        """
        self.get_logger().warn('═══════ TF-BASED GRIPPER CENTERING ═══════')

        t0 = time.monotonic()
        last_log = 0.0
        converge_count = 0
        required_converge = 3  # consecutive good reads

        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)

            # Cube TF
            try:
                tc = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.1))
                cube_x = tc.transform.translation.x
                cube_y = tc.transform.translation.y
            except Exception:
                self.get_logger().warn('  Cube TF unavailable — retrying')
                time.sleep(0.1)
                continue

            # Finger centre TF
            finger_x, finger_y, finger_src = self._get_finger_center_x()
            if finger_src != 'finger_TF':
                self.get_logger().warn(
                    f'  Finger TF unavailable (src={finger_src}) — retrying')
                time.sleep(0.1)
                continue

            # Finger world position (full 2D transform)
            fw_x = (self._odom_x
                    + finger_x * math.cos(self._odom_yaw)
                    - finger_y * math.sin(self._odom_yaw))
            fw_y = (self._odom_y
                    + finger_x * math.sin(self._odom_yaw)
                    + finger_y * math.cos(self._odom_yaw))

            # Offset in robot frame
            dx = cube_x - fw_x
            dy = cube_y - fw_y
            forward_err = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
            lateral_err = -dx * math.sin(self._odom_yaw) + dy * math.cos(self._odom_yaw)

            if time.monotonic() - last_log > 0.5:
                self.get_logger().info(
                    f'  centering: forward={forward_err*1000:+.1f}mm  '
                    f'lateral={lateral_err*1000:+.1f}mm  '
                    f'finger_world=({fw_x:.4f},{fw_y:.4f})  '
                    f'cube=({cube_x:.4f},{cube_y:.4f})')
                last_log = time.monotonic()

            # Convergence check: within 5 mm forward and 3 mm lateral
            if abs(forward_err) < 0.005 and abs(lateral_err) < 0.003:
                converge_count += 1
                if converge_count >= required_converge:
                    self.get_logger().warn(
                        f'  ✅ TF centering converged '
                        f'(fwd={forward_err*1000:+.1f}mm, lat={lateral_err*1000:+.1f}mm)')
                    break
            else:
                converge_count = 0

            # Compute corrections
            twist = Twist()
            # Forward correction (proportional, clamped)
            twist.linear.x = max(-0.10, min(0.10, forward_err * 1.5))
            # Lateral correction via yaw (rotate toward cube)
            if abs(lateral_err) > 0.003:
                # Target yaw that would eliminate lateral offset
                target_yaw = math.atan2(cube_y - self._odom_y, cube_x - self._odom_x)
                yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)
                twist.angular.z = max(-0.4, min(0.4, yaw_err * 2.0))

            self._publish_cmd_vel(twist)
            time.sleep(0.05)

        # Full stop
        self._publish_cmd_vel(Twist())
        for _ in range(5):
            self._publish_cmd_vel(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.2)

    def _forward_distance_check(self):
        """Check and correct forward distance only — no lateral rotation.

        The camera-guided approach already aligned the gripper laterally with
        the cube.  After the arm descends to REACH_DOWN, we only verify the
        forward distance is correct.  Small forward/backward pulses are applied
        if needed, but NO rotation (which would break lateral alignment on a
        skid-steer robot).
        """
        self.get_logger().warn('═══════ FORWARD DISTANCE CHECK ═══════')

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        for iteration in range(5):
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.2))
                cube_x = t.transform.translation.x
                cube_y = t.transform.translation.y
            except Exception:
                self.get_logger().warn('  TF cube unavailable')
                return

            finger_x, finger_y, finger_src = self._get_finger_center_x()
            if finger_src != 'finger_TF':
                self.get_logger().warn(f'  Finger TF unavailable (src={finger_src})')
                return

            # Finger world position (full 2D transform)
            fw_x = self._odom_x + finger_x * math.cos(self._odom_yaw) - finger_y * math.sin(self._odom_yaw)
            fw_y = self._odom_y + finger_x * math.sin(self._odom_yaw) + finger_y * math.cos(self._odom_yaw)

            # Forward error (along robot heading)
            dx = cube_x - fw_x
            dy = cube_y - fw_y
            forward_err = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
            lateral_err = -dx * math.sin(self._odom_yaw) + dy * math.cos(self._odom_yaw)

            self.get_logger().warn(
                f'  [iter={iteration}] forward={forward_err*1000:+.1f}mm  '
                f'lateral={lateral_err*1000:+.1f}mm')

            if abs(forward_err) < 0.008:
                self.get_logger().warn('  ✅ Forward distance OK')
                return

            # Only correct forward — NO rotation
            twist = Twist()
            twist.linear.x = max(-0.05, min(0.05, forward_err * 0.8))
            self._publish_cmd_vel(twist)
            time.sleep(0.2)
            self._publish_cmd_vel(Twist())
            time.sleep(0.15)

        self.get_logger().warn('  ⚠️ Forward check finished')

    def _wait_for_pick_condition(self, timeout=30.0):
        """Wait until the cube meets the manufacturer pick condition.

        Manufacturer (ROS1) condition from autopilot_main.py Wrecker():
          abs(pixel_x - 320) < 10 AND pixel_y > 440

        This means the cube is centered horizontally and close enough vertically
        (near the bottom of the image = physically close to the gripper).

        Returns True if condition met, False if timeout.
        """
        if not CV_AVAILABLE or self._bridge is None:
            self.get_logger().warn('  _wait_for_pick_condition: cv_bridge unavailable')
            return False

        self.get_logger().warn('═══════ WAITING FOR PICK CONDITION ═══════')
        self.get_logger().warn('  Target: abs(pixel_x - 320) < 10 AND pixel_y > 440')

        t0 = time.monotonic()
        last_log = 0.0
        consecutive_good = 0

        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._wrist_image is None:
                time.sleep(0.05)
                continue

            try:
                hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]
                if not large:
                    consecutive_good = 0
                    continue

                largest = max(large, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                px = x + w // 2
                py = y + h // 2

                if time.monotonic() - last_log > 1.0:
                    self.get_logger().info(
                        f'  cube_pixel=({px},{py})  '
                        f'cx_err={px - 320}  '
                        f'py_ok={py > 440}')
                    last_log = time.monotonic()

                if abs(px - 320) < 10 and py > 440:
                    consecutive_good += 1
                    if consecutive_good >= 3:
                        self.get_logger().warn(
                            f'  ✅ PICK CONDITION MET: pixel=({px},{py})')
                        return True
                else:
                    consecutive_good = 0

            except Exception:
                consecutive_good = 0

        self.get_logger().warn(f'  ⚠️ Pick condition timeout after {timeout}s')
        return False

    def _get_cube_pixel(self):
        """Return the latest (pixel_x, pixel_y) of the blue cube in the wrist camera.

        Returns (None, None) if no cube detected.
        """
        if not CV_AVAILABLE or self._bridge is None or self._wrist_image is None:
            return None, None
        try:
            hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]
            if not large:
                return None, None
            largest = max(large, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            return x + w // 2, y + h // 2
        except Exception:
            return None, None

    def _quick_camera_check(self, timeout=5.0):
        """Quick check: is the blue cube roughly centered in the wrist camera?

        Returns True if abs(cx_err) < 15 for 2 consecutive frames.
        """
        if not CV_AVAILABLE or self._bridge is None:
            return False

        t0 = time.monotonic()
        consecutive = 0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._wrist_image is None:
                continue

            try:
                hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]
                if not large:
                    consecutive = 0
                    continue

                largest = max(large, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                width = self._wrist_image.shape[1]
                cx_err = cx - width // 2

                if abs(cx_err) < 15:
                    consecutive += 1
                    if consecutive >= 2:
                        return True
                else:
                    consecutive = 0
            except Exception:
                consecutive = 0

        return False

    def _lateral_center_gripper_on_cube(self, timeout=8.0):
        """Correct lateral (side-to-side) offset between gripper centre and cube using TF.

        After camera alignment the cube is centred in the wrist camera image,
        but the camera (on arm_link4) and gripper (on arm_link5) are at different
        physical positions.  This creates a systematic lateral offset that causes
        the gripper to miss the cube by ~2 cm.

        This method reads the actual finger-centre TF and cube TF, computes the
        lateral offset perpendicular to the robot heading, and rotates the base
        to eliminate it.  A short forward re-approach follows each rotation to
        restore the correct stand-off distance.
        """
        self.get_logger().warn('═══════ LATERAL GRIPPER CENTERING (TF) ═══════')

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        for iteration in range(8):
            # Get cube position from GT TF
            cube_x, cube_y = None, None
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.3))
                cube_x = t.transform.translation.x
                cube_y = t.transform.translation.y
            except Exception as e:
                self.get_logger().warn(f'  TF cube lookup failed: {e}')
                return

            # Get finger centre from TF
            finger_x, finger_y, finger_src = self._get_finger_center_x()
            if finger_src != 'finger_TF':
                self.get_logger().warn(
                    f'  Finger TF unavailable (src={finger_src}), skipping lateral centering')
                return

            # Finger world position (full 2D transform)
            fw_x = self._odom_x + finger_x * math.cos(self._odom_yaw) - finger_y * math.sin(self._odom_yaw)
            fw_y = self._odom_y + finger_x * math.sin(self._odom_yaw) + finger_y * math.cos(self._odom_yaw)

            # Lateral offset: perpendicular to heading
            dx = cube_x - fw_x
            dy = cube_y - fw_y
            lateral = -dx * math.sin(self._odom_yaw) + dy * math.cos(self._odom_yaw)
            forward_err = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)

            self.get_logger().warn(
                f'  [iter={iteration}] lateral={lateral*1000:+.1f}mm  '
                f'forward={forward_err*1000:+.1f}mm  finger_src={finger_src}')

            if abs(lateral) < 0.003 and abs(forward_err) < 0.008:
                self.get_logger().warn('  ✅ Lateral centering converged')
                return

            # Rotate toward cube to correct lateral offset — small, precise pulses
            target_yaw = math.atan2(cube_y - self._odom_y, cube_x - self._odom_x)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

            twist = Twist()
            twist.angular.z = max(-0.15, min(0.15, yaw_err * 1.5))
            self._publish_cmd_vel(twist)
            time.sleep(0.3)
            self._publish_cmd_vel(Twist())
            time.sleep(0.2)

            # Re-approach forward to restore stand-off
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            finger_x_now, finger_y_now, _ = self._get_finger_center_x()
            if finger_x_now is not None:
                fw_x_now = self._odom_x + finger_x_now * math.cos(self._odom_yaw) - finger_y_now * math.sin(self._odom_yaw)
                fw_y_now = self._odom_y + finger_x_now * math.sin(self._odom_yaw) + finger_y_now * math.cos(self._odom_yaw)
                fwd_now = (cube_x - fw_x_now) * math.cos(self._odom_yaw) + \
                          (cube_y - fw_y_now) * math.sin(self._odom_yaw)
                if abs(fwd_now) > 0.010:
                    twist = Twist()
                    twist.linear.x = max(-0.08, min(0.08, fwd_now * 1.0))
                    t_fwd = time.monotonic()
                    while time.monotonic() - t_fwd < 2.0:
                        rclpy.spin_once(self, timeout_sec=0.05)
                        self._publish_cmd_vel(twist)
                        time.sleep(0.05)
                    self._publish_cmd_vel(Twist())
                    time.sleep(0.15)

        self.get_logger().warn('  ⚠️ Lateral centering timed out')

    def _verify_and_correct_gripper_position(self):
        """After camera alignment, verify gripper-to-cube distance using TF and correct if needed.

        This is a small verification step after ROS1 camera alignment. The camera tells us
        the cube is "close enough" (pixel_y > 440) but doesn't give exact distance. TF gives
        us the ground-truth distance so we can do a small correction.

        The cube is 4cm wide. The gripper fingers have a ~5cm gap. We want the finger center
        to be within ~1-2cm of the cube center for a successful grasp.
        """
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        cube_x, cube_y = None, None
        try:
            t_c = self._tf_buffer.lookup_transform('odom', 'test_block', Time(), Duration(seconds=0.3))
            cube_x, cube_y = t_c.transform.translation.x, t_c.transform.translation.y
            self.get_logger().warn(f'  TF cube position: ({cube_x:.4f},{cube_y:.4f})')
        except Exception as e:
            self.get_logger().warn(f'  TF cube lookup failed: {e}')
            return

        finger_x, finger_y, finger_src = None, None, 'none'
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            finger_x, finger_y, finger_src = self._get_finger_center_x()
            if finger_src == 'finger_TF':
                break

        if finger_x is None or cube_x is None:
            self.get_logger().warn('  Cannot verify - no finger/cube TF')
            return

        finger_world_x = self._odom_x + finger_x * math.cos(self._odom_yaw) - finger_y * math.sin(self._odom_yaw)
        finger_world_y = self._odom_y + finger_x * math.sin(self._odom_yaw) + finger_y * math.cos(self._odom_yaw)
        dist_to_cube = math.hypot(cube_x - finger_world_x, cube_y - finger_world_y)

        self.get_logger().warn(
            f'  Distance check: finger_base={finger_x:.4f}m  '
            f'dist={dist_to_cube*1000:.0f}mm  src={finger_src}')

        # Target: finger center should be ~2cm past the cube center
        # Cube is 4cm wide, so we want finger tip just past the far edge
        # But since we're using camera alignment (which worked to ~2cm),
        # we just verify we're in the right ballpark
        target_dist = 0.02  # 2cm - close enough for 5cm finger gap
        tolerance = 0.015   # 1.5cm tolerance

        if dist_to_cube > target_dist + tolerance:
            correction = dist_to_cube - target_dist
            self.get_logger().warn(f'  Gripper too far from cube by {correction*1000:.0f}mm - correcting forward')
            twist = Twist()
            twist.linear.x = min(0.15, correction * 1.5)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and dist_to_cube > target_dist + tolerance * 0.5:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    t_c = self._tf_buffer.lookup_transform('odom', 'test_block', Time(), Duration(seconds=0.1))
                    cube_x, cube_y = t_c.transform.translation.x, t_c.transform.translation.y
                    finger_x_now, finger_y_now, _ = self._get_finger_center_x()
                    if finger_x_now is not None:
                        finger_world_x = self._odom_x + finger_x_now * math.cos(self._odom_yaw) - finger_y_now * math.sin(self._odom_yaw)
                        finger_world_y = self._odom_y + finger_x_now * math.sin(self._odom_yaw) + finger_y_now * math.cos(self._odom_yaw)
                        dist_to_cube = math.hypot(cube_x - finger_world_x, cube_y - finger_world_y)
                except Exception:
                    pass
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
            self._publish_cmd_vel(Twist())
            self.get_logger().warn(f'  Forward correction done. Final dist: {dist_to_cube*1000:.0f}mm')
        elif dist_to_cube < target_dist - tolerance:
            correction = target_dist - dist_to_cube
            self.get_logger().warn(f'  Gripper too close to cube by {correction*1000:.0f}mm - backing up')
            twist = Twist()
            twist.linear.x = -min(0.10, correction * 1.5)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and dist_to_cube < target_dist - tolerance * 0.5:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    t_c = self._tf_buffer.lookup_transform('odom', 'test_block', Time(), Duration(seconds=0.1))
                    cube_x, cube_y = t_c.transform.translation.x, t_c.transform.translation.y
                    finger_x_now, finger_y_now, _ = self._get_finger_center_x()
                    if finger_x_now is not None:
                        finger_world_x = self._odom_x + finger_x_now * math.cos(self._odom_yaw) - finger_y_now * math.sin(self._odom_yaw)
                        finger_world_y = self._odom_y + finger_x_now * math.sin(self._odom_yaw) + finger_y_now * math.cos(self._odom_yaw)
                        dist_to_cube = math.hypot(cube_x - finger_world_x, cube_y - finger_world_y)
                except Exception:
                    pass
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
            self._publish_cmd_vel(Twist())
            self.get_logger().warn(f'  Back up done. Final dist: {dist_to_cube*1000:.0f}mm')
        else:
            self.get_logger().warn(f'  ✅ Gripper well positioned: dist={dist_to_cube*1000:.0f}mm (target ~20mm)')

    def _final_gripper_micro_center(self):
        """One last precise lateral adjustment right before gripper closes.

        Reads finger-centre and cube TF, computes lateral offset, and applies
        a small rotation + forward nudge to get within 2 mm.
        """
        self.get_logger().warn('═══════ FINAL MICRO-CENTER (2 mm) ═══════')

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        for _ in range(3):
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.2))
                cube_x, cube_y = t.transform.translation.x, t.transform.translation.y
            except Exception:
                return

            finger_x, finger_y, finger_src = self._get_finger_center_x()
            if finger_src != 'finger_TF':
                return

            fw_x = self._odom_x + finger_x * math.cos(self._odom_yaw) - finger_y * math.sin(self._odom_yaw)
            fw_y = self._odom_y + finger_x * math.sin(self._odom_yaw) + finger_y * math.cos(self._odom_yaw)

            dx = cube_x - fw_x
            dy = cube_y - fw_y
            lateral = -dx * math.sin(self._odom_yaw) + dy * math.cos(self._odom_yaw)
            forward_err = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)

            self.get_logger().warn(
                f'  lateral={lateral*1000:+.1f}mm  forward={forward_err*1000:+.1f}mm')

            if abs(lateral) < 0.002 and abs(forward_err) < 0.005:
                self.get_logger().warn('  ✅ Micro-center converged')
                return

            # Tiny rotation pulse
            target_yaw = math.atan2(cube_y - self._odom_y, cube_x - self._odom_x)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)
            twist = Twist()
            twist.angular.z = max(-0.08, min(0.08, yaw_err * 1.0))
            self._publish_cmd_vel(twist)
            time.sleep(0.2)
            self._publish_cmd_vel(Twist())
            time.sleep(0.15)

            # Tiny forward nudge
            if abs(forward_err) > 0.005:
                twist = Twist()
                twist.linear.x = max(-0.04, min(0.04, forward_err * 0.8))
                self._publish_cmd_vel(twist)
                time.sleep(0.2)
                self._publish_cmd_vel(Twist())
                time.sleep(0.1)

        self.get_logger().warn('  ⚠️ Micro-center finished (may not fully converge)')

    def _verify_pickup(self, obj_map):
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            grip_pos = js.get('grip_joint', None)
            if grip_pos is not None:
                if abs(grip_pos - GRIPPER_HOLD) < 0.3:
                    return True
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'test_block', Time(), Duration(seconds=0.5))
            cube_x = t.transform.translation.x
            cube_y = t.transform.translation.y
            dist = math.hypot(cube_x - obj_map.pose.position.x, cube_y - obj_map.pose.position.y)
            return dist > 0.3
        except Exception:
            return True

    def _gripper_center_x_at_joints(self, joints):
        """Compute gripper center X position (base_link frame) from joint angles.
        
        Improved model that accounts for arm_joint5 (j5) rotation effect on reach.
        The wrist camera (mono_link) is on arm_link4, gripper is on arm_link5.
        When j5 rotates, the effective forward distance of the gripper changes due to
        arm_link5 position shifting. At REACH_DOWN (j5≈-0.54), this adds ~26mm offset.
        
        Calibration basis:
        - CENTER_REF (0.3032 m): gripper center reach at reference pose
        - arm_link4→arm_link5 distance ≈ 0.17455 m (y-component from URDF)
        - j5 angle effect: cos(j5) component of the rotated reach
        """
        j1, j2, j3, j4, j5 = joints
        J2_REF = -1.45
        J3_REF = -0.180
        J5_REF = 0.0  # Reference is j5=0 (arm_link5 aligned with arm_link4)
        
        CENTER_REF = 0.3032
        dX_dJ2 = 0.150 * math.cos(J2_REF)
        dX_dJ3 = 0.145 * math.cos(J3_REF)
        
        # J5 effect: arm_link5 is 0.17455 m away from arm_link4 (URDF joint xyz).
        # When j5 rotates, the forward component changes by this distance × (cos(j5) - cos(j5_ref))
        # This accounts for the parallax between camera (on link4) and gripper (on link5).
        ARM_LINK5_REACH = 0.17455
        dX_dJ5 = ARM_LINK5_REACH * (math.cos(j5) - math.cos(J5_REF))
        
        return CENTER_REF + dX_dJ2 * (j2 - J2_REF) + dX_dJ3 * (j3 - J3_REF) + dX_dJ5

    def _get_finger_center_x(self):
        """Get finger centre position in base_link frame from actual TF.

        Uses rlink2 / llink2 finger link TFs directly (midpoint), which
        correctly accounts for all joint rotations via the URDF kinematic
        chain.  Falls back to FK model if TFs are unavailable.
        Returns (finger_center_x, finger_center_y, source_string).
        """
        try:
            rt = self._tf_buffer.lookup_transform(
                'base_link', 'rlink2', Time(), Duration(seconds=0.3))
            lt = self._tf_buffer.lookup_transform(
                'base_link', 'llink2', Time(), Duration(seconds=0.3))
            cx = (rt.transform.translation.x + lt.transform.translation.x) / 2.0
            cy = (rt.transform.translation.y + lt.transform.translation.y) / 2.0
            return cx, cy, 'finger_TF'
        except Exception:
            pass
        # Fallback: arm_link5 + offset (less accurate at non-zero joint angles)
        try:
            t = self._tf_buffer.lookup_transform(
                'base_link', 'arm_link5', Time(), Duration(seconds=0.3))
            return t.transform.translation.x + ARM5_TO_GRIP_CENTER, 0.0, 'arm5_TF'
        except Exception:
            pass
        return (self._gripper_center_x_at_joints(REACH_DOWN)
                - FK_SETTLE_COMPENSATION, 0.0, 'FK_fallback')

    def _cube_in_odom(self):
        """Return the latest detected cube pose expressed in the 'odom' frame.

        `_detected_pose_map` is stored in the 'map' frame (see
        `_on_detected_object`). The robot pose (`_odom_x / _odom_y /
        _odom_yaw`) is in the 'odom' frame. Nav2 / AMCL publishes a non-identity
        `map -> odom` transform, so mixing the two directly produces a vector
        that points to the wrong place. This helper bridges the two frames via
        TF so downstream code can compare apples to apples.

        Returns a (x, y) tuple in odom, or (None, None) if no detection is
        available yet or the TF lookup fails.
        """
        if self._detected_pose_map is None:
            return None, None
        cube = self._detected_pose_map
        try:
            if cube.header.frame_id == 'odom':
                return cube.pose.position.x, cube.pose.position.y
            t = self._tf_buffer.lookup_transform(
                'odom', cube.header.frame_id,
                Time(), Duration(seconds=0.5))
            cube_in_odom = tf2_geometry_msgs.do_transform_pose_stamped(cube, t)
            return cube_in_odom.pose.position.x, cube_in_odom.pose.position.y
        except Exception as e:
            self.get_logger().warn(f'_cube_in_odom: TF lookup failed: {e}')
            return None, None

    def _pixel_pid_navigate(self, timeout=60.0):
        """Drive toward cube using pixel-based PID (manufacturer approach).

        Manufacturer (ROS1) uses PID on pixel position from wrist camera:
          - angular_z proportional to (pixel_x - 320) / 60
          - linear_x based on pixel_y (higher = closer):
              pixel_y < 250: drive fast (0.20 m/s)
              pixel_y < 350: drive medium (0.12 m/s)
              pixel_y < 440: drive slow (0.06 m/s)
              pixel_y >= 440: stop (close enough)

        Stops when pick condition met: abs(pixel_x - 320) < 10 AND pixel_y > 440
        """
        if not CV_AVAILABLE or self._bridge is None:
            self.get_logger().warn('  _pixel_pid_navigate: cv_bridge unavailable')
            return False

        self.get_logger().warn('═══════ PIXEL PID NAVIGATION ═══════')
        self.get_logger().warn('  Target: abs(pixel_x - 320) < 10 AND pixel_y > 440')

        t0 = time.monotonic()
        last_log = 0.0
        consecutive_good = 0

        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._wrist_image is None:
                # No image — drive straight slowly
                twist = Twist()
                twist.linear.x = 0.10
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
                continue

            try:
                hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]
                if not large:
                    # No cube in view — rotate slowly to search
                    twist = Twist()
                    twist.angular.z = 0.3
                    self._publish_cmd_vel(twist)
                    if time.monotonic() - last_log > 2.0:
                        self.get_logger().info('  No cube in view — searching...')
                        last_log = time.monotonic()
                    continue

                largest = max(large, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                px = x + w // 2
                py = y + h // 2

                if time.monotonic() - last_log > 0.5:
                    self.get_logger().info(
                        f'  cube_pixel=({px},{py})  '
                        f'cx_err={px - 320}  '
                        f'odom=({self._odom_x:.2f},{self._odom_y:.2f})')
                    last_log = time.monotonic()

                # Check pick condition
                if abs(px - 320) < 10 and py > 440:
                    consecutive_good += 1
                    if consecutive_good >= 3:
                        self.get_logger().warn(
                            f'  ✅ Pick condition MET: pixel=({px},{py})')
                        self._publish_cmd_vel(Twist())
                        return True
                else:
                    consecutive_good = 0

                # PID drive (manufacturer logic)
                twist = Twist()

                # Rotate toward cube (proportional control)
                if abs(px - 320) > 8:
                    ang_vel = max(-0.5, min(0.5, (px - 320) / 60.0))
                    twist.angular.z = ang_vel

                # Drive forward when roughly aligned
                if abs(px - 320) < 30:
                    if py < 250:
                        twist.linear.x = 0.20  # far
                    elif py < 350:
                        twist.linear.x = 0.12  # medium
                    elif py < 440:
                        twist.linear.x = 0.06  # close

                self._publish_cmd_vel(twist)

            except Exception as e:
                if time.monotonic() - last_log > 1.0:
                    self.get_logger().warn(f'  Navigation error: {e}')
                    last_log = time.monotonic()
                self._publish_cmd_vel(Twist())

        self.get_logger().warn(f'  ⚠️ Pixel PID navigation timeout after {timeout}s')
        self._publish_cmd_vel(Twist())
        return False

    def _yaw_align_to_cube(self, timeout=15.0):
        """Rotate the base in place until it faces the cube (GT TF).

        Called at the start of APPROACH to eliminate large heading errors
        (e.g. 180°) before the visual servo loop begins.  Without this,
        the servo loop would lose TF mid-turn and abort with the robot
        facing the wrong direction.
        """
        self.get_logger().warn('═══════ PRE-APPROACH YAW ALIGNMENT ═══════')

        # Get cube position from GT TF
        cube_x, cube_y = None, None
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.2))
                cube_x = t.transform.translation.x
                cube_y = t.transform.translation.y
                break
            except Exception:
                pass

        if cube_x is None:
            self.get_logger().warn('  Cube TF unavailable — skipping yaw alignment')
            return

        self.get_logger().warn(
            f'  cube=({cube_x:.3f},{cube_y:.3f})  '
            f'robot=({self._odom_x:.3f},{self._odom_y:.3f})  '
            f'yaw={math.degrees(self._odom_yaw):.0f}°')

        t0 = time.monotonic()
        last_log = 0.0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)

            # Refresh cube TF periodically
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(), Duration(seconds=0.1))
                cube_x = t.transform.translation.x
                cube_y = t.transform.translation.y
            except Exception:
                pass

            dx = cube_x - self._odom_x
            dy = cube_y - self._odom_y
            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

            if time.monotonic() - last_log > 0.5:
                self.get_logger().info(
                    f'  yaw_align: target_yaw={math.degrees(target_yaw):.0f}°  '
                    f'current_yaw={math.degrees(self._odom_yaw):.0f}°  '
                    f'err={math.degrees(yaw_err):+.1f}°')
                last_log = time.monotonic()

            if abs(yaw_err) < math.radians(5.0):
                self.get_logger().warn(
                    f'  ✅ Yaw aligned: err={math.degrees(yaw_err):+.1f}°')
                break

            twist = Twist()
            twist.angular.z = max(-0.8, min(0.8, yaw_err * 2.5))
            self._publish_cmd_vel(twist)
            time.sleep(0.02)

        # Full stop
        self._publish_cmd_vel(Twist())
        for _ in range(5):
            self._publish_cmd_vel(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.2)

    def _camera_guided_approach(self):
        """Closed-loop approach: align the gripper centre with the cube.

        CRITICAL FIX: Ground-truth ``test_block`` TF (published by Gazebo's
        PosePublisher plugin, relayed into the ROS TF tree) is used as the
        PRIMARY cube position source throughout the servo loop.  Camera
        detection is kept only as a fallback.

        Background on the 90 ° turn bug (now fixed):
          At ≤0.3 m range the front depth camera is at height ~0.42 m and
          views the cube at a steep ~40 ° downward angle.  The depth sensor
          may hit the floor behind the cube instead of the cube's face, and
          the centroid pixel is near the image bottom-edge with a large
          lateral offset.  The resulting 3-D back-projection placed the cube
          ~90 ° to the side of the robot in the odom frame, causing the
          visual servo to rotate the robot AWAY from the cube before the arm
          tried to pick.  By using the Gazebo GT TF instead of the camera
          reading for position, this geometry-induced error is completely
          eliminated.

        Pipeline:
          Phase A+B — GT-TF servo: yaw-align + forward approach.  The GT TF
            is refreshed every loop iteration so AMCL drift and camera noise
            cannot accumulate into a heading error.
          Phase C   — Precision yaw alignment (≤1.5 °, GT TF only): a
            dedicated yaw-only loop that eliminates any residual heading
            error before the arm descends.
          Phase D   — Final X centering (original Phase C, inherited):
            nudge the base so the FK finger centre lands exactly on the cube.
        """
        finger_center = self._gripper_center_x_at_joints(REACH_DOWN) - FK_SETTLE_COMPENSATION

        # ── Get initial cube position: GT TF > _object_pose_map > camera ──────
        cube_ox, cube_oy = None, None

        # Priority 1: ground-truth test_block TF (immune to depth-sensor noise)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf_blk = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(),
                    Duration(seconds=0.3))
                cube_ox = tf_blk.transform.translation.x
                cube_oy = tf_blk.transform.translation.y
                self.get_logger().info(
                    f'  Initial cube pos from GT TF: ({cube_ox:.3f},{cube_oy:.3f})')
                break
            except Exception:
                time.sleep(0.05)

        # Priority 2: _object_pose_map (refreshed from GT TF after NAVIGATE,
        #   already expressed in odom — no extra transform needed)
        if cube_ox is None and self._object_pose_map is not None:
            if self._object_pose_map.header.frame_id == 'odom':
                cube_ox = self._object_pose_map.pose.position.x
                cube_oy = self._object_pose_map.pose.position.y
                self.get_logger().info(
                    f'  Initial cube pos from _object_pose_map: '
                    f'({cube_ox:.3f},{cube_oy:.3f})')

        # Priority 3: camera detection (last resort fallback)
        if cube_ox is None:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                cube_ox, cube_oy = self._cube_in_odom()
                if cube_ox is not None and cube_oy is not None:
                    self.get_logger().info(
                        f'  Initial cube pos from camera: '
                        f'({cube_ox:.3f},{cube_oy:.3f})')
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

        if cube_ox is None or cube_oy is None:
            self.get_logger().warn('Camera guided approach: no cube position available')
            return False

        # Hold the last known position for fallback during brief TF gaps
        last_good_cube = (cube_ox, cube_oy, time.monotonic())

        # ── Phase A + B: yaw-align + forward approach (GT TF servo) ──────────
        self.get_logger().warn('═══════ VISUAL SERVOING: align + approach ═══════')
        self.get_logger().warn(
            f'  initial cube odom=({cube_ox:.3f},{cube_oy:.3f})  '
            f'robot odom=({self._odom_x:.3f},{self._odom_y:.3f})  '
            f'yaw={math.degrees(self._odom_yaw):.0f}°  '
            f'finger_reach={finger_center*1000:.0f}mm'
        )

        t0 = time.monotonic()
        timeout = 25.0
        last_log = 0.0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)

            # Refresh cube position — GT TF every iteration (prevents stale
            # or noise-corrupted reads from the close-range depth sensor)
            try:
                tf_blk = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(),
                    Duration(seconds=0.05))
                cube_ox = tf_blk.transform.translation.x
                cube_oy = tf_blk.transform.translation.y
                last_good_cube = (cube_ox, cube_oy, time.monotonic())
            except Exception:
                # NEVER fall back to camera at close range — the depth
                # back-projection is unreliable (90° offset bug).  Use
                # the last known GT TF position instead.
                age = time.monotonic() - last_good_cube[2]
                if age > 5.0:
                    self.get_logger().warn(
                        f'Camera guided: GT TF lost for '
                        f'{age:.1f}s, stopping servo')
                    break
                cube_ox, cube_oy, _ = last_good_cube

            # Yaw error (heading → cube direction)
            dx = cube_ox - self._odom_x
            dy = cube_oy - self._odom_y
            
            dist_to_cube = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

            # Forward distance projected on heading
            forward = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
            dist_err = forward - finger_center

            # Log every 0.5 s
            if time.monotonic() - last_log > 0.5:
                lat = -dx * math.sin(self._odom_yaw) + dy * math.cos(self._odom_yaw)
                self.get_logger().info(
                    f'  servo: forward={forward*1000:.0f}mm '
                    f'target={finger_center*1000:.0f}mm '
                    f'dist_err={dist_err*1000:+.0f}mm '
                    f'lateral={lat*1000:+.0f}mm '
                    f'yaw_err={math.degrees(yaw_err):+.1f}°')
                last_log = time.monotonic()

            # Termination: within 8 mm in distance AND 2° in heading.
            if abs(dist_err) < 0.008 and abs(yaw_err) < math.radians(2.0):
                self.get_logger().warn(
                    f'  ✅ visual servo converged: '
                    f'dist_err={dist_err*1000:+.0f}mm '
                    f'yaw_err={math.degrees(yaw_err):+.1f}°')
                break

            twist = Twist()
            # Always correct yaw (cheap; needed for accurate forward projection)
            twist.angular.z = max(-0.6, min(0.6, yaw_err * 2.0))
            # Only drive forward/backward if heading is roughly aligned,
            # otherwise the forward projection is meaningless.
            if abs(yaw_err) < math.radians(20.0):
                twist.linear.x = max(-0.20, min(0.20, dist_err * 1.5))
            else:
                twist.linear.x = 0.0
            self._publish_cmd_vel(twist)

        # Full stop
        self._publish_cmd_vel(Twist())
        for _ in range(5):
            self._publish_cmd_vel(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.2)

        # ── Phase C: precision yaw alignment (GT TF, ≤1.5 °) ─────────────────
        # After the translational servo, run a dedicated yaw-only loop that
        # tightens heading to ≤1.5 ° before the arm descends.  Any residual
        # error from the servo (e.g. convergence hysteresis at 2 °) is
        # corrected here so the gripper drops straight onto the cube.
        self.get_logger().warn('═══════ PRECISION YAW ALIGNMENT ═══════')
        t_yaw = time.monotonic()
        while time.monotonic() - t_yaw < 6.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            cube_fx, cube_fy = cube_ox, cube_oy  # last known fallback
            try:
                tf_blk = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(),
                    Duration(seconds=0.2))
                cube_fx = tf_blk.transform.translation.x
                cube_fy = tf_blk.transform.translation.y
            except Exception:
                pass
            dx = cube_fx - self._odom_x
            dy = cube_fy - self._odom_y
            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)
                
            if abs(yaw_err) < math.radians(1.5):
                self.get_logger().warn(
                    f'  ✅ precision yaw aligned: err={math.degrees(yaw_err):+.1f}°')
                break
            twist = Twist()
            twist.angular.z = max(-0.3, min(0.3, yaw_err * 3.0))
            self._publish_cmd_vel(twist)
        for _ in range(5):
            self._publish_cmd_vel(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.2)

        # ── Phase D: final X centering using ground-truth cube TF ────────────
        # (Inherited from original Phase C: nudge the base forward/back so
        # the FK finger centre lands exactly on the cube.)
        cube_world_x = None
        cube_world_y = None
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'test_block', Time(),
                Duration(seconds=0.5))
            cube_world_x = t.transform.translation.x
            cube_world_y = t.transform.translation.y
        except Exception:
            # Fallback to _object_pose_map (refreshed from GT TF after NAVIGATE)
            if self._object_pose_map is not None:
                if self._object_pose_map.header.frame_id == 'odom':
                    cube_world_x = self._object_pose_map.pose.position.x
                    cube_world_y = self._object_pose_map.pose.position.y
                else:
                    try:
                        t_map = self._tf_buffer.lookup_transform(
                            'odom', self._object_pose_map.header.frame_id,
                            Time(), Duration(seconds=0.3))
                        p_odom = tf2_geometry_msgs.do_transform_pose_stamped(
                            self._object_pose_map, t_map)
                        cube_world_x = p_odom.pose.position.x
                        cube_world_y = p_odom.pose.position.y
                    except Exception:
                        pass
            if cube_world_x is None:
                self.get_logger().warn('test_block TF and _object_pose_map unavailable; skipping final X centering')

        if cube_world_x is not None and cube_world_y is not None:
            self.get_logger().warn('═══════ FINAL X CENTERING (test_block TF) ═══════')
            self.get_logger().warn(
                f'  cube_world=({cube_world_x:.4f},{cube_world_y:.4f})  '
                f'robot odom=({self._odom_x:.4f},{self._odom_y:.4f})')

            # Re-derive finger reach from current arm configuration so the
            # controller targets the actual gripper, not a stale FK value.
            actual_finger_base, actual_finger_lat, finger_src = self._get_finger_center_x()
            self.get_logger().warn(
                f'  Phase D finger_center=({actual_finger_base:.5f}, {actual_finger_lat:.5f}) m '
                f'(source={finger_src})')

            finger_target_x = cube_world_x
            finger_target_y = cube_world_y
            # Back out the desired base position so the finger lands on the cube.
            desired_base_x = finger_target_x - actual_finger_base * math.cos(self._odom_yaw) + actual_finger_lat * math.sin(self._odom_yaw)
            desired_base_y = finger_target_y - actual_finger_base * math.sin(self._odom_yaw) - actual_finger_lat * math.cos(self._odom_yaw)

            t0 = time.monotonic()
            while time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.02)
                bx = desired_base_x - self._odom_x
                by = desired_base_y - self._odom_y
                err = math.hypot(bx, by)
                if err < 0.005:
                    break
                # Drive only in the heading-projected forward direction so we
                # do not introduce lateral drift.
                forward_cmd = bx * math.cos(self._odom_yaw) + by * math.sin(self._odom_yaw)
                twist = Twist()
                twist.linear.x = max(-0.10, min(0.10, forward_cmd * 1.5))
                self._publish_cmd_vel(twist)

            self._publish_cmd_vel(Twist())
            for _ in range(5):
                self._publish_cmd_vel(Twist())
                rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.2)

            # Log final alignment
            finger_world_x = self._odom_x + actual_finger_base * math.cos(self._odom_yaw)
            finger_world_y = self._odom_y + actual_finger_base * math.sin(self._odom_yaw)
            final_err = math.hypot(
                finger_world_x - cube_world_x, finger_world_y - cube_world_y)
            self.get_logger().warn(
                f'  final finger_world=({finger_world_x:.4f},{finger_world_y:.4f}) '
                f'cube=({cube_world_x:.4f},{cube_world_y:.4f}) '
                f'misalignment={final_err*1000:.1f}mm')

        # ── Phase E: LATERAL CENTERING (gripper ↔ cube) ──────────────────
        # Explicitly correct the side-to-side offset between the gripper
        # finger centre and the cube centre.  Previous phases only corrected
        # forward distance and yaw heading; any residual yaw error translates
        # into a lateral offset that causes the gripper to miss the cube.
        # This phase iteratively: (1) measures the lateral offset, (2) rotates
        # the robot to face the cube, and (3) re-approaches forward — repeating
        # until the lateral offset is within the 3 mm tolerance.
        self.get_logger().warn('═══════ PHASE E: LATERAL CENTERING ═══════')
        for _lat_iter in range(5):
            # Refresh cube position from GT TF
            cw_x, cw_y = None, None
            try:
                t_lat = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(),
                    Duration(seconds=0.2))
                cw_x = t_lat.transform.translation.x
                cw_y = t_lat.transform.translation.y
            except Exception:
                break

            # Current finger centre in world frame (full 2D transform)
            fw_x = self._odom_x + actual_finger_base * math.cos(self._odom_yaw) - actual_finger_lat * math.sin(self._odom_yaw)
            fw_y = self._odom_y + actual_finger_base * math.sin(self._odom_yaw) + actual_finger_lat * math.cos(self._odom_yaw)

            # Lateral offset: perpendicular to heading
            dx_lat = cw_x - fw_x
            dy_lat = cw_y - fw_y
            lateral = -dx_lat * math.sin(self._odom_yaw) + dy_lat * math.cos(self._odom_yaw)
            forward_err = dx_lat * math.cos(self._odom_yaw) + dy_lat * math.sin(self._odom_yaw)

            self.get_logger().warn(
                f'  [lat] iter={_lat_iter}  lateral={lateral*1000:+.1f}mm  '
                f'forward_err={forward_err*1000:+.1f}mm')

            if abs(lateral) < 0.003 and abs(forward_err) < 0.005:
                self.get_logger().warn('  ✅ lateral centering converged')
                break

            # If lateral offset is significant, rotate toward the cube then
            # drive forward to re-approach.  The rotation angle is chosen so
            # that one rotation step reduces the lateral offset to near zero
            # at the current forward distance.
            if abs(lateral) > 0.003:
                target_yaw_lat = math.atan2(
                    cw_y - self._odom_y, cw_x - self._odom_x)
                yaw_err_lat = self._normalize_angle(
                    target_yaw_lat - self._odom_yaw)
                # Apply yaw correction (small, controlled)
                twist = Twist()
                twist.angular.z = max(-0.20, min(0.20, yaw_err_lat * 2.0))
                self._publish_cmd_vel(twist)
                time.sleep(0.4)
                self._publish_cmd_vel(Twist())
                time.sleep(0.15)

            # After rotation, drive forward to re-approach the cube
            if abs(forward_err) > 0.005:
                twist = Twist()
                twist.linear.x = max(-0.08, min(0.08, forward_err * 1.0))
                t_fwd = time.monotonic()
                while time.monotonic() - t_fwd < 1.5:
                    rclpy.spin_once(self, timeout_sec=0.02)
                    # Refresh forward error
                    try:
                        t_f = self._tf_buffer.lookup_transform(
                            'odom', 'test_block', Time(),
                            Duration(seconds=0.1))
                        cwx = t_f.transform.translation.x
                        cwy = t_f.transform.translation.y
                    except Exception:
                        cwx, cwy = cw_x, cw_y
                    fwx = self._odom_x + actual_finger_base * math.cos(self._odom_yaw) - actual_finger_lat * math.sin(self._odom_yaw)
                    fwy = self._odom_y + actual_finger_base * math.sin(self._odom_yaw) + actual_finger_lat * math.cos(self._odom_yaw)
                    fwd = (cwx - fwx) * math.cos(self._odom_yaw) + (cwy - fwy) * math.sin(self._odom_yaw)
                    if abs(fwd) < 0.005:
                        break
                    twist.linear.x = max(-0.08, min(0.08, fwd * 1.0))
                    self._publish_cmd_vel(twist)
                self._publish_cmd_vel(Twist())
                time.sleep(0.15)

        # Final stop
        self._publish_cmd_vel(Twist())
        for _ in range(5):
            self._publish_cmd_vel(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.15)

        # Log final lateral offset for diagnostics
        try:
            t_final = self._tf_buffer.lookup_transform(
                'odom', 'test_block', Time(),
                Duration(seconds=0.2))
            fcw_x = t_final.transform.translation.x
            fcw_y = t_final.transform.translation.y
            ffw_x = self._odom_x + actual_finger_base * math.cos(self._odom_yaw) - actual_finger_lat * math.sin(self._odom_yaw)
            ffw_y = self._odom_y + actual_finger_base * math.sin(self._odom_yaw) + actual_finger_lat * math.cos(self._odom_yaw)
            fdx = fcw_x - ffw_x
            fdy = fcw_y - ffw_y
            lat_final = -fdx * math.sin(self._odom_yaw) + fdy * math.cos(self._odom_yaw)
            fwd_final = fdx * math.cos(self._odom_yaw) + fdy * math.sin(self._odom_yaw)
            self.get_logger().warn(
                f'  [lat] FINAL: lateral={lat_final*1000:+.1f}mm  '
                f'forward={fwd_final*1000:+.1f}mm')
        except Exception:
            pass

        self.get_logger().warn('═══════ VISUAL SERVOING COMPLETE ═══════')
        return True

    def _correct_robot_x_during_pick(self, obj_world_x, max_correction=0.25):
        """Closed-loop drive in robot-forward direction to align finger over cube.

        Yaw-aware: projects the robot→cube vector onto the heading and drives
        until the projection equals the actual TF finger reach. Uses TF as
        ground truth, not the unreliable FK model.
        """
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # Primary source: actual TF-based finger position
        finger_center_base, finger_center_lat, finger_src = self._get_finger_center_x()
        if finger_src != 'finger_TF':
            # Fallback to FK if TF unavailable
            joints = REACH_DOWN
            finger_center_base = self._gripper_center_x_at_joints(joints) - FK_SETTLE_COMPENSATION
            finger_center_lat = 0.0
            self.get_logger().warn(
                f'  _correct_robot_x: TF unavailable, using FK finger={finger_center_base:.4f}m')
        else:
            self.get_logger().warn(
                f'  _correct_robot_x: TF finger=({finger_center_base:.4f}, {finger_center_lat:.4f})m')

        # Recover cube y from test_block TF (most reliable)
        obj_world_y = self._odom_y
        try:
            tt = self._tf_buffer.lookup_transform(
                'odom', 'test_block', Time(),
                Duration(seconds=0.3))
            obj_world_y = tt.transform.translation.y
        except Exception:
            pass

        twist = Twist()
        t0 = time.monotonic()
        deadline = t0 + 4.0
        last_log = t0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            # Re-fetch cube position from TF each iteration
            try:
                tt = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', Time(),
                    Duration(seconds=0.1))
                obj_world_x = tt.transform.translation.x
                obj_world_y = tt.transform.translation.y
            except Exception:
                pass

            # Re-fetch actual finger position from TF each iteration
            curr_finger, curr_lat, curr_src = self._get_finger_center_x()
            if curr_src == 'finger_TF':
                finger_center_base = curr_finger

            dx = obj_world_x - self._odom_x
            dy = obj_world_y - self._odom_y
            forward = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
            err = forward - finger_center_base
            if time.monotonic() - last_log > 0.3:
                self.get_logger().warn(
                    f'  _correct_robot_x: forward={forward*1000:.1f}mm  '
                    f'target={finger_center_base*1000:.1f}mm  err={err*1000:.1f}mm')
                last_log = time.monotonic()
            if abs(err) < 0.008:
                break
            # Clamp single-step correction - increased speed
            err_clamped = max(-max_correction, min(max_correction, err))
            twist.linear.x = max(-0.20, min(0.20, err_clamped * 1.5))
            self._publish_cmd_vel(twist)
            time.sleep(0.02)

        stop_twist = Twist()
        for _ in range(5):
            self._publish_cmd_vel(stop_twist)
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def _backup_and_strafe(self):
        self.get_logger().info('Safety maneuver: backup + turn')
        twist = Twist()
        twist.linear.x = -0.3
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            self._publish_cmd_vel(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        twist.linear.x = 0.0
        twist.angular.z = 1.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.35:
            self._publish_cmd_vel(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        twist.angular.z = 0.0
        self._publish_cmd_vel(twist)
        time.sleep(0.2)

    def _drive_to_pose_xy(self, target_x, target_y, stop_dist=0.15,
                         max_lin=0.4, max_ang=1.0, timeout=60.0,
                         log_prefix='  Drive',
                         target_tf_frame=None,
                         target_pose_attr=None):
        """Closed-loop drive to (target_x, target_y) in the odom frame.

        Continuously corrects yaw while driving forward. Stops when within
        ``stop_dist`` of the target. Yaw-aware: works at any starting heading.

        If ``target_tf_frame`` is provided, the target is refreshed each
        iteration by looking up that frame in 'odom' (lets us follow a moving
        ground-truth target like Gazebo's test_block, immune to wheel slip).
        If ``target_pose_attr`` is provided, falls back to that attribute
        (a PoseStamped already in 'odom') if the TF lookup fails.
        """
        twist = Twist()
        deadline = time.monotonic() + timeout
        last_log = 0.0
        stuck_counter = 0  # Track iterations without progress
        last_pos_x = self._odom_x
        last_pos_y = self._odom_y
        
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

            # Live target update.
            if target_tf_frame is not None:
                try:
                    t = self._tf_buffer.lookup_transform(
                        'odom', target_tf_frame, Time(),
                        Duration(seconds=0.05))
                    target_x = t.transform.translation.x
                    target_y = t.transform.translation.y
                except Exception:
                    if target_pose_attr is not None:
                        p = getattr(self, target_pose_attr, None)
                        if p is not None:
                            if p.header.frame_id != 'odom':
                                try:
                                    t = self._tf_buffer.lookup_transform(
                                        'odom', p.header.frame_id,
                                        Time(),
                                        Duration(seconds=0.05))
                                    p_odom = tf2_geometry_msgs.do_transform_pose_stamped(p, t)
                                    target_x = p_odom.pose.position.x
                                    target_y = p_odom.pose.position.y
                                except Exception:
                                    target_x = p.pose.position.x
                                    target_y = p.pose.position.y
                            else:
                                target_x = p.pose.position.x
                                target_y = p.pose.position.y

            dx = target_x - self._odom_x
            dy = target_y - self._odom_y
            dist = math.hypot(dx, dy)
            if dist <= stop_dist:
                break

            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

            # Progress detection: break if stuck rotating at large yaw_err
            motion = math.hypot(self._odom_x - last_pos_x, self._odom_y - last_pos_y)
            if abs(yaw_err) > 0.8 and motion < 0.005:  # Not moving, large error
                stuck_counter += 1
                if stuck_counter > 50:  # ~1 second of no progress
                    self.get_logger().warn(
                        f'{log_prefix}: STUCK rotating at yaw_err={math.degrees(yaw_err):.0f}°, '
                        f'exiting (progress={motion:.4f}m)')
                    break
            else:
                stuck_counter = 0
            last_pos_x, last_pos_y = self._odom_x, self._odom_y

            # If facing significantly off, rotate in place first.
            if abs(yaw_err) > 0.6:
                twist.linear.x = 0.0
            else:
                # Slow as we approach the target / when off-yaw.
                slow = max(0.2, math.cos(yaw_err))
                lin = max_lin * slow
                if dist < 0.5:
                    lin = min(lin, 0.2)
                if dist < 0.2:
                    lin = min(lin, 0.08)
                twist.linear.x = lin

            twist.angular.z = max(-max_ang, min(max_ang, yaw_err * 1.5))
            self._publish_cmd_vel(twist)

            if time.monotonic() - last_log > 0.5:
                self.get_logger().info(
                    f'{log_prefix}: odom=({self._odom_x:.2f},{self._odom_y:.2f}) '
                    f'yaw={math.degrees(self._odom_yaw):.0f}° '
                    f'dist={dist:.2f} yaw_err={math.degrees(yaw_err):.0f}°')
                last_log = time.monotonic()

            time.sleep(0.02)

        # Full stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(5):
            self._publish_cmd_vel(twist)
            time.sleep(0.05)
        # Final state
        try:
            final_dist = math.hypot(target_x - self._odom_x,
                                    target_y - self._odom_y)
        except Exception:
            final_dist = -1.0
        self.get_logger().info(
            f'{log_prefix}: STOPPED at odom=({self._odom_x:.2f},{self._odom_y:.2f}) '
            f'yaw={math.degrees(self._odom_yaw):.0f}° final_dist={final_dist:.2f}m '
            f'(stop_dist={stop_dist:.2f})')

    def _drive_to_target(self, target, target_dist=0.15, speed=0.3):
        # Backwards-compat shim: delegate to the unified helper.
        self._drive_to_pose_xy(
            target.pose.position.x, target.pose.position.y,
            stop_dist=target_dist, max_lin=speed, max_ang=1.0, timeout=60.0,
            log_prefix='  Transport')

    def _align_yaw_to_target(self, target_map):
        robot_pose = self._get_robot_pose_in_odom()
        if robot_pose is None:
            return
        dx = target_map.pose.position.x - robot_pose.pose.position.x
        dy = target_map.pose.position.y - robot_pose.pose.position.y
        target_yaw = math.atan2(dy, dx)
        current_yaw = self._yaw_from_quat(robot_pose.pose.orientation)
        yaw_err = self._normalize_angle(target_yaw - current_yaw)

        twist = Twist()
        kp = 1.5
        max_omega = 1.0
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            robot_pose = self._get_robot_pose_in_odom()
            if robot_pose is None:
                break
            current_yaw = self._yaw_from_quat(robot_pose.pose.orientation)
            yaw_err = self._normalize_angle(target_yaw - current_yaw)
            if abs(yaw_err) < 0.05:
                break
            twist.angular.z = max(-max_omega, min(max_omega, kp * yaw_err))
            self._publish_cmd_vel(twist)
            time.sleep(0.05)

        twist.angular.z = 0.0
        self._publish_cmd_vel(twist)

    def _get_robot_pose_in_odom(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time(), Duration(seconds=2.0))
            p = PoseStamped()
            p.header = t.header
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.position.z = t.transform.translation.z
            p.pose.orientation = t.transform.rotation
            return p
        except Exception:
            return None

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _yaw_from_quat(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _quat_from_yaw(self, yaw):
        q = Quaternion()
        q.x = q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _publish_cmd_vel(self, twist):
        self._cmd_vel_pub.publish(twist)
        self._cmd_vel_ign_pub.publish(twist)

    def _check_object_visible(self, label=''):
        if not CV_AVAILABLE or self._bridge is None:
            return None
        deadline = time.monotonic() + WRIST_CAMERA_TIMEOUT
        while self._wrist_image is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._wrist_image is None:
            return None
        try:
            hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # FIX: filter noise — only count contours above VISION_MIN_AREA pixels.
            large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]
            return len(large) > 0
        except Exception:
            return None

    def _align_base_to_cube_camera(self, timeout=10.0):
        """Align base so the blue cube is centered in the wrist camera image.

        ROS1 approach: use ONLY camera pixel position, NO TF, NO FK for alignment.
        Condition for pick: abs(pixel_x - 320) < 10 AND pixel_y > 440
        When this condition is met, proceed immediately to pick.

        Returns True if condition met (ready to pick), False if timeout.
        """
        if not CV_AVAILABLE or self._bridge is None:
            self.get_logger().warn('  Camera alignment skipped: cv_bridge unavailable')
            return False

        self.get_logger().warn('  Starting ROS1-style camera alignment...')
        self.get_logger().warn('  Target: abs(pixel_x - 320) < 10 AND pixel_y > 440')

        t0 = time.monotonic()
        last_log = t0
        consecutive_good = 0

        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._wrist_image is None:
                self._publish_cmd_vel(Twist())
                time.sleep(0.05)
                continue

            try:
                hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Filter noise - require meaningful contour
                large = [c for c in contours if cv2.contourArea(c) >= VISION_MIN_AREA]

                if len(large) == 0:
                    if time.monotonic() - last_log > 1.5:
                        self.get_logger().warn('  No blue cube in view - searching...')
                        last_log = time.monotonic()
                    self._publish_cmd_vel(Twist())
                    consecutive_good = 0
                    continue

                # Find the largest contour (most likely the cube)
                largest = max(large, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)

                # Cube center in pixel coordinates
                cube_cx = x + w // 2
                cube_cy = y + h // 2

                # Image center and ROS1 thresholds
                height, width = self._wrist_image.shape[:2]
                img_center_x = width // 2  # 320 for 640 width

                cx_err = cube_cx - img_center_x
                cy_err = cube_cy  # ROS1 uses raw pixel_y, not error

                if time.monotonic() - last_log > 0.5:
                    cube_area = w * h
                    self.get_logger().warn(
                        f'  Camera: cube_pixel=({cube_cx},{cube_cy})  '
                        f'cx_err={cx_err}  cy={cy_err}  area={cube_area}px')

                # ROS1 pick condition: abs(point_x - 320) < 10 AND point_y > 440
                if abs(cx_err) < 10 and cube_cy > 440:
                    consecutive_good += 1
                    if consecutive_good >= 3:
                        self.get_logger().warn(
                            f'  ✅ ROS1 PICK CONDITION MET: cx_err={cx_err} (<10), cy={cube_cy} (>440)')
                        self._publish_cmd_vel(Twist())
                        return True
                else:
                    consecutive_good = 0

                # Drive base to center the cube and approach
                # Following ROS1: rotate to center, drive forward until y > 440
                twist = Twist()

                # Rotate toward cube (proportional control)
                if abs(cx_err) > 8:
                    ang_vel = max(-0.5, min(0.5, cx_err / 60.0))
                    twist.angular.z = ang_vel

                # Drive forward when roughly aligned - ROS1 uses y > 440 as "close enough"
                # If cube is in lower part of image (cy > 300), drive slow
                # If cube is in upper part (cy < 250), drive faster forward
                if abs(cx_err) < 30:  # only drive if roughly aligned
                    if cube_cy < 250:  # cube is far (high in image)
                        twist.linear.x = 0.20
                    elif cube_cy < 350:  # cube is medium distance
                        twist.linear.x = 0.12
                    elif cube_cy < 440:  # cube is close but not close enough
                        twist.linear.x = 0.06
                    # if cube_cy >= 440, we're close enough - don't drive

                self._publish_cmd_vel(twist)

            except Exception as e:
                if time.monotonic() - last_log > 1.0:
                    self.get_logger().warn(f'  Camera alignment error: {e}')
                    last_log = time.monotonic()
                self._publish_cmd_vel(Twist())

        self.get_logger().warn(f'  ⚠️ Camera alignment timeout after {timeout}s')
        self._publish_cmd_vel(Twist())
        return False


def main():
    rclpy.init()
    node = VisionPickPlace()
    
    # We run the state machine executor sequentially in a separate spinning pattern
    # to avoid thread-safety concerns, just like pick_and_place.py
    try:
        node.run_autopilot()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
