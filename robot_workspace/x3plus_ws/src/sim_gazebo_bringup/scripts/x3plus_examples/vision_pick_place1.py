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
# SRDF group_states (from x3plus_moveit_config/config/yahboomcar_X3plus.srdf)
ARM_UP      = [0.0,   0.0,    0.0,    0.0,   0.0]   # SRDF "up"
ARM_DOWN    = [0.0,  -1.5708, 0.0,    0.0,   0.0]   # SRDF "down"
ARM_INIT    = [0.0,   0.7854,-1.5708,-1.5708, 0.0]  # SRDF "init"

HOME        = ARM_UP                                 # alias for SRDF "up"
PRE_PICK    = [0.0,  -0.8,   -0.4,   -0.3,   0.0]
REACH_DOWN  = [0.0,  -1.45,  -0.54,  -1.21,  0.0]
CARRY       = [0.0,  -0.8,   -0.4,   -0.3,   0.0]
PLACE_DOWN  = [0.0,  -1.40,  -0.524, -0.873, 0.0]
PRE_PLACE   = [0.0,  -0.8,   -0.4,   -0.3,   0.0]

# SRDF gripper group_states: open=-1.54, close=0.0
GRIPPER_OPEN  = -1.54
GRIPPER_HOLD  = -0.676  # Parallel linkage secure gap (4.8 cm finger gap on 4 cm cube)
GRIPPER_CLOSE = 0.0
ARM_JOINT_NAMES = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']

# MoveIt joint_limits.yaml: max_velocity = 1.0 rad/s for all arm joints.
# Use a safety scaling matching default_velocity_scaling_factor = 0.1 ... 1.0.
ARM_MAX_VELOCITY = 1.0  # rad/s (hard URDF/MoveIt limit)
ARM_VEL_SCALE    = 0.7  # leave 30% headroom so the trajectory controller doesn't saturate

# MoveIt kinematics.yaml goal_position_tolerance = 0.0001 m. We use a slightly
# looser deadband (1 mm) for closed-loop base nudging, since the diff drive
# can't realistically resolve to 100 µm.
POSITION_TOLERANCE_M = 0.001

# arm_link5 -> grip centre offset (calibrated from TF: arm_link5.x=0.407,
# actual finger centre ≈ 0.292 → offset = -0.115 m).
ARM5_TO_GRIP_CENTER = -0.115

# Empirical FK compensation (m): the trajectory controller settles ~40 mm
# short of REACH_DOWN due to gravity droop. Compensates approach distance.
# NOTE: 2026-06-02 test showed actual error ~41mm, keep at 0.040 but monitor
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
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _on_detected_object(self, msg: PoseStamped):
        """Handle detected object pose from camera."""
        self._detected_pose_cam = msg
        self._detected_pose_time = self.get_clock().now()
        try:
            cam_to_map = self._tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(msg, cam_to_map)
            self._detected_pose_map = pose_map
        except Exception as e:
            self.get_logger().debug(f'TF camera->map failed: {e}')

    def _on_wrist_image(self, msg: Image):
        if self._bridge is None:
            return
        try:
            self._wrist_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warning(f'Wrist camera cv_bridge failed: {e}')

    def _on_llink2_contact(self, msg: Contacts):
        self._llink2_contact = msg

    def _on_rlink2_contact(self, msg: Contacts):
        self._rlink2_contact = msg

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def run_autopilot(self):
        """Sequential executor for the visual autopilot state machine."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING AUTOPILOT VISION PICK-AND-PLACE')
        self.get_logger().info('=' * 60)

        # 1. Wait for infrastructure
        if not self._wait_for_servers():
            return False

        # ── STATE: IDLE ─────────────────────────────────────────────
        self.state = self.STATE_IDLE
        self.get_logger().info(
            f'[STATE] {self.state}: Moving arm to OBSERVE pose (SRDF "init") so '
            'the wrist camera looks forward-horizontally for cube detection')
        # SRDF "init" group_state from x3plus_moveit_config: arm folded with
        # wrist horizontal. With arm at HOME (all zeros = SRDF "up") the
        # wrist camera (mono_link, mounted on arm_link4) points straight up
        # at the ceiling, which is why detection never fires. ARM_INIT
        # rotates j3 = j4 = -π/2 so the wrist faces forward.
        self._gripper_open()      # also open gripper so it doesn't occlude
        self._sleep_sim(0.5)
        self._move_arm(ARM_INIT, 'observe', duration_sec=3.0)
        time.sleep(1.0)           # let the camera image stabilise

        # Wait until camera detects physical cube
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._detected_pose_map is not None:
                # Check that detection is fresh
                age = (self.get_clock().now() - self._detected_pose_time).nanoseconds / 1e9
                if age < 3.0:
                    break
            time.sleep(0.1)

        if self._detected_pose_map is None:
            self.get_logger().error('Timeout waiting for vision detection of blue test cube')
            return False

        # ── STATE: DETECT ───────────────────────────────────────────
        self.state = self.STATE_DETECT
        self._object_pose_map = self._detected_pose_map
        self.get_logger().info(
            f'[STATE] {self.state}: Camera detected cube at '
            f'({self._object_pose_map.pose.position.x:.2f}, {self._object_pose_map.pose.position.y:.2f})'
        )

        # ── STATE: NAVIGATE (Coarse Approach) ───────────────────────
        self.state = self.STATE_NAVIGATE
        self.get_logger().info(f'[STATE] {self.state}: Driving to cube via proportional cmd_vel')

        # Fold the arm out of the way before driving so it doesn't snag on
        # anything. PRE_PICK is also the safe carry pose used by APPROACH.
        self._move_arm(PRE_PICK, 'pre_pick_for_drive', duration_sec=2.0)
        time.sleep(0.5)

        target_x = self._object_pose_map.pose.position.x
        target_y = self._object_pose_map.pose.position.y

        finger_center_reach = self._gripper_center_x_at_joints(REACH_DOWN) + FK_SETTLE_COMPENSATION
        DESIRED_STANDOFF = 0.00  # Center gripper centered onto cube center
        target_dist = finger_center_reach + DESIRED_STANDOFF

        self.get_logger().info(
            f'  Target cube odom=({target_x:.2f},{target_y:.2f})  '
            f'finger_reach={finger_center_reach:.3f}m  stop_dist={target_dist:.3f}m')

        # Closed-loop drive with continuous yaw correction AND continuous
        # target update. The cube target is refreshed each iteration from
        # (a) the live /detected_object_pose -> map transform, falling back to
        # (b) the gazebo test_block TF (ground truth) so wheel-slip /
        # odom drift can't make us stop short.
        self._drive_to_pose_xy(
            target_x, target_y,
            stop_dist=target_dist,
            max_lin=0.4,
            max_ang=1.0,
            timeout=120.0,
            log_prefix='  Navigate',
            target_tf_frame='test_block',
            target_pose_attr='_detected_pose_map',
        )

        # Refresh _object_pose_map for downstream PICK to use the latest cube
        # position (whichever source we ended on).
        if self._detected_pose_map is not None:
            self._object_pose_map = self._detected_pose_map
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'test_block', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
            tb = PoseStamped()
            tb.header = t.header
            tb.pose.position.x = t.transform.translation.x
            tb.pose.position.y = t.transform.translation.y
            tb.pose.position.z = t.transform.translation.z
            tb.pose.orientation.w = 1.0
            self._object_pose_map = tb
        except Exception:
            pass
        self.get_logger().info(
            f'  Navigate done: odom=({self._odom_x:.2f},{self._odom_y:.2f}) '
            f'final_target=({self._object_pose_map.pose.position.x:.2f},'
            f'{self._object_pose_map.pose.position.y:.2f})')

        # Fully stop
        twist = Twist()
        twist.linear.x = 0.0
        for _ in range(10):
            self._publish_cmd_vel(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.5)

        # ── STATE: APPROACH (Camera-Guided Fine Alignment) ────────
        self.state = self.STATE_APPROACH
        self.get_logger().info(f'[STATE] {self.state}: Centering camera guided fine-tuning')

        # Open gripper and raise arm to allow camera a clear, un-occluded view
        self._gripper_open()
        self._sleep_sim(2.0)
        self._move_arm(PRE_PICK, 'pre_pick', duration_sec=2.0)
        time.sleep(0.3)

        camera_ok = self._camera_guided_approach()
        if camera_ok:
            self.get_logger().warning('✅ Camera-guided alignment complete.')
        else:
            self.get_logger().warning('⚠️ Camera-guided alignment timed out or failed. Falling back.')

        # ── STATE: PICK (High precision grip and lift) ────────────
        self.state = self.STATE_PICK
        self.get_logger().info(f'[STATE] {self.state}: Executing precise picking routine')

        # Live pick-time correction
        obj_x_tf = self._object_pose_map.pose.position.x
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'test_block', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            obj_x_tf = t.transform.translation.x
        except Exception:
            pass

        self._correct_robot_x_during_pick(obj_x_tf)

        # Verify distance right before lowering
        self.get_logger().warning('═══════ DISTANCE VERIFICATION ═══════')
        # ── DEPTH CAMERA DISTANCE VERIFICATION (with backup-and-reread) ─
        # Adopted from pick_and_place.py: if we ended up too close to the
        # cube, back up and re-read the depth camera before lowering.
        self.get_logger().warning('═══════ DEPTH CAMERA DISTANCE CHECK ═══════')
        if self._detected_pose_map is not None:
            cube = self._detected_pose_map
            try:
                if cube.header.frame_id != 'odom':
                    t = self._tf_buffer.lookup_transform(
                        'odom', cube.header.frame_id, rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
                    cube_in_odom = tf2_geometry_msgs.do_transform_pose_stamped(cube, t)
                else:
                    cube_in_odom = cube

                cube_x = cube_in_odom.pose.position.x
                cube_y = cube_in_odom.pose.position.y
                dist_to_cube = math.hypot(cube_x - self._odom_x, cube_y - self._odom_y)
                finger_center = self._gripper_center_x_at_joints(REACH_DOWN) + FK_SETTLE_COMPENSATION
                error = dist_to_cube - finger_center
                self.get_logger().warning(
                    f'  Initial cam dist={dist_to_cube*1000:.1f} mm, '
                    f'target={finger_center*1000:.1f} mm, error={error*1000:.1f} mm')

                if error < -0.010:
                    backup_amount = abs(error) + 0.010
                    self.get_logger().warning(f'  Too close! Backing up {backup_amount*1000:.1f} mm')
                    twist = Twist()
                    twist.linear.x = -0.1
                    backup_duration = backup_amount / 0.1
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < backup_duration:
                        self._publish_cmd_vel(twist)
                        rclpy.spin_once(self, timeout_sec=0.01)
                        time.sleep(0.02)
                    twist.linear.x = 0.0
                    for _ in range(5):
                        self._publish_cmd_vel(twist)
                        time.sleep(0.05)
                    time.sleep(0.3)

                    # Re-read distance after backup
                    dist_new = math.hypot(cube_x - self._odom_x, cube_y - self._odom_y)
                    self.get_logger().warning(
                        f'  After backup: dist={dist_new*1000:.1f} mm, '
                        f'error={(dist_new - finger_center)*1000:.1f} mm')
                else:
                    self.get_logger().warning('  ✅ Distance OK, no backup needed')
            except Exception as e:
                self.get_logger().warning(f'  Distance verification failed: {e}')

        # REACH DOWN
        self._move_arm(REACH_DOWN, 'reach_down', duration_sec=2.0)
        self.get_logger().info('  Waiting for arm joints to fully settle...')
        time.sleep(2.0)

        # ── TF-BASED FK CALIBRATION (thorough, with logging) ────────
        # Adopted from pick_and_place.py. Looks up the actual arm_link5 pose
        # via TF, applies the calibrated ARM5_TO_GRIP_CENTER offset, then
        # closed-loop drives the base to align finger centre with cube.
        self.get_logger().warning('═══════ TF-BASED FK CALIBRATION ═══════')
        fk_finger_base = self._gripper_center_x_at_joints(REACH_DOWN)
        self.get_logger().warning(f'  FK-predicted finger_center_base = {fk_finger_base:.5f} m')

        # Log actual arm joints after REACH_DOWN
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            self.get_logger().warning(
                f'  /joint_states: '
                f'[{js.get("arm_joint1",0):.3f}, {js.get("arm_joint2",0):.3f}, '
                f'{js.get("arm_joint3",0):.3f}, {js.get("arm_joint4",0):.3f}, '
                f'{js.get("arm_joint5",0):.3f}]')

        actual_finger_base = None
        for attempt in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self._tf_buffer.lookup_transform(
                    'base_link', 'arm_link5', rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5))
                arm5_x = t.transform.translation.x
                arm5_z = t.transform.translation.z
                actual_finger_base = arm5_x + ARM5_TO_GRIP_CENTER
                self.get_logger().warning(
                    f'  TF arm_link5: x={arm5_x:.5f} z={arm5_z:.5f} → '
                    f'finger_center_x={actual_finger_base:.5f} m')
                # Log finger TFs for diagnostics
                for finger in ('rlink2', 'llink2'):
                    try:
                        ft = self._tf_buffer.lookup_transform(
                            'base_link', finger, rclpy.time.Time(),
                            rclpy.duration.Duration(seconds=0.2))
                        self.get_logger().warning(
                            f'  TF {finger}: x={ft.transform.translation.x:.4f} '
                            f'z={ft.transform.translation.z:.4f}')
                    except Exception:
                        pass
                break
            except Exception as e:
                self.get_logger().warning(f'  TF lookup attempt {attempt+1} failed: {e}')
                time.sleep(0.2)

        if actual_finger_base is None:
            self.get_logger().warning('  ⚠️  TF unavailable, falling back to FK')
            actual_finger_base = fk_finger_base
        else:
            self.get_logger().warning(
                f'  FK error = {(actual_finger_base - fk_finger_base)*1000:.1f} mm')

        cube_world_x = self._object_pose_map.pose.position.x
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'test_block', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
            cube_world_x = t.transform.translation.x
            self.get_logger().warning(f'  Cube TF (odom): x={cube_world_x:.5f}')
        except Exception:
            pass

        finger_world = self._odom_x + actual_finger_base
        error = finger_world - cube_world_x
        self.get_logger().warning(
            f'  odom_x={self._odom_x:.5f} finger_world={finger_world:.5f} '
            f'cube_world={cube_world_x:.5f} error={error*1000:.1f} mm')

        # ── CENTER GRIPPER ON CUBE (robot-frame forward; yaw-aware) ──
        if abs(error) < POSITION_TOLERANCE_M:
            self.get_logger().warning('  ✅ Already centred — no move needed')
        else:
            # Robot is now facing the cube (yaw aligned in APPROACH). Drive
            # forward/backward in the *robot* frame and project onto the
            # robot-cube vector to terminate; do NOT assume yaw==0 / odom_x.
            cube_world_y = self._object_pose_map.pose.position.y
            try:
                tt = self._tf_buffer.lookup_transform(
                    'odom', 'test_block', rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.3))
                cube_world_y = tt.transform.translation.y
            except Exception:
                pass
            twist = Twist()
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0:
                rclpy.spin_once(self, timeout_sec=0.01)
                dx = cube_world_x - self._odom_x
                dy = cube_world_y - self._odom_y
                dist = math.hypot(dx, dy)
                # Signed forward distance along robot heading
                forward = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
                signed_err = forward - actual_finger_base
                if abs(signed_err) < POSITION_TOLERANCE_M:
                    break
                twist.linear.x = max(-0.15, min(0.15, signed_err * 1.0))
                self._publish_cmd_vel(twist)
                time.sleep(0.02)
            twist.linear.x = 0.0
            for _ in range(5):
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
            time.sleep(0.3)
            self.get_logger().warning(
                f'  After: odom=({self._odom_x:.3f},{self._odom_y:.3f}) '
                f'yaw={math.degrees(self._odom_yaw):.1f}°')

        # GRASP details
        self.get_logger().info('═══════ GRIPPER CLOSE & VERIFY ═══════')
        self.get_logger().info('Grasping cube to secure hold')
        self._gripper_close()
        self._sleep_sim(2.5)
        self.get_logger().info('  Gripper close command sent')

        # Verify grasp and Lift arm
        grasped = self._verify_pickup(self._object_pose_map)
        if grasped:
            self.get_logger().info('✅ GRASP CONFIRMED - Object secured')
        else:
            self.get_logger().warning('⚠️ Pickup not confirmed by physics. Lifting anyway.')

        # Lift
        self.get_logger().info('═══════ LIFT SEQUENCE ═══════')
        self._move_arm(CARRY, 'lift_carry', duration_sec=4.0)
        self.get_logger().info('  Arm lifted to CARRY pose')
        time.sleep(0.5)
        self._move_arm(HOME, 'fold_home', duration_sec=3.0)
        self.get_logger().info('  Arm folded to HOME pose')
        time.sleep(0.3)

        # Vision verification
        vision_ok = self._check_object_visible('[PICK]')
        if vision_ok:
            self.get_logger().info('✅ VISION VERIFIED: object in grasp.')
        else:
            self.get_logger().info('⚠️ Wrist camera: object not visible (expected if in gripper)')

        # ── STATE: TRANSPORT ────────────────────────────────────────
        self.state = self.STATE_TRANSPORT
        self.get_logger().info(f'[STATE] {self.state}: Navigating to green landing pad zone')
        self.get_logger().info('═══════ TRANSPORT SEQUENCE ═══════')

        # Safety maneuver
        self._backup_and_strafe()
        self.get_logger().info('  Safety maneuver complete')

        drop_x = self.get_parameter('drop_off_x').value
        drop_y = self.get_parameter('drop_off_y').value

        # The landing pad sits in the +y direction from the cube spawn.
        # Stop short of the pad so PLACE_DOWN can lower the cube onto its
        # centre. Stand-off accounts for the gripper reach forward of base.
        gripper_reach = self._gripper_center_x_at_joints(REACH_DOWN) + FK_SETTLE_COMPENSATION
        place_standoff = max(0.20, gripper_reach)

        # Drive in a single closed-loop step that yaw-aligns toward the pad
        # and drives forward with continuous heading correction. Stops at
        # ``place_standoff`` short of the pad so the wall is never reached.
        drop = PoseStamped()
        drop.header.frame_id = 'odom'
        drop.pose.position.x = drop_x
        drop.pose.position.y = drop_y
        drop.pose.orientation = self._quat_from_yaw(self.get_parameter('drop_off_yaw').value)
        self.get_logger().info(
            f'  Driving to landing pad ({drop_x:.2f}, {drop_y:.2f}) '
            f'with stand-off {place_standoff:.2f} m')
        self._drive_to_pose_xy(
            drop_x, drop_y,
            stop_dist=place_standoff,
            max_lin=0.3,
            max_ang=1.0,
            timeout=60.0,
            log_prefix='  Transport',
        )
        self.get_logger().info('  Transport complete - arrived at drop zone')

        # ── STATE: PLACE ────────────────────────────────────────────
        self.state = self.STATE_PLACE
        self.get_logger().info(f'[STATE] {self.state}: Releasing object onto landing pad')

        self._move_arm(PRE_PLACE, 'pre_place', duration_sec=2.0)
        time.sleep(0.3)
        self._move_arm(PLACE_DOWN, 'place_down', duration_sec=2.0)
        time.sleep(0.3)
        
        self._gripper_open()
        self._sleep_sim(3.5)
        self.get_logger().info('Gripper released cube')

        self._move_arm(HOME, 'final_home', duration_sec=2.0)
        time.sleep(0.3)

        vision_released = self._check_object_visible('[PLACE]')
        if vision_released is False:
            self.get_logger().info('✅ PLACE VISION CONFIRMED: gripper empty.')

        self.get_logger().info('=' * 60)
        self.get_logger().info('AUTOPILOT PICK-AND-PLACE SUCCESSFUL!')
        self.get_logger().info('=' * 60)
        
        # Reset state to idle
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
        point.time_from_start = rclpy.duration.Duration(seconds=duration_sec).to_msg()
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
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        goal.trajectory.points = [point]
        self._gripper_traj_ac.send_goal_async(goal)

    def _sleep_sim(self, seconds):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _verify_pickup(self, obj_map):
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            grip_pos = js.get('grip_joint', None)
            if grip_pos is not None:
                if abs(grip_pos - GRIPPER_HOLD) < 0.3:
                    return True
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'test_block', rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
            cube_x = t.transform.translation.x
            cube_y = t.transform.translation.y
            dist = math.hypot(cube_x - obj_map.pose.position.x, cube_y - obj_map.pose.position.y)
            return dist > 0.3
        except Exception:
            return True

    def _gripper_center_x_at_joints(self, joints):
        j1, j2, j3, j4, j5 = joints
        J2_REF = -1.45
        J3_REF = -0.180
        CENTER_REF = 0.3032
        dX_dJ2 = 0.150 * math.cos(J2_REF)
        dX_dJ3 = 0.145 * math.cos(J3_REF)
        return CENTER_REF + dX_dJ2 * (j2 - J2_REF) + dX_dJ3 * (j3 - J3_REF)

    def _camera_guided_approach(self):
        """Fine-position the gripper over the cube using depth camera feedback."""
        deadline = time.monotonic() + 10.0
        while self._detected_pose_map is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._detected_pose_map is None:
            self.get_logger().warning('Camera guided: no detection available')
            return False

        cube = self._detected_pose_map
        cube_ox = cube.pose.position.x
        cube_oy = cube.pose.position.y

        # Yaw alignment and centering
        dx = cube_ox - self._odom_x
        dy = cube_oy - self._odom_y
        target_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

        if abs(yaw_err) > 0.05:
            self.get_logger().warning('Fine-guided Face Cube Yaw Alignment')
            twist = Twist()
            yaw_deadline = time.monotonic() + 15.0
            while time.monotonic() < yaw_deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                dx = cube_ox - self._odom_x
                dy = cube_oy - self._odom_y
                yaw_err = self._normalize_angle(math.atan2(dy, dx) - self._odom_yaw)
                if abs(yaw_err) < 0.03:
                    break
                twist.angular.z = max(-0.5, min(0.5, yaw_err * 1.5))
                self._publish_cmd_vel(twist)
                time.sleep(0.02)
                
            twist.angular.z = 0.0
            self._publish_cmd_vel(twist)
        return True

    def _correct_robot_x_during_pick(self, obj_world_x, max_correction=0.25):
        """Closed-loop drive in robot-forward direction to align finger over cube.

        Yaw-aware: projects the robot→cube vector onto the heading and drives
        until the projection equals the FK finger reach. Does NOT assume the
        robot is aligned with the +x odom axis.
        """
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        joints = REACH_DOWN
        finger_center_base = self._gripper_center_x_at_joints(joints) + FK_SETTLE_COMPENSATION

        # Recover cube y from latest detection (defaults to current robot y).
        obj_world_y = self._odom_y
        if self._object_pose_map is not None:
            obj_world_y = self._object_pose_map.pose.position.y
        try:
            tt = self._tf_buffer.lookup_transform(
                'odom', 'test_block', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.3))
            obj_world_y = tt.transform.translation.y
        except Exception:
            pass

        twist = Twist()
        t0 = time.monotonic()
        deadline = t0 + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = obj_world_x - self._odom_x
            dy = obj_world_y - self._odom_y
            forward = dx * math.cos(self._odom_yaw) + dy * math.sin(self._odom_yaw)
            err = forward - finger_center_base
            if abs(err) < 0.008:
                break
            # Clamp single-step correction
            err_clamped = max(-max_correction, min(max_correction, err))
            twist.linear.x = max(-0.15, min(0.15, err_clamped * 1.0))
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
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

            # Live target update.
            if target_tf_frame is not None:
                try:
                    t = self._tf_buffer.lookup_transform(
                        'odom', target_tf_frame, rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=0.05))
                    target_x = t.transform.translation.x
                    target_y = t.transform.translation.y
                except Exception:
                    if target_pose_attr is not None:
                        p = getattr(self, target_pose_attr, None)
                        if p is not None:
                            target_x = p.pose.position.x
                            target_y = p.pose.position.y

            dx = target_x - self._odom_x
            dy = target_y - self._odom_y
            dist = math.hypot(dx, dy)
            if dist <= stop_dist:
                break

            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

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
                'odom', 'base_footprint', rclpy.time.Time(), rclpy.time.Duration(seconds=2.0))
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
            return len(contours) > 0
        except Exception:
            return None


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
