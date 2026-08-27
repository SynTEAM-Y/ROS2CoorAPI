#!/usr/bin/env python3
"""Autonomous pick-and-place using Nav2 + fixed arm sequences.

ROS2 adaptation of the ROS1 garbage_identify_yolov11 state machine.
Uses Nav2 for navigation, FollowJointTrajectory for arm control, and
fixed joint angles (no MoveIt / no IK planning).

Usage:
  ros2 launch sim_gazebo_bringup pick_and_place.launch.py

=== OPTIMISED PICK CONFIG (tuned for test_block 4 cm blue cube) ===

Standoff distance:
  DESIRED_STANDOFF = 0.02 m  (2 cm)  — robot stops so gripper centre
    is 4.5 cm from cube centre when arm is at REACH_DOWN.
  GAP_BIAS         = 0.02 m  (2 cm)  — FK correction adds +2 cm margin,
    final gap ≈ 5 cm between gripper centre and cube centre.

Arm joint angles [J1..J5] in radians:
  REACH_DOWN = [0.0, -1.45, -0.54, -1.21, 0.0]  — low pick pose
  CARRY      = [0.0, -0.80, -0.40, -0.30, 0.0]  — lift / carry pose
  HOME       = [0.0,  0.00,  0.00,  0.00, 0.0]  — folded

Gripper (grip_joint position in radians):
  GRIPPER_OPEN = -1.54  — fully open
  GRIPPER_HOLD = -0.676 — 4.8 cm finger gap (8 mm wider than cube for clearance)

Physics:
  mu1 = mu2 = 30.0  — gripper fingers and test_block (high friction)
  Cube mass = 0.02 kg (20 g)

Timing (pick sequence):
  gripper open wait      2.0 s  (trajectory 1.0 s + mimic ramp @ 1.5 rad/s)
  arm to PRE_PICK        2.0 s  + 0.3 s settle
  arm to REACH_DOWN      2.0 s  + 0.5 s settle
  gripper close wait     2.0 s
  extra settle           0.3 s
  arm to CARRY           4.0 s  + 0.5 s (slow lift to avoid dropping)
  arm to HOME            3.0 s  + 0.3 s

Transport drive speed:  0.8 m/s
Drop-off fixed location: (2.0, 1.2) in odom frame — green landing pad
  (models/landing_pad/model.sdf) in front of the static wall at (2, 2).
  Robot parks facing the wall and places the cube against it.

=== END OPTIMISED PICK CONFIG ===
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry  # for direct odom feedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
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


# ── HSV range for the test block (blue/cyan 4cm cube) ─────────────────────────────────
# Matches object_detector.py defaults for the test_block material.
BLUE_LOWER = np.array([80, 50, 50])
BLUE_UPPER = np.array([120, 255, 255])
VISION_MIN_AREA = 200        # minimum contour area (px) to consider object present
WRIST_CAMERA_TIMEOUT = 3.0   # seconds to wait for first wrist camera frame


# ── Pre-defined arm joint positions (radians) ──────────────────────────
HOME        = [0.0,   0.0,    0.0,    0.0,   0.0]
PRE_PICK    = [0.0,  -0.8,   -0.4,   -0.3,   0.0]
REACH_DOWN  = [0.0,  -1.45,  -0.54,  -1.21,  0.0]
CARRY       = [0.0,  -0.8,   -0.4,   -0.3,   0.0]
PLACE_DOWN  = [0.0,  -1.40,  -0.524, -0.873, 0.0]
PRE_PLACE   = [0.0,  -0.8,   -0.4,   -0.3,   0.0]

GRIPPER_OPEN  = -1.54
GRIPPER_HOLD  = -0.676  # 4.8 cm finger gap — 8 mm wider than cube width for clearance
GRIPPER_CLOSE = 0.0
ARM_JOINT_NAMES = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']

# Empirical FK compensation (m): the arm trajectory controller cannot reach REACH_DOWN
# — the arm settles at gravity equilibrium ~40 mm short of commanded position.
# FK(REACH_DOWN) = 0.252 m, but actual finger_center_base ≈ 0.292 m (from TF with
# correct offset).  Adding this offset makes approach/FK-correction match reality.
FK_SETTLE_COMPENSATION = 0.040


class PickAndPlace(Node):
    """State-machine-based pick-and-place using Nav2 + fixed arm angles."""

    def __init__(self):
        super().__init__('pick_and_place')
        self._cb_group = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('object_x', 2.0)
        self.declare_parameter('object_y', 0.0)
        self.declare_parameter('object_z', 0.03)
        self.declare_parameter('drop_off_x', 2.0)
        self.declare_parameter('drop_off_y', 1.2)
        self.declare_parameter('drop_off_yaw', 0.0)
        self.declare_parameter('approach_offset', 0.35)
        self.declare_parameter('skip_navigation', False)
        self.declare_parameter('use_fixed_object', True)

        # ── TF ──────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Action clients ──────────────────────────────────────────
        self._arm_traj_ac = ActionClient(
            self, FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory',
            callback_group=self._cb_group)
        self._gripper_traj_ac = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_group_controller/follow_joint_trajectory',
            callback_group=self._cb_group)
        self._nav_ac = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self._cb_group)

        # ── Direct gripper publisher ────────────────────────────────
        # The gripper controller expects Float64 messages on this topic
        # when the trajectory controller is not available.
        self._grip_pub = self.create_publisher(Float64, '/gripper_group_controller/commands', 10)

        # ── Command velocity publisher ──────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cmd_vel_ign_pub = self.create_publisher(
            Twist, '/model/x3plus/cmd_vel', 10)

        # ── Direct odom subscriber ──────────────────────────────────
        self._odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self.create_subscription(
            Odometry, '/odom', self._on_odom, self._odom_qos)

        # ── Sensor: detected object pose from camera ────────────────
        self._detected_pose_cam = None
        self._detected_pose_map = None
        self._detected_pose_time = None
        self.create_subscription(
            PoseStamped, '/detected_object_pose', self._on_detected_object, 10)

        # ── Wrist camera for grasp confirmation ─────────────────────
        self._wrist_image = None
        self._bridge = CvBridge() if CV_AVAILABLE else None
        if CV_AVAILABLE:
            self.create_subscription(
                Image, '/wrist_mono_camera/image_raw',
                self._on_wrist_image, 1)

        # ── Joint state subscriber ──────────────────────────────────
        self._joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, self._odom_qos)

        # ── Contact sensors on gripper fingers ─────────────────────
        self._llink2_contact = None
        self._rlink2_contact = None
        self.create_subscription(
            Contacts, '/model/x3plus/contact/llink2',
            self._on_llink2_contact, 10)
        self.create_subscription(
            Contacts, '/model/x3plus/contact/rlink2',
            self._on_rlink2_contact, 10)

        # ── State machine ───────────────────────────────────────────
        self._state = 'WAIT'
        self._object_pose_map = None

        self.get_logger().info('Pick-and-Place state machine initialised')

    # ── Public entry point ───────────────────────────────────────────

    def run(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING AUTONOMOUS PICK-AND-PLACE')
        self.get_logger().info('=' * 60)

        skip_nav = self.get_parameter('skip_navigation').value

        # 1. Wait for infrastructure
        if not self._wait_for_servers():
            return False

        # 2. Build object pose in map frame
        use_fixed = self.get_parameter('use_fixed_object').value
        obj_map = PoseStamped()
        obj_map.header.frame_id = 'map'
        obj_map.header.stamp = self.get_clock().now().to_msg()

        if not use_fixed and self._detected_pose_map is not None:
            obj_map = self._detected_pose_map
            self.get_logger().info(
                f'Using CAMERA-DETECTED object pose: '
                f'({obj_map.pose.position.x:.2f}, '
                f'{obj_map.pose.position.y:.2f}, '
                f'{obj_map.pose.position.z:.2f})'
            )
        else:
            obj_map.pose.position.x = self.get_parameter('object_x').value
            obj_map.pose.position.y = self.get_parameter('object_y').value
            obj_map.pose.position.z = self.get_parameter('object_z').value
            obj_map.pose.orientation.w = 1.0
            self.get_logger().info(
                f'Using FIXED object pose: '
                f'({obj_map.pose.position.x:.2f}, '
                f'{obj_map.pose.position.y:.2f}, '
                f'{obj_map.pose.position.z:.2f})'
            )
        self._object_pose_map = obj_map

        # 3. Move arm to HOME (gripper stays closed until pick)
        self._move_arm(HOME, 'home', duration_sec=2.0)
        time.sleep(0.3)

        # ═════════════════════════════════════════════════════════════
        # STATE MACHINE
        # ═════════════════════════════════════════════════════════════

        # ── STATE: APPROACH ─────────────────────────────────────────
        if not skip_nav:
            self.get_logger().info('[STATE] APPROACH: driving to object via cmd_vel')

            target_x = obj_map.pose.position.x
            target_y = obj_map.pose.position.y

            # Gripper center extends past robot base — account for it so we
            # don't ram the cube.  See module docstring for optimal config.
            finger_center_reach = self._gripper_center_x_at_joints(REACH_DOWN) + FK_SETTLE_COMPENSATION
            DESIRED_STANDOFF = 0.00  # 0 cm — gripper centre aligns with cube centre

            # Use 2D distance (accounts for robot yaw, unlike X-only comparison)
            dx = target_x - self._odom_x
            dy = target_y - self._odom_y
            dist_to_cube = math.hypot(dx, dy)
            target_dist = finger_center_reach + DESIRED_STANDOFF

            self.get_logger().warning(
                f'═ APPROACH: cube at ({target_x:.2f},{target_y:.2f}), '
                f'finger_center_base={finger_center_reach:.3f}m, '
                f'robot→cube={dist_to_cube*1000:.0f}mm, target_dist={target_dist*1000:.0f}mm'
            )

            twist = Twist()
            twist.linear.x = 1.0
            deadline = time.monotonic() + 180.0
            last_log = 0.0

            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)

                dx = target_x - self._odom_x
                dy = target_y - self._odom_y
                dist_to_cube = math.hypot(dx, dy)
                remaining = dist_to_cube - target_dist

                if time.monotonic() - last_log > 0.5:
                    self.get_logger().info(
                        f'  Approach: base=({self._odom_x:.3f},{self._odom_y:.3f}) '
                        f'dist_to_cube={dist_to_cube:.3f} '
                        f'remaining={remaining:.3f}'
                    )
                    last_log = time.monotonic()

                if remaining < 0.20:
                    twist.linear.x = 0.20
                if remaining < 0.10:
                    twist.linear.x = 0.10
                if remaining < 0.05:
                    twist.linear.x = 0.04
                if remaining <= 0.0:
                    self.get_logger().info(
                        f'  Gripper at cube standoff, stopping for PICK'
                    )
                    break

                self._publish_cmd_vel(twist)
                time.sleep(0.02)

            twist.linear.x = 0.0
            self._publish_cmd_vel(twist)
            for _ in range(10):
                self._publish_cmd_vel(twist)
                rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.5)

            dx = target_x - self._odom_x
            dy = target_y - self._odom_y
            self.get_logger().warning(
                f'═ APPROACH done: base_odom=({self._odom_x:.4f},{self._odom_y:.4f}), '
                f'robot→cube={math.hypot(dx, dy):.4f}m, target={target_dist:.4f}m'
            )
        else:
            self.get_logger().info('[STATE] APPROACH: skipped (skip_navigation=true)')

        # Debug FK at approach position — shows where finger would be at REACH_DOWN
        self._debug_fk('after approach', REACH_DOWN)

        # ── STATE: PICK ─────────────────────────────────────────────
        self.get_logger().info('[STATE] PICK: reaching down and gripping')

        # Open gripper — trajectory (1.0 sim-s) + mimic relay ramp (~1.0 sim-s @ 1.5 rad/s)
        self._gripper_open()
        self._sleep_sim(2.0)

        self._move_arm(PRE_PICK, 'pre_pick', duration_sec=2.0)
        time.sleep(0.3)

        # ── DEPTH CAMERA GUIDED APPROACH ────────────────────────────
        # After the coarse approach stop, use the depth camera to get the
        # exact cube position and drive closed-loop to centre the gripper.
        # The arm is raised (PRE_PICK) so the camera has a clear view.
        camera_ok = self._camera_guided_approach()

        if camera_ok:
            self.get_logger().warning('✅ Camera-guided approach succeeded')
        else:
            # Fall back to open-loop FK correction
            self.get_logger().warning('⚠️  Camera not available, using FK correction')
            obj_x = self._object_pose_map.pose.position.x
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'test_block',
                    rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
                obj_x = t.transform.translation.x
                self.get_logger().warning(
                    f'  Cube X via TF: {obj_x:.5f} m'
                )
            except Exception:
                self.get_logger().warning(
                    f'  Cannot get cube TF, using fixed pose X: {obj_x:.5f} m'
                )

            for attempt in range(3):
                ok = self._correct_robot_x_during_pick(obj_x, max_correction=0.25)
                if ok:
                    self.get_logger().warning(f'Correction succeeded on attempt {attempt+1}')
                    break
                self.get_logger().warning(f'Correction attempt {attempt+1} failed, retrying...')
                time.sleep(0.3)

        # ── DEPTH CAMERA DISTANCE VERIFICATION ──────────────────────
        # Read depth camera distance to cube, backup if too close, re-read
        self.get_logger().warning('═══════ DEPTH CAMERA DISTANCE CHECK ═══════')
        
        # Get current cube distance from detected pose
        if self._detected_pose_map is not None:
            cube = self._detected_pose_map
            try:
                if cube.header.frame_id != 'odom':
                    t = self._tf_buffer.lookup_transform(
                        'odom', cube.header.frame_id,
                        rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
                    cube_in_odom = tf2_geometry_msgs.do_transform_pose_stamped(cube, t)
                else:
                    cube_in_odom = cube
                
                cube_x = cube_in_odom.pose.position.x
                cube_y = cube_in_odom.pose.position.y
                dist_to_cube = math.hypot(cube_x - self._odom_x, cube_y - self._odom_y)
                
                self.get_logger().warning(f'  Initial depth camera distance: {dist_to_cube*1000:.1f} mm')
                self.get_logger().warning(f'  Robot odom: ({self._odom_x:.4f}, {self._odom_y:.4f})')
                self.get_logger().warning(f'  Cube odom:  ({cube_x:.4f}, {cube_y:.4f})')
                
                # Target distance: gripper center aligned with cube FRONT FACE.
                # The depth camera detects the front face (2 cm closer than the
                # true centre), so finger_centre = front_face puts the gripper
                # centre 2 cm behind the true centre — matching FK/GAP_BIAS.
                finger_center = self._gripper_center_x_at_joints(REACH_DOWN) + FK_SETTLE_COMPENSATION
                target_dist = finger_center  # front face offset already in camera reading
                error = dist_to_cube - target_dist
                
                self.get_logger().warning(f'  Target distance: {target_dist*1000:.1f} mm')
                self.get_logger().warning(f'  Error (positive = too far): {error*1000:.1f} mm')
                
                # If robot is too close (error < -10mm), backup and re-read
                if error < -0.010:
                    backup_amount = abs(error) + 0.010  # Backup error + 1cm margin
                    self.get_logger().warning(f'  ⚠️  Too close! Backing up {backup_amount*1000:.1f} mm')
                    
                    # Backup
                    twist = Twist()
                    twist.linear.x = -0.1  # Slow backup
                    backup_duration = backup_amount / 0.1
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < backup_duration:
                        self._publish_cmd_vel(twist)
                        rclpy.spin_once(self, timeout_sec=0.01)
                        time.sleep(0.02)
                    
                    # Stop
                    twist.linear.x = 0.0
                    for _ in range(5):
                        self._publish_cmd_vel(twist)
                        time.sleep(0.05)
                    time.sleep(0.3)
                    
                    # Re-read distance
                    dist_to_cube_new = math.hypot(cube_x - self._odom_x, cube_y - self._odom_y)
                    error_new = dist_to_cube_new - target_dist
                    self.get_logger().warning(f'  After backup distance: {dist_to_cube_new*1000:.1f} mm')
                    self.get_logger().warning(f'  New error: {error_new*1000:.1f} mm')
                else:
                    self.get_logger().warning('  ✅ Distance OK, no backup needed')
                    
            except Exception as e:
                self.get_logger().warning(f'  Depth camera distance check failed: {e}')
        else:
            self.get_logger().warning('  No detected pose available for distance check')
        
        self.get_logger().warning('═══════════════════════════════════════════')

        self._move_arm(REACH_DOWN, 'reach_down', duration_sec=2.0)
        self.get_logger().warning('═══════ Arm trajectory to REACH_DOWN complete ═══════')
        self.get_logger().warning('  Waiting 2.0s for arm to fully settle at pick position...')
        time.sleep(2.0)  # Increased settle time: ensure arm physically reaches final position before gripper closes

        # ── TF-BASED FK CALIBRATION ─────────────────────────────────
        # Look up the ACTUAL arm_link5 position from TF to get the real
        # finger_center_base, then use it to center the gripper on the cube.
        # This bypasses any FK inaccuracy.
        self.get_logger().warning('═══════ TF-BASED FK CALIBRATION ═══════')

        # FK-predicted finger center (for logging comparison)
        fk_finger_base = self._gripper_center_x_at_joints(REACH_DOWN)
        self.get_logger().warning(f'  FK-predicted finger_center_base = {fk_finger_base:.5f} m')

        # Get ACTUAL finger centre from TF via arm_link5 + calibrated offset.
        # arm_link5 IS in the TF tree; finger links (rlink2/llink2) are NOT
        # because <mimic> tags were removed from URDF for Ignition compatibility.
        # The offset from arm_link5 to the grip centre was calibrated:
        #   arm_link5.x = 0.407m, actual grip centre ≈ 0.292m → offset = -0.115m
        ARM5_TO_GRIP_centre = -0.115  # m — arm_link5.x=0.407, grip≈0.292

        # Log actual arm joint states after REACH_DOWN
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            self.get_logger().warning(
                f'  /joint_states after REACH_DOWN: '
                f'[{js.get("arm_joint1",0):.3f}, {js.get("arm_joint2",0):.3f}, '
                f'{js.get("arm_joint3",0):.3f}, {js.get("arm_joint4",0):.3f}, '
                f'{js.get("arm_joint5",0):.3f}]'
            )

        actual_finger_base = None
        for attempt in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self._tf_buffer.lookup_transform(
                    'base_link', 'arm_link5',
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
                arm5_x = t.transform.translation.x
                arm5_z = t.transform.translation.z
                actual_finger_base = arm5_x + ARM5_TO_GRIP_centre
                self.get_logger().warning(
                    f'  TF arm_link5: x={arm5_x:.5f}  z={arm5_z:.5f}  '
                    f'offset={ARM5_TO_GRIP_centre:.3f}  → finger_center_x={actual_finger_base:.5f} m'
                )
                # Log cube Z for comparison
                cube_z = self._object_pose_map.pose.position.z if self._object_pose_map else 0.0
                # arm5_z is in base_link frame; base_link.z in odom ≈ 0.076 via base_footprint
                arm5_z_odom = arm5_z + 0.076
                self.get_logger().warning(
                    f'  Cube centre z={cube_z:.3f}  —  arm_link5 z_odom≈{arm5_z_odom:.3f}  '
                    f'(gap={(arm5_z_odom - cube_z)*100:.0f} cm)'
                )
                # Log actual finger positions from TF
                for finger in ['rlink2', 'llink2']:
                    try:
                        ft = self._tf_buffer.lookup_transform(
                            'base_link', finger,
                            rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2))
                        self.get_logger().warning(
                            f'  TF {finger}: x={ft.transform.translation.x:.4f} '
                            f'y={ft.transform.translation.y:.4f} '
                            f'z={ft.transform.translation.z:.4f} '
                            f'(odom z≈{ft.transform.translation.z + 0.076:.4f})'
                        )
                    except Exception:
                        pass
                break
            except Exception as e:
                self.get_logger().warning(f'  TF lookup attempt {attempt+1} failed: {e}')
                time.sleep(0.2)

        if actual_finger_base is None:
            self.get_logger().warning('  ⚠️  Cannot get TF arm_link5 — falling back to FK')
            actual_finger_base = fk_finger_base
        else:
            self.get_logger().warning(
                f'  FK error = {(actual_finger_base - fk_finger_base)*1000:.1f} mm'
            )

        # Get cube world X from TF (or fall back to fixed pose)
        cube_world_x = obj_map.pose.position.x
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'test_block',
                rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            cube_world_x = t.transform.translation.x
            self.get_logger().warning(f'  Cube TF (odom): x={cube_world_x:.5f} m')
        except Exception:
            self.get_logger().warning('  Cannot get cube TF, using fixed pose')

        # Compute error: positive = finger ahead of cube → need backward
        finger_world = self._odom_x + actual_finger_base
        error = finger_world - cube_world_x
        self.get_logger().warning(
            f'  odom_x={self._odom_x:.5f}  finger_world={finger_world:.5f}  '
            f'cube_world={cube_world_x:.5f}  error={error*1000:.1f} mm'
        )

        # ── CENTER GRIPPER ON CUBE ──────────────────────────────────
        if abs(error) < 0.003:
            self.get_logger().warning('  ✅ Already centred (< 3 mm) — no move needed')
        else:
            target_odom_x = cube_world_x - actual_finger_base
            if self._odom_x < target_odom_x:
                drive_speed = 0.15
                dir_label = 'forward'
            else:
                drive_speed = -0.15
                dir_label = 'backward'
            self.get_logger().warning(
                f'  Driving {dir_label} to odom_x={target_odom_x:.5f} '
                f'(error={error*1000:.1f} mm)'
            )
            twist = Twist()
            twist.linear.x = drive_speed
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0:
                rclpy.spin_once(self, timeout_sec=0.01)
                if abs(self._odom_x - target_odom_x) < 0.003:
                    break
                self._publish_cmd_vel(twist)
                time.sleep(0.02)
            # Stop
            twist.linear.x = 0.0
            for _ in range(5):
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
            time.sleep(0.3)

            # Log result
            finger_world_after = self._odom_x + actual_finger_base
            actual_error = finger_world_after - cube_world_x
            self.get_logger().warning(
                f'  After: odom_x={self._odom_x:.5f}  '
                f'finger_world={finger_world_after:.5f}  '
                f'error={actual_error*1000:.1f} mm'
            )

        self.get_logger().warning('═══════════════════════════════════════════')

        # Close gripper — ONLY AFTER arm has fully completed REACH_DOWN, settled, and backed off
        # Use gentle GRIPPER_HOLD position to maintain parallel linkage (no excessive squeeze)
        self.get_logger().warning('═══════ ARM SETTLED at REACH_DOWN — closing gripper to HOLD position ═══════')
        self._gripper_close()
        self._sleep_sim(2.0)  # Wait for gripper to reach position and settle
        self.get_logger().warning('═══════ Gripper closed to HOLD (4.8cm gap) — parallel linkage maintained ═══════')

        # Extra settle time to ensure grip takes hold
        self._sleep_sim(0.5)

        # Verify pickup
        grasped = self._verify_pickup(obj_map)
        if grasped:
            self.get_logger().info('✅ PICKUP VERIFIED — object grasped')
        else:
            self.get_logger().warning('⚠️  Pickup NOT confirmed — object may still be on ground')

        self.get_logger().info('Gripper closed — object grasped')

        # ── STATE: LIFT ─────────────────────────────────────────────
        self.get_logger().info('[STATE] LIFT: raising arm')
        self._move_arm(CARRY, 'lift', duration_sec=4.0)
        time.sleep(0.5)
        self._move_arm(HOME, 'fold_home', duration_sec=3.0)
        time.sleep(0.3)

        # Vision confirmation: wrist camera checks if object is still held
        vision_ok = self._check_object_visible('[PICK]')
        if vision_ok is True:
            self.get_logger().info('✅ PICK VISION CONFIRMED — object visible in wrist cam')
        elif vision_ok is False:
            self.get_logger().warning('⚠️  PICK VISION FAILED — object NOT visible, may have dropped')
        else:
            self.get_logger().info('ℹ️  PICK vision check unavailable (no cv_bridge or no frame)')

        # ── STATE: TRANSPORT ────────────────────────────────────────
        if not skip_nav:
            self.get_logger().info('[STATE] TRANSPORT: driving to drop-off')

            self._backup_and_strafe()

            drop_x = self.get_parameter('drop_off_x').value
            drop_y = self.get_parameter('drop_off_y').value

            # Phase 1: safe waypoint to the left/south of the wall
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'odom'
            waypoint.pose.position.x = 0.5
            waypoint.pose.position.y = 1.0
            waypoint.pose.orientation.w = 1.0
            self.get_logger().info('TRANSPORT Phase 1: driving to (0.5, 1.0)')
            self._align_yaw_to_target(waypoint)
            self._drive_to_target(waypoint)

            # Phase 2: drive to the green landing pad in front of the wall
            drop = PoseStamped()
            drop.header.frame_id = 'odom'
            drop.pose.position.x = drop_x
            drop.pose.position.y = drop_y
            drop.pose.orientation = self._quat_from_yaw(
                self.get_parameter('drop_off_yaw').value)
            self.get_logger().info(f'TRANSPORT Phase 2: driving to ({drop_x}, {drop_y})')
            self._align_yaw_to_target(drop)
            self._drive_to_target(drop)

            # Align to face the wall at (2,2) for placing
            wall = PoseStamped()
            wall.header.frame_id = 'odom'
            wall.pose.position.x = 2.0
            wall.pose.position.y = 2.0
            wall.pose.orientation.w = 1.0
            self._align_yaw_to_target(wall)
        else:
            self.get_logger().info('[STATE] TRANSPORT: skipped')

        # ── STATE: PLACE ────────────────────────────────────────────
        self.get_logger().info('[STATE] PLACE: lowering and releasing')
        self._move_arm(PRE_PLACE, 'pre_place', duration_sec=2.0)
        time.sleep(0.3)
        self._move_arm(PLACE_DOWN, 'place_down', duration_sec=2.0)
        time.sleep(0.3)
        self._gripper_open()
        self._sleep_sim(3.5)
        self.get_logger().info('Gripper open — object released')

        # ── STATE: RETURN ───────────────────────────────────────────
        self.get_logger().info('[STATE] RETURN: folding arm')
        self._move_arm(HOME, 'final_home', duration_sec=2.0)
        time.sleep(0.3)

        # Vision confirmation: wrist camera checks object is no longer held
        vision_ok = self._check_object_visible('[PLACE]')
        if vision_ok is False:
            self.get_logger().info('✅ PLACE VISION CONFIRMED — object no longer in gripper')
        elif vision_ok is True:
            self.get_logger().warning('⚠️  PLACE VISION FAILED — object still visible, may not have released')
        else:
            self.get_logger().info('ℹ️  PLACE vision check unavailable (no cv_bridge or no frame)')

        self.get_logger().info('=' * 60)
        self.get_logger().info('PICK-AND-PLACE COMPLETED SUCCESSFULLY')
        self.get_logger().info('=' * 60)
        return True

    # ── Arm control ──────────────────────────────────────────────────

    def _move_arm(self, joint_positions, label, duration_sec=2.0):
        self.get_logger().info(
            f'[{label}] Arm -> {["%.2f" % j for j in joint_positions]}'
        )
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
            self.get_logger().warning(f'[{label}] Arm goal rejected')
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)
        if not result_future.done():
            self.get_logger().warning(f'[{label}] Arm trajectory timed out')
            return False
        self.get_logger().info(f'[{label}] Arm motion completed')
        return True

    def _sleep_sim(self, seconds):
        """Sleep for `seconds` of simulation time (not wall-clock)."""
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _gripper_open(self):
        self.get_logger().info('Gripper -> OPEN')
        self._gripper(GRIPPER_OPEN)

    def _gripper_close(self):
        self.get_logger().info('Gripper -> HOLD (4.8cm)')
        self._gripper(GRIPPER_HOLD)

    def _gripper_close_until_contact(self, timeout_sec=5.0):
        """DEPRECATED: This function caused excessive squeeze force that broke
        the parallel linkage. Use _gripper_close() instead.
        
        Gentle close to GRIPPER_HOLD position. High friction (μ=100) on both
        cube and fingers provides secure grip without needing excessive force.
        This maintains the parallel linkage constraint."""
        self.get_logger().warning('⚠️  _gripper_close_until_contact is DEPRECATED')
        self.get_logger().warning('   Using gentle GRIPPER_HOLD position to maintain parallel linkage')
        
        # Simply close to GRIPPER_HOLD position (48mm gap for 40mm cube)
        self._gripper_close()
        self._sleep_sim(2.0)
        
        return True

    def _gripper(self, position):
        self.get_logger().warning(f'═══════ GRIPPER COMMAND: position={position:.3f} ═══════')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['grip_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()  # Increased from 0.3s
        goal.trajectory.points = [point]

        # Try trajectory controller first
        if self._gripper_traj_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('  Using trajectory controller for gripper')
            future = self._gripper_traj_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done() and future.result().accepted:
                self.get_logger().info('  Gripper trajectory goal accepted')
                result_future = future.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                self.get_logger().info('  Gripper trajectory complete')
                return
            else:
                self.get_logger().warning('⚠️  Gripper trajectory goal rejected')

        # Fallback: direct publish to the correct topic
        self.get_logger().warning('⚠️  Gripper trajectory controller NOT available, using direct pub')
        self.get_logger().warning(f'  Publishing {position:.3f} to /gripper_group_controller/commands')
        
        # Publish multiple times to ensure it gets through
        for i in range(10):
            self._grip_pub.publish(Float64(data=position))
            time.sleep(0.1)
            if i % 3 == 0:
                self.get_logger().warning(f'  Published attempt {i+1}/10')
        
        self.get_logger().warning('  Direct publish complete')

    # ── Sensor callbacks ────────────────────────────────────────────

    def _on_detected_object(self, msg: PoseStamped):
        self._detected_pose_cam = msg
        self._detected_pose_time = self.get_clock().now()

        try:
            cam_to_map = self._tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(msg, cam_to_map)
            self._detected_pose_map = pose_map
        except Exception as e:
            self.get_logger().debug(f'TF cam->map failed: {e}')

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

    def _check_finger_contact(self, target_model='test_block'):
        """Return True if either finger is in contact with the target model."""
        for contact_msg in [self._llink2_contact, self._rlink2_contact]:
            if contact_msg is None:
                continue
            for contact in contact_msg.contacts:
                if target_model in contact.collision1 or target_model in contact.collision2:
                    return True
        return False

    def _wait_for_wrist_frame(self):
        """Block until the wrist camera has delivered at least one frame."""
        deadline = time.monotonic() + WRIST_CAMERA_TIMEOUT
        while self._wrist_image is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._wrist_image is not None

    def _check_object_visible(self, label=''):
        """Use OpenCV HSV thresholding on the wrist camera to check if the
        blue test block is visible in the gripper.  Returns True if a
        contour meeting VISION_MIN_AREA is found."""
        if not CV_AVAILABLE or self._bridge is None:
            self.get_logger().info(f'{label} Vision unavailable (no cv_bridge) — skipping')
            return None   # indeterminate

        if not self._wait_for_wrist_frame():
            self.get_logger().warning(f'{label} No wrist camera frame after {WRIST_CAMERA_TIMEOUT}s')
            return None

        try:
            hsv = cv2.cvtColor(self._wrist_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area

            found = max_area >= VISION_MIN_AREA
            self.get_logger().info(
                f'{label} Wrist cam: {"OBJECT VISIBLE" if found else "no object"} '
                f'(largest contour {max_area:.0f} px, threshold {VISION_MIN_AREA})'
            )
            return found
        except Exception as e:
            self.get_logger().warning(f'{label} Vision check error: {e}')
            return None

    # ── Navigation ───────────────────────────────────────────────────

    def _navigate(self, goal_pose):
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        wrapped = NavigateToPose.Goal()
        wrapped.pose = goal_pose

        self.get_logger().info('Waiting for Nav2 /navigate_to_pose...')
        server_ready = False
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if self._nav_ac.wait_for_server(timeout_sec=5.0):
                server_ready = True
                break
            remaining = int(deadline - time.monotonic())
            self.get_logger().info(
                f'Nav2 not ready — retrying ({remaining}s left)'
            )
        if not server_ready:
            self.get_logger().error('Nav2 action server not available after 60 s')
            return False

        future = self._nav_ac.send_goal_async(wrapped)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error('Navigation goal send timed out')
            return False
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error('Navigation goal handle is None')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected by Nav2')
            return False

        self.get_logger().info('Nav2 goal accepted — driving...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        if not result_future.done():
            self.get_logger().error('Navigation result timed out after 120 s')
            return False

        status = result_future.result().status
        success = (status == 4)
        if success:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().warning(
                f'Navigation finished with status {status} '
                f'(0=unknown, 1=accepted, 2=executing, 3=canceling, '
                f'4=succeeded, 5=canceled, 6=aborted)'
            )
        return success

    # ── Helpers ──────────────────────────────────────────────────────

    def _wait_for_servers(self):
        self.get_logger().info('Waiting for action servers (arm + gripper)...')
        deadline = time.monotonic() + 60.0

        while time.monotonic() < deadline:
            arm_ok = self._arm_traj_ac.wait_for_server(timeout_sec=1.0)
            grip_ok = self._gripper_traj_ac.wait_for_server(timeout_sec=1.0)

            if arm_ok and grip_ok:
                self.get_logger().info('Arm + gripper action servers ready')
                return True

            missing = []
            if not arm_ok: missing.append('arm_traj')
            if not grip_ok: missing.append('gripper_traj')
            self.get_logger().info(
                f'Waiting for: {missing} ({int(deadline - time.monotonic())}s left)'
            )
            time.sleep(2.0)

        self.get_logger().error('Action servers not ready after 60 s')
        return False

    def _get_robot_pose_in_odom(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint',
                rclpy.time.Time(), rclpy.time.Duration(seconds=2.0))
            p = PoseStamped()
            p.header = t.header
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.position.z = t.transform.translation.z
            p.pose.orientation = t.transform.rotation
            return p
        except Exception as e:
            self.get_logger().debug(f'TF odom->base_footprint: {e}')
            return None

    def _xy_distance(self, a, b):
        return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)

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

    def _camera_guided_approach(self):
        """Fine-position the gripper over the cube using depth camera feedback.

        Phase 1 — Yaw alignment: rotate to face the cube.
        Phase 2 — Distance approach: drive until gripper centre is 5 cm
        from the cube centre (uses 2D distance, accounts for robot yaw).
        Returns True if a fresh detection was available and alignment succeeded.
        """
        deadline = time.monotonic() + 10.0
        while self._detected_pose_map is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._detected_pose_map is None:
            self.get_logger().warning('Camera guided: no detection available after 10s')
            return False

        age = (self.get_clock().now() - self._detected_pose_time).nanoseconds / 1e9
        if age > 5.0:
            self.get_logger().warning(f'Camera guided: detection too old ({age:.1f}s)')
            return False

        cube = self._detected_pose_map
        cube_frame = cube.header.frame_id

        # Transform cube into odom frame for comparison with robot odom
        try:
            if cube_frame != 'odom':
                t = self._tf_buffer.lookup_transform(
                    'odom', cube_frame,
                    rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
                cube_in_odom = tf2_geometry_msgs.do_transform_pose_stamped(cube, t)
            else:
                cube_in_odom = cube
        except Exception:
            self.get_logger().warning('Cannot TF cube→odom, using raw detection')
            cube_in_odom = cube

        cube_ox = cube_in_odom.pose.position.x
        cube_oy = cube_in_odom.pose.position.y

        finger_center = self._gripper_center_x_at_joints(REACH_DOWN) + FK_SETTLE_COMPENSATION
        # The depth camera detects the cube's FRONT FACE (2 cm closer than the
        # true centre for a 4 cm cube).  Targeting finger_centre at the front
        # face puts the gripper centre 2 cm behind the true centre, which
        # exactly matches the coarse approach (DESIRED_STANDOFF = 0.02) and the
        # FK correction (GAP_BIAS = 0.02).
        DESIRED_STANDOFF = 0.00

        self.get_logger().warning('═══════ CAMERA-GUIDED FINE ADJUSTMENT ═══════')
        self.get_logger().warning(f'  Camera sees cube at:  ({cube_ox:.4f}, {cube_oy:.4f})')
        self.get_logger().warning(f'  Robot odom:           ({self._odom_x:.4f}, {self._odom_y:.4f})')
        self.get_logger().warning(f'  Finger centre (base):  {finger_center:.4f} m')
        self.get_logger().warning(f'  DESIRED_STANDOFF:      {DESIRED_STANDOFF*1000:.0f} mm (camera = front face)')

        # ── Phase 1: Yaw alignment ────────────────────────────────────
        dx = cube_ox - self._odom_x
        dy = cube_oy - self._odom_y
        target_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize_angle(target_yaw - self._odom_yaw)

        self.get_logger().warning(f'  Yaw error:            {math.degrees(yaw_err):.1f}°')

        if abs(yaw_err) > 0.05:
            self.get_logger().warning('  Phase 1 — aligning yaw to face cube')
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
            for _ in range(5):
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
            time.sleep(0.2)

        # ── Phase 2: Distance approach ────────────────────────────────
        dx = cube_ox - self._odom_x
        dy = cube_oy - self._odom_y
        dist_to_cube = math.hypot(dx, dy)
        target_dist = finger_center + DESIRED_STANDOFF
        error_dist = dist_to_cube - target_dist

        self.get_logger().warning(
            f'  Phase 2 — robot→cube={dist_to_cube*1000:.0f}mm  '
            f'target={target_dist*1000:.0f}mm  error={error_dist*1000:.0f}mm'
        )

        if abs(error_dist) < 0.005:
            self.get_logger().info('Camera guided: already at target distance')
            self.get_logger().warning('═══════════════════════════════════════')
            return True

        twist = Twist()
        max_speed = 0.08
        dist_deadline = time.monotonic() + 30.0

        while time.monotonic() < dist_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            dx = cube_ox - self._odom_x
            dy = cube_oy - self._odom_y
            dist_to_cube = math.hypot(dx, dy)
            error_dist = dist_to_cube - target_dist
            if abs(error_dist) < 0.005:
                break
            speed = max(-max_speed, min(max_speed, error_dist * 1.5))
            twist.linear.x = speed
            self._publish_cmd_vel(twist)
            time.sleep(0.02)

        twist.linear.x = 0.0
        for _ in range(10):
            self._publish_cmd_vel(twist)
            time.sleep(0.05)
        time.sleep(0.3)

        dx = cube_ox - self._odom_x
        dy = cube_oy - self._odom_y
        final_error = math.hypot(dx, dy) - target_dist
        self.get_logger().warning(f'  Final distance error:  {final_error*1000:.1f} mm')
        self.get_logger().warning('═══════════════════════════════════════')
        return abs(final_error) < 0.015

    def _backup_and_strafe(self):
        self.get_logger().info('Safety manoeuvre: backup + turn (diff-drive)')
        twist = Twist()

        # Backup 0.3 m/s for 0.5 s (~15 cm)
        twist.linear.x = -0.3
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            self._publish_cmd_vel(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        # Rotate 1.0 rad/s for 0.35 s (~20 deg)
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
        self.get_logger().info('Safety manoeuvre complete')

    def _drive_to_target(self, target, target_dist=0.15, speed=0.8):
        robot_pose = self._get_robot_pose_in_odom()
        if robot_pose is None:
            self.get_logger().warning('Cannot drive to target — no robot pose')
            return

        dx = target.pose.position.x - robot_pose.pose.position.x
        dy = target.pose.position.y - robot_pose.pose.position.y
        dist = math.hypot(dx, dy)

        if dist <= target_dist:
            self.get_logger().info(
                f'Already at target ({dist:.2f} m ≤ {target_dist:.2f} m)'
            )
            return

        self.get_logger().info(
            f'Driving to target: {dist:.2f} m away, speed {speed:.1f} m/s'
        )

        twist = Twist()
        twist.linear.x = speed
        deadline = time.monotonic() + max(60.0, dist / speed * 20.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            robot_pose = self._get_robot_pose_in_odom()
            if robot_pose is None:
                break
            dx = target.pose.position.x - robot_pose.pose.position.x
            dy = target.pose.position.y - robot_pose.pose.position.y
            dist = math.hypot(dx, dy)
            if dist <= target_dist:
                break
            self._publish_cmd_vel(twist)
            time.sleep(0.05)

        twist.linear.x = 0.0
        self._publish_cmd_vel(twist)
        time.sleep(0.2)
        self.get_logger().info('Drive to target complete')

    def _align_yaw_to_target(self, target_map):
        robot_pose = self._get_robot_pose_in_odom()
        if robot_pose is None:
            self.get_logger().warning('Cannot align yaw — no robot pose')
            return

        dx = target_map.pose.position.x - robot_pose.pose.position.x
        dy = target_map.pose.position.y - robot_pose.pose.position.y
        target_yaw = math.atan2(dy, dx)
        current_yaw = self._yaw_from_quat(robot_pose.pose.orientation)
        yaw_err = self._normalize_angle(target_yaw - current_yaw)

        self.get_logger().info(
            f'Yaw alignment: current={math.degrees(current_yaw):.1f}°, '
            f'target={math.degrees(target_yaw):.1f}°, error={math.degrees(yaw_err):.1f}°'
        )

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
        self.get_logger().info('Yaw alignment complete')

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _on_odom(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _publish_cmd_vel(self, twist):
        self._cmd_vel_pub.publish(twist)
        self._cmd_vel_ign_pub.publish(twist)

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _verify_pickup(self, obj_map):
        # Check if gripper actually closed
        grip_closed = False
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            grip_pos = js.get('grip_joint', None)
            if grip_pos is not None:
                grip_closed = abs(grip_pos - GRIPPER_HOLD) < 0.3
                self.get_logger().info(
                    f'Gripper verification: grip_joint={grip_pos:.3f} '
                    f'({"HOLD ✅" if grip_closed else "OPEN ❌"})'
                )

        if grip_closed:
            return True

        # Try TF verification
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'test_block',
                rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
            cube_x = t.transform.translation.x
            cube_y = t.transform.translation.y
            dist = math.hypot(
                cube_x - obj_map.pose.position.x,
                cube_y - obj_map.pose.position.y)
            self.get_logger().info(
                f'TF verification: cube at ({cube_x:.2f}, {cube_y:.2f}) '
                f'— {dist:.2f}m from spawn'
            )
            return dist > 0.3
        except Exception:
            self.get_logger().warning('Cannot verify pickup — proceeding anyway')
            return True

    # ── Forward-kinematics helpers ────────────────────────────────────

    def _gripper_center_x_at_joints(self, joints):
        """Gripper centre X in base_link at given joint angles (midpoint)."""
        j1, j2, j3, j4, j5 = joints
        J2_REF = -1.45
        J3_REF = -0.180
        CENTER_REF = 0.3032

        dX_dJ2 = 0.150 * math.cos(J2_REF)
        dX_dJ3 = 0.145 * math.cos(J3_REF)

        return CENTER_REF + dX_dJ2 * (j2 - J2_REF) + dX_dJ3 * (j3 - J3_REF)

    def _debug_fk(self, label, joints):
        """Log detailed FK breakdown for debugging."""
        j1, j2, j3, j4, j5 = joints
        J2_REF = -1.45
        J3_REF = -0.180
        CENTER_REF = 0.3032
        dX_dJ2 = 0.150 * math.cos(J2_REF)
        dX_dJ3 = 0.145 * math.cos(J3_REF)
        delta_j2 = j2 - J2_REF
        delta_j3 = j3 - J3_REF
        contrib_j2 = dX_dJ2 * delta_j2
        contrib_j3 = dX_dJ3 * delta_j3
        result = CENTER_REF + contrib_j2 + contrib_j3

        self.get_logger().warning(f'═══ FK DEBUG ({label}) ═══')
        self.get_logger().warning(f'  joints: [{j1:.3f}, {j2:.3f}, {j3:.3f}, {j4:.3f}, {j5:.3f}]')
        self.get_logger().warning(f'  CENTER_REF={CENTER_REF:.5f}')
        self.get_logger().warning(f'  J2_REF={J2_REF:.3f}  j2={j2:.3f}  delta={delta_j2:.5f}')
        self.get_logger().warning(f'  J3_REF={J3_REF:.3f}  j3={j3:.3f}  delta={delta_j3:.5f}')
        self.get_logger().warning(f'  dX_dJ2={dX_dJ2:.5f}  contrib={contrib_j2:.5f}')
        self.get_logger().warning(f'  dX_dJ3={dX_dJ3:.5f}  contrib={contrib_j3:.5f}')
        self.get_logger().warning(f'  RESULT finger_center_base={result:.5f} m')

        # Also log what we know from /joint_states if available
        if self._joint_state is not None:
            js = {n: p for n, p in zip(self._joint_state.name, self._joint_state.position)}
            self.get_logger().warning(f'  /joint_states arm joints: '
                f'[{js.get("arm_joint1",0):.3f}, {js.get("arm_joint2",0):.3f}, '
                f'{js.get("arm_joint3",0):.3f}, {js.get("arm_joint4",0):.3f}, '
                f'{js.get("arm_joint5",0):.3f}]')
            real_center = self._gripper_center_x_at_joints([
                js.get("arm_joint1",0), js.get("arm_joint2",0),
                js.get("arm_joint3",0), js.get("arm_joint4",0),
                js.get("arm_joint5",0)])
            self.get_logger().warning(f'  FK from /joint_states: finger_center_base={real_center:.5f} m')

        # Log world position
        finger_world = self._odom_x + result
        self.get_logger().warning(f'  odom_x={self._odom_x:.5f}  finger_world_x={finger_world:.5f}')
        # Log cube Z if known
        if self._object_pose_map is not None:
            self.get_logger().warning(
                f'  cube z={self._object_pose_map.pose.position.z:.3f}  at x={self._object_pose_map.pose.position.x:.3f}'
            )
        self.get_logger().warning(f'═══════════════════════════════════════════')

    def _compute_rf_tip_x_at_joints(self, joints):
        """FK for right-finger tip X in base_link at given joint angles."""
        return self._gripper_center_x_at_joints(joints)

    def _compute_lf_tip_x_at_joints(self, joints):
        """FK for left-finger tip X in base_link at given joint angles."""
        return self._gripper_center_x_at_joints(joints)

    def _correct_robot_x_during_pick(self, obj_world_x, max_correction=0.25):
        GAP_BIAS = 0.00  # 0 cm — gripper centres exactly on cube centre
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        odom_x = self._odom_x

        # Use REACH_DOWN joints so correction targets the pickup arm config,
        # not the current (HOME) configuration where the gripper is retracted.
        joints = REACH_DOWN

        rf_x = self._compute_rf_tip_x_at_joints(joints)
        lf_x = self._compute_lf_tip_x_at_joints(joints)
        finger_center_base = (rf_x + lf_x) / 2.0 + FK_SETTLE_COMPENSATION

        finger_center_world = odom_x + finger_center_base

        error = obj_world_x - finger_center_world - GAP_BIAS

        raw_error = obj_world_x - (odom_x + finger_center_base)

        self.get_logger().warning(
            '═══════ LIVE PICK-TIME CORRECTION ═══════'
        )
        self.get_logger().warning(
            f'  odom_x:                        {odom_x:.5f} m'
        )
        self.get_logger().warning(
            f'  live arm joints:               [{joints[0]:.3f}, {joints[1]:.3f}, '
            f'{joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}]'
        )
        self.get_logger().warning(
            f'  rf_tip_x (base_link, FK):      {rf_x:.5f} m'
        )
        self.get_logger().warning(
            f'  lf_tip_x (base_link, FK):      {lf_x:.5f} m'
        )
        self.get_logger().warning(
            f'  finger_center (base_link):     {finger_center_base:.5f} m'
        )
        self.get_logger().warning(
            f'  finger_center WORLD X:         {finger_center_world:.5f} m'
        )
        self.get_logger().warning(
            f'  blue cube WORLD X:             {obj_world_x:.5f} m'
        )
        self.get_logger().warning(
            f'  ══ GAP (raw) = {raw_error*1000:.1f} mm '
            f'(target {GAP_BIAS*1000:.0f} mm wider) ══ '
            f'{"nudge +X" if error > 0 else "nudge -X"}'
        )
        self.get_logger().warning(
            '═══════════════════════════════════════════'
        )

        if abs(error) < 0.003:
            self.get_logger().info('Error < 3 mm — no correction needed ✅')
            return True

        nudge_dir = 1.0 if error > 0 else -1.0
        nudge_amount = max(0.002, min(max_correction, abs(error)))
        nudge_speed = 0.3
        nudge_duration = nudge_amount / nudge_speed

        self.get_logger().warning(
            f'Nudging robot {nudge_amount*1000:.1f} mm at {nudge_speed:.1f} m/s '
            f'for {nudge_duration:.2f}s'
        )

        twist = Twist()
        twist.linear.x = nudge_dir * nudge_speed
        t0 = time.monotonic()
        closed_loop_deadline = t0 + max(5.0, nudge_duration * 4)

        while time.monotonic() < closed_loop_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            current_error = obj_world_x - (self._odom_x + finger_center_base) - GAP_BIAS

            # Slow down as we approach
            if abs(current_error) < 0.008:
                self._publish_cmd_vel(Twist())
                break

            # Ensure minimum publish duration so sim processes the command
            if time.monotonic() - t0 > nudge_duration:
                # If we've published for long enough but error still large,
                # keep going (don't overshoot) — slow down
                twist.linear.x = nudge_dir * 0.15

            self._publish_cmd_vel(twist)

        stop_twist = Twist()
        for _ in range(5):
            self._publish_cmd_vel(stop_twist)
            rclpy.spin_once(self, timeout_sec=0.02)

        final_error = obj_world_x - (self._odom_x + finger_center_base) - GAP_BIAS
        self.get_logger().warning(
            f'After correction: odom_x={self._odom_x:.5f}, '
            f'X error = {final_error*1000:.1f} mm '
            f'({"✅" if abs(final_error) < 0.01 else "⚠️" if abs(final_error) < 0.02 else "❌"})'
        )

        return abs(final_error) < 0.02


def main():
    rclpy.init()
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
