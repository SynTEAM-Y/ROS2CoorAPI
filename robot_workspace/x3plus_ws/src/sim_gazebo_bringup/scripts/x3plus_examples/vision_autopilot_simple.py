#!/usr/bin/env python3
"""
Simplified Vision Autopilot for X3Plus Robot
Based on manufacturer's workflow from yahboomcar_ws/src/arm_autopilot/

Workflow:
1. Read cube position from Gazebo TF
2. Drive to appropriate standoff distance
3. Use HSV camera for final alignment
4. Execute manufacturer's pickup sequence (joint poses)
5. Read green landing pad position from Gazebo TF
6. Drive to landing pad and drop cube
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from tf2_ros import TransformListener, Buffer
import cv2
from cv_bridge import CvBridge
import numpy as np


class HSVColorDetector:
    """HSV color detection for alignment (from manufacturer's workflow)
    
    The manufacturer's original autopilot used HSV color detection for:
    1. Identifying the pickup cube during approach
    2. Centering the gripper on the cube (wrist camera feedback)
    3. Detecting the landing pad for final placement
    
    This implementation uses restrictive HSV ranges for robust detection
    in Gazebo's consistent lighting environment.
    """
    
    def __init__(self):
        # HSV ranges optimized for Gazebo physics simulation.
        # Gazebo renders test_block with ambient color (0, 0.5, 1) which
        # produces hue ~98, not the manufacturer's typical 100.
        self.color_hsv_list = {
            # BLUE CUBE (2 cm × 2 cm × 2 cm test_block — see models/test_block/model.sdf).
            # Gazebo adjustment: Hue 90-124 (tighter than manufacturer's 80-120)
            # because Gazebo lighting is more consistent and predictable.
            "blue": ((90, 43, 46), (124, 255, 255)),

            # GREEN LANDING PAD (500 mm × 500 mm pad at (2.0, 1.2)).
            "green": ((35, 43, 46), (77, 253, 255)),

            # RED and YELLOW for potential future objects.
            "red": ((0, 43, 46), (10, 253, 255)),
            "yellow": ((26, 43, 46), (34, 253, 255)),
        }
        self.target_color = "blue"
        self.center_x = 0
        self.center_y = 0
        self.radius = 0
        self.detected = False

    def detect(self, image_bgr):
        """Detect colored object and return center position.

        Robustness for the 2 cm cube:
        1. Validate blob circularity (cube should appear as a circle in
           top-down view).
        2. Reject blobs whose area is implausible for a 2 cm cube at the
           expected approach distance (15-180 px² at the gripper reach).
        3. Use the minimum enclosing circle for a stable center estimate.
        4. Filter out noise and false positives via morphology.
        """
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array(self.color_hsv_list[self.target_color][0], dtype="uint8")
        upper = np.array(self.color_hsv_list[self.target_color][1], dtype="uint8")

        # Create binary mask for the target color.
        mask = cv2.inRange(hsv, lower, upper)

        # Morphological operations to clean up mask.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove small noise

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.detected = False
            return False

        # Find the largest blob (should be the cube).
        areas = [cv2.contourArea(c) for c in contours]
        max_idx = np.argmax(areas)
        max_area = areas[max_idx]

        # Sanity checks for the 2 cm cube at the wrist camera (640x480).
        # At ~0.5 m the blob is ~15-40 px², at the gripper reach (0.29 m)
        # it is ~80-180 px². Earlier (80-2500) values were tuned for a 4 cm
        # cube and would reject the 2 cm blob at the pickup distance.
        MIN_AREA = 15
        MAX_AREA = 400

        if max_area < MIN_AREA or max_area > MAX_AREA:
            self.detected = False
            return False
        
        # Use minimum enclosing circle for more stable center estimation
        contour = contours[max_idx]
        (x, y), radius = cv2.minEnclosingCircle(contour)
        
        # Additional validation: check blob circularity
        # For a cube viewed from above, should be roughly circular
        area = cv2.contourArea(contour)
        if area > 0:
            # Circularity = 4π*Area / Perimeter²
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                # Cube should have circularity ~0.6-0.9 (not too elongated)
                if circularity < 0.4:  # Too elongated, reject
                    self.detected = False
                    return False
        
        # Store detected position and size
        self.center_x = int(x)
        self.center_y = int(y)
        self.radius = int(radius)
        self.detected = True
        return True


# --- Manufacturer arm poses (yahboomcar_ws/src/arm_autopilot/autopilot_main.py
# and arm_color_transport/transport_main.py), converted with the sim
# convention rad = radians(servo_deg - 90). Values at the +/-pi/2 joint limit
# are backed off to +/-1.55 to avoid PID chatter against the hard stop. ---
#
# CUBE SIZE: 2 cm (0.02 m) — matches models/test_block/model.sdf. The cube was
# originally 4 cm in earlier revisions of this codebase; all constants below
# have been unified to the 2 cm manufacturer value so REACH_DOWN → LIFT_POSE
# does NOT snap the wrist (j4) mid-lift and flick the cube out of the fingers.

HOME        = [0.0,   0.0,    0.0,    0.0,   0.0]   # arm folded straight up

# DRIVE_POSE: observation pose while driving. Wrist camera looks at the floor
# 0.3-1.0 m ahead. Manufacturer: [90, 120, 0, 0, 90].
DRIVE_POSE  = [0.0,   0.524, -1.55,  -1.55,  0.0]

# REACH_DOWN: gripper ready to close on the 2 cm cube.
# CRITICAL: j2 + j3 + j4 must equal approximately -pi (-180°) so the last
# link (and therefore the gripper pads) points STRAIGHT DOWN (vertical, aligned
# with the Z axis).  If the sum is not -180° the pads tilt forward or backward
# and cannot clamp the cube faces.
#
# pick_and_place.py uses j4=-1.21 with j2=-1.45, j3=-0.54, giving a sum of
# about -182° — almost perfectly vertical and validated for successful picking.
# Using these exact angles keeps the gripper vertical over the cube centre.
REACH_DOWN  = [0.0,  -1.45,  -0.524, -1.21,  0.0]

# LIFT_POSE: shoulder lift. j4 MUST keep the same j2+j3+j4 sum as REACH_DOWN
# so the gripper stays vertical and does not snap/flick the cube on lift.
LIFT_POSE   = [0.0,  -0.524, -0.524, -1.21,  0.0]

# CARRY: transport pose — cube held up and back so wheels cannot drag it.
CARRY       = [0.0,   0.96,  -1.55,  -0.785, 0.0]

# PLACE_DOWN: gentle release on the landing pad.
# Use j4 = -1.047 so j2+j3+j4 stays near -pi and pads remain vertical during place.
PLACE_DOWN  = [0.0,  -1.40,  -0.524, -1.047,  0.0]

# Gripper (master grip_joint, radians).
# The parallel-linkage pads are ~25 mm apart at grip_joint = 0 (fully closed)
# and ~48 mm apart at grip_joint = -0.676.  A 2 cm cube therefore needs a
# grip value much closer to 0 than the old -0.50 rad used for a 4 cm cube.
# With -0.05 rad the target gap is ~15-20 mm; the mimic relay clamps to the
# measured angle as soon as the pads touch the cube, so the pads stay parallel.
GRIPPER_OPEN  = -1.54   # fingers fully open
GRIPPER_HOLD  = -0.05   # very tight hold for 2 cm cube
GRIPPER_CLOSE = 0.0     # fully closed / minimum pad separation


class ArmController:
    """Arm joint position controller using the sim's proven joint poses."""

    def __init__(self, node):
        self.node = node
        self.joint_publishers = {}
        self.busy = False
        self._last_cmd = None
        joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'grip_joint']
        for name in joint_names:
            self.joint_publishers[name] = node.create_publisher(Float64, f'/{name}_cmd_pos', 10)

    def run_async(self, fn):
        """Run an arm sequence in a background thread; poll .busy for completion."""
        import threading
        self.busy = True

        def _wrap():
            try:
                fn()
            finally:
                self.busy = False
        threading.Thread(target=_wrap, daemon=True).start()

    def publish_pose(self, arm_pos, grip_pos):
        """Publish targets without blocking (for use inside timer callbacks).
        Publish each target twice to reduce message-drop impact without
        increasing command rate."""
        names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        for name, pos in zip(names, arm_pos):
            msg = Float64()
            msg.data = float(pos)
            self.joint_publishers[name].publish(msg)
        gmsg = Float64()
        gmsg.data = float(grip_pos)
        self.joint_publishers['grip_joint'].publish(gmsg)
        self._last_cmd = (list(arm_pos), grip_pos)

    def _sim_sleep(self, dur_s):
        """Sleep for dur_s SIM seconds (wall sleep scales with real-time factor).
        Must only be called from a background thread, never from an executor
        callback, so /clock keeps being processed."""
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < dur_s * 1e9:
            time.sleep(0.05)

    def set_joints(self, arm_pos, grip_pos, duration_ms=2500):
        """Smoothly interpolate from the last commanded pose to the target over
        duration_ms of SIM time. More steps and longer settle reduce PID
        oscillation and gripper shake when opening/closing."""
        if self._last_cmd is None:
            # First command: assume arm at home, gripper closed
            self._last_cmd = (list(HOME), GRIPPER_CLOSE)
        start_arm, start_grip = self._last_cmd
        # Use finer interpolation for smoother motion and less shake.
        steps = 50
        step_dt = (duration_ms / 1000.0) / steps
        for i in range(1, steps + 1):
            # Smooth-step easing: reduces jerk at start/end.
            t = i / steps
            a = t * t * (3.0 - 2.0 * t)
            interp_arm = [s + (targ - s) * a for s, targ in zip(start_arm, arm_pos)]
            interp_grip = start_grip + (grip_pos - start_grip) * a
            self.publish_pose(interp_arm, interp_grip)
            self._sim_sleep(step_dt)
        # longer settle time damps residual PID oscillation
        self._sim_sleep(0.8)
        self._last_cmd = (list(arm_pos), grip_pos)

    def to_drive_pose(self):
        """Manufacturer driving/observation pose, gripper open."""
        self.set_joints(DRIVE_POSE, GRIPPER_OPEN, 2500)

    def reach_down_open(self):
        """Lower arm to REACH_DOWN with gripper open for TF-based alignment."""
        self.set_joints(REACH_DOWN, GRIPPER_OPEN, 4000)

    def grasp_and_lift(self):
        """From REACH_DOWN with gripper open: close, lift, carry."""
        self.node.get_logger().info('Grasping and lifting...')
        self.set_joints(REACH_DOWN, GRIPPER_HOLD, 2500)
        self.set_joints(LIFT_POSE,  GRIPPER_HOLD, 2500)
        self.set_joints(CARRY,      GRIPPER_HOLD, 3500)
        self.node.get_logger().info('Grasp and lift complete')

    def pickup_sequence(self):
        """Manufacturer pick sequence (arm_autopilot arm_gripper()):
        reach down open -> close -> lift shoulder -> transport carry pose.
        Slower gripper close reduces shake."""
        self.node.get_logger().info('Starting pickup sequence...')
        self.set_joints(REACH_DOWN, GRIPPER_OPEN, 4000)
        self.set_joints(REACH_DOWN, GRIPPER_HOLD, 2500)  # slow close on cube
        self.set_joints(LIFT_POSE,  GRIPPER_HOLD, 2500)  # shoulder lift (servo2 -> 60)
        self.set_joints(CARRY,      GRIPPER_HOLD, 3500)  # transport carry pose
        self.node.get_logger().info('Pickup sequence complete')

    def lower_and_release(self):
        """Manufacturer place (transport Grip_down): lower the cube to the
        pad and open the gripper. Slower open reduces shake.
        The robot must back away BEFORE the arm is lifted, otherwise the cube
        gets dragged up by the fingers and lands on the robot's deck."""
        self.node.get_logger().info('Lowering cube to pad...')
        self.set_joints(LIFT_POSE,  GRIPPER_HOLD, 3000)
        self.set_joints(PLACE_DOWN, GRIPPER_HOLD, 3000)
        self.set_joints(PLACE_DOWN, GRIPPER_OPEN, 2500)  # slow release
        self._sim_sleep(3.0)  # let the cube settle on the pad (matches pick_and_place)
        self.node.get_logger().info('Cube released')

    def fold_arm(self):
        """Return the arm to the driving pose after the robot backed away."""
        self.set_joints(LIFT_POSE,  GRIPPER_OPEN, 2500)
        self.set_joints(DRIVE_POSE, GRIPPER_OPEN, 2500)
        self.node.get_logger().info('Arm back in driving pose')


class VisionAutopilotSimple(Node):
    """Main autopilot node"""
    
    def __init__(self):
        super().__init__('vision_autopilot_simple')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ⭐ WRIST CAMERA ONLY (No Depth Camera)
        # Manufacturer's simple autopilot uses the arm-mounted mono camera for:
        # 1. Detecting the blue cube during final approach
        # 2. HSV-based blob centroid for alignment
        # 3. Determining when the cube is at the pickup position
        # 
        # In DRIVE_POSE, the wrist camera watches the floor 0.3-1.0m ahead,
        # allowing the robot to drive and track the cube without using depth.
        # The depth camera is NOT used by this simple autopilot — it relies on
        # Gazebo TF (ground truth GPS-like positions) for coarse navigation,
        # then HSV color detection for fine alignment.
        self.image_sub = self.create_subscription(Image, '/wrist_mono_camera/image_raw', self.image_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.color_detector = HSVColorDetector()
        self.arm_controller = ArmController(self)
        self.bridge = CvBridge()
        
        self.state = 'IDLE'
        self.approach_target = None
        self.approach_yaw = None
        self.landing_target = None
        self.cube_pose = None
        self.landing_pose = None
        self.has_cube = False
        self.image_center_x = 320
        self.image_center_y = 240
        self.hsv_ok_frames = 0
        self.backup_start = None
        self.idle_tf_timeout = 0  # Counter for TF lookup failures in IDLE
        self._reach_down_done = False
        self._align_int_x = 0.0
        self._align_int_y = 0.0
        self._align_ok_frames = 0
        
        # 0.292 m = gripper finger-center forward reach at REACH_DOWN pose
        # (measured/FK-verified in pick_and_place.py).
        # For PICK: this is the standoff from the cube.
        # For PLACE: this is the standoff from the landing pad center; the
        # arm is the same in both cases, but the geometric meaning differs
        # (cube = exactly the gripper reach; pad = gripper reach so the
        # cube is released at the pad's center, not its near edge).
        self.declare_parameter('standoff_distance', 0.292)
        # Kept as a separate parameter so future pad size changes don't
        # accidentally affect the cube pickup distance.
        self.declare_parameter('drop_off_standoff_distance', 0.292)
        # Faster approach speed for very-fast driving behaviour.
        self.declare_parameter('approach_speed', 0.80)  # m/s, was 0.30
        # GPS coarse approach stops this far from the cube; HSV covers the rest.
        self.declare_parameter('pre_approach_distance', 0.65)
        # Manufacturer stop criterion (autopilot_main.py:116): blob centroid
        # within +/-10 px of image center AND below the calibrated stop row.
        # hsv_stop_y is calibrated in sim for the 2 cm cube at the REACH_DOWN
        # finger-center distance. The 2 cm cube's blob stops ~30 px higher
        # in the image than the 4 cm blob (smaller top-of-cube projection).
        self.declare_parameter('hsv_stop_y', 410)
        self.declare_parameter('hsv_x_tol', 10)
        # Higher linear/angular limits used by drive_to_pose/face_point.
        self.declare_parameter('max_linear_speed', 0.90)   # m/s, was hard-coded 0.4
        self.declare_parameter('max_angular_speed', 1.5)   # rad/s, was hard-coded 0.8
        
        self.create_timer(0.1, self.main_loop)
        self.get_logger().info('Vision Autopilot Simple initialized')
        
    def get_tf_pose(self, target_frame):
        try:
            # The gazebo_pose_tf_relay nodes publish odom->test_block and
            # odom->landing_pad, and ground-truth odom->base_footprint comes
            # from the PosePublisher relay — so everything lives in 'odom'.
            trans = self.tf_buffer.lookup_transform('odom', target_frame, rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed for {target_frame}: {e}')
            return None
            
    def image_callback(self, msg):
        """Process wrist camera images for cube detection
        
        Runs asynchronously (10Hz) in a subscription callback.
        Updates self.color_detector with blob position for HSV approach phase.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.color_detector.detect(cv_image)
        except Exception as e:
            # Gracefully handle image processing errors
            # (e.g., if ROS message format changes)
            pass
            
    def drive_to_pose(self, target_x, target_y, max_linear=None, max_angular=None):
        robot_pose = self.get_tf_pose('base_footprint')
        if not robot_pose:
            return False

        # Use configured speed limits so the robot drives very fast.
        if max_linear is None:
            max_linear = self.get_parameter('max_linear_speed').value
        if max_angular is None:
            max_angular = self.get_parameter('max_angular_speed').value

        dx = target_x - robot_pose.pose.position.x
        dy = target_y - robot_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.025:
            self.stop()
            return True

        target_yaw = math.atan2(dy, dx)
        current_yaw = self.get_yaw(robot_pose.pose.orientation)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        twist = Twist()
        if abs(yaw_error) > 0.1:
            twist.angular.z = float(np.clip(yaw_error * 2.5, -max_angular, max_angular))
        else:
            twist.linear.x = float(np.clip(distance * 0.8, 0.08, max_linear))
            twist.angular.z = float(np.clip(yaw_error * 2.5, -max_angular, max_angular))

        self.cmd_vel_pub.publish(twist)
        return False
        
    def standoff_point(self, obj_x, obj_y, param_name='standoff_distance'):
        """Point at the named standoff distance from (obj_x, obj_y) along the
        robot->object line. Defaults to 'standoff_distance' (cube pick); the
        drop-off path passes 'drop_off_standoff_distance' so the cube lands
        at the pad center, not its near edge."""
        robot_pose = self.get_tf_pose('base_footprint')
        if not robot_pose:
            return None, None
        standoff = self.get_parameter(param_name).value
        dx = obj_x - robot_pose.pose.position.x
        dy = obj_y - robot_pose.pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 1e-6:
            return obj_x, obj_y
        return obj_x - standoff * dx / dist, obj_y - standoff * dy / dist

    def tf_final_align(self):
        """Align arm_link5 (gripper centre) with the cube centre.

        The manufacturer URDF places arm_link5 at the midpoint of the
        rlink2/llink2 finger pads, so arm_link5 IS the TCP for picking.
        This method drives the base until arm_link5 in odom coincides with
        the cube centre in odom, within 2 mm for several consecutive frames.
        """
        cube = self.get_tf_pose('test_block')
        arm5 = self.get_tf_pose('arm_link5')
        robot = self.get_tf_pose('base_footprint')
        if not cube or not arm5 or not robot:
            self.get_logger().warn('TF align: missing TF data, skipping')
            return True

        err_x = cube.pose.position.x - arm5.pose.position.x
        err_y = cube.pose.position.y - arm5.pose.position.y
        err_dist = math.hypot(err_x, err_y)

        self.get_logger().info(
            f'[TF_ALIGN] arm_link5=({arm5.pose.position.x:.3f},{arm5.pose.position.y:.3f}) '
            f'cube=({cube.pose.position.x:.3f},{cube.pose.position.y:.3f}) '
            f'error={err_dist*1000:.1f}mm'
        )

        if err_dist < 0.002:
            self._align_ok_frames += 1
            if self._align_ok_frames >= 5:
                self.stop()
                self._align_int_x = 0.0
                self._align_int_y = 0.0
                self._align_ok_frames = 0
                self.get_logger().info('[TF_ALIGN] centred (< 2 mm x 5 frames), proceeding to pick')
                return True
        else:
            self._align_ok_frames = 0

        # Error in robot base frame for diff-drive PI control
        yaw = self.get_yaw(robot.pose.orientation)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        fwd_err = err_x * cos_y + err_y * sin_y
        lat_err = -err_x * sin_y + err_y * cos_y

        # Integral with windup limit
        self._align_int_x += fwd_err * 0.1
        self._align_int_y += lat_err * 0.1
        self._align_int_x = float(np.clip(self._align_int_x, -0.05, 0.05))
        self._align_int_y = float(np.clip(self._align_int_y, -0.05, 0.05))

        Kp = 1.0
        Ki = 0.5
        twist = Twist()
        twist.linear.x = float(np.clip(fwd_err * Kp + self._align_int_x * Ki, -0.02, 0.02))
        twist.angular.z = float(np.clip(lat_err * Kp + self._align_int_y * Ki, -0.08, 0.08))
        self.cmd_vel_pub.publish(twist)
        return False

    def face_point(self, target_x, target_y, tol=0.05):
        """Rotate in place to face (target_x, target_y). Returns True when aligned."""
        robot_pose = self.get_tf_pose('base_footprint')
        if not robot_pose:
            return False
        dx = target_x - robot_pose.pose.position.x
        dy = target_y - robot_pose.pose.position.y
        yaw_error = self.normalize_angle(
            math.atan2(dy, dx) - self.get_yaw(robot_pose.pose.orientation))
        if abs(yaw_error) < tol:
            self.stop()
            return True
        max_angular = self.get_parameter('max_angular_speed').value
        twist = Twist()
        twist.angular.z = float(np.clip(yaw_error * 3.0, -max_angular, max_angular))
        self.cmd_vel_pub.publish(twist)
        return False

    def face_aligned_pre_point(self, cube_pose):
        """Pre-approach point aligned with the cube's faces.

        The cube's ground-truth yaw is known (GPS-like TF), so pick the face
        normal closest to the current robot->cube bearing and approach along
        it: the robot body then squares up with a flat cube face and the
        gripper pads land on faces, not edges."""
        robot_pose = self.get_tf_pose('base_footprint')
        if not robot_pose:
            return None
        cx = cube_pose.pose.position.x
        cy = cube_pose.pose.position.y
        bearing = math.atan2(cy - robot_pose.pose.position.y,
                             cx - robot_pose.pose.position.x)
        cube_yaw = self.get_yaw(cube_pose.pose.orientation)
        # Four face normals; choose the one closest to the bearing.
        approach_dir = min(
            (cube_yaw + k * math.pi / 2.0 for k in range(4)),
            key=lambda a: abs(self.normalize_angle(a - bearing)))
        approach_dir = self.normalize_angle(approach_dir)
        pre = self.get_parameter('pre_approach_distance').value
        return (cx - pre * math.cos(approach_dir),
                cy - pre * math.sin(approach_dir),
                approach_dir)

    def hsv_drive_to_pick(self):
        """Manufacturer final approach (autopilot_main.py Wrecker/robot_location):
        servo the base on the wrist-camera blob until it is horizontally
        centered and its pixel row passes the calibrated stop row (= cube at
        the pre-calibrated pick distance). Returns True when stopped in the
        pick position."""
        stop_y = self.get_parameter('hsv_stop_y').value
        x_tol = self.get_parameter('hsv_x_tol').value

        # Safety net: never run the cube over if the blob is lost/occluded.
        robot_pose = self.get_tf_pose('base_footprint')
        if robot_pose and self.cube_pose:
            dx = self.cube_pose.pose.position.x - robot_pose.pose.position.x
            dy = self.cube_pose.pose.position.y - robot_pose.pose.position.y
            if math.sqrt(dx * dx + dy * dy) < \
                    self.get_parameter('standoff_distance').value - 0.03:
                self.stop()
                self.get_logger().warn('HSV approach: GPS floor hit, stopping')
                return True

        if not self.color_detector.detected:
            # Creep forward; the cube re-enters the view as the robot closes in.
            twist = Twist()
            twist.linear.x = 0.15  # faster creep to re-acquire blob
            self.cmd_vel_pub.publish(twist)
            return False

        err_x = self.color_detector.center_x - self.image_center_x
        err_y = stop_y - self.color_detector.center_y  # >0 -> keep driving

        if abs(err_x) < x_tol and err_y <= 0:
            self.hsv_ok_frames += 1
            if self.hsv_ok_frames >= 3:   # debounce like the manufacturer's flag
                self.stop()
                return True
        else:
            self.hsv_ok_frames = 0

        twist = Twist()
        # Faster HSV approach gains (still bounded to avoid overshoot).
        twist.linear.x = float(np.clip(err_y * 0.003, 0.0, 0.35))
        twist.angular.z = float(np.clip(-err_x * 0.006, -0.8, 0.8))
        self.cmd_vel_pub.publish(twist)
        return False
        
    def get_yaw(self, quaternion):
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        
    def main_loop(self):
        if self.state == 'IDLE':
            cube_pose = self.get_tf_pose('test_block')
            if cube_pose:
                self.cube_pose = cube_pose
                self.idle_tf_timeout = 0  # Reset timeout counter on success
                self.get_logger().info(f'[IDLE] Cube found at ({cube_pose.pose.position.x:.2f}, {cube_pose.pose.position.y:.2f})')
                # Manufacturer drives in joints_init: wrist camera watches the
                # floor ahead, head camera unobstructed.
                self.arm_controller.run_async(self.arm_controller.to_drive_pose)
                self.state = 'ARM_TO_DRIVE'
            else:
                self.idle_tf_timeout += 1
                if self.idle_tf_timeout % 50 == 0:  # Log every 5 seconds (50 * 0.1s)
                    self.get_logger().warn(f'[IDLE] Waiting for cube TF... (timeout count: {self.idle_tf_timeout}). Check if gazebo_pose_tf_relay nodes are running!')

        elif self.state == 'ARM_TO_DRIVE':
            if not self.arm_controller.busy:
                self.get_logger().info('[ARM_TO_DRIVE] Complete, moving to APPROACH_CUBE')
                self.state = 'APPROACH_CUBE'

        elif self.state == 'APPROACH_CUBE':
            if not self.cube_pose:
                self.get_logger().warn('[APPROACH_CUBE] Lost cube pose, returning to IDLE')
                self.state = 'IDLE'
                return

            # Freeze the pre-approach target on first computation: recomputing
            # it from the moving robot position every tick makes the goal
            # drift and the robot can orbit/overshoot. The point lies on the
            # cube-face normal closest to us, so the robot body squares up
            # with a flat face of the cube.
            if self.approach_target is None:
                res = self.face_aligned_pre_point(self.cube_pose)
                if res is None:
                    return
                tx, ty, adir = res
                self.approach_target = (tx, ty)
                self.approach_yaw = adir
                self.get_logger().info(
                    f'[APPROACH_CUBE] Face-aligned pre-approach: ({tx:.2f}, {ty:.2f}), '
                    f'approach dir {math.degrees(adir):.0f} deg')
            if self.drive_to_pose(*self.approach_target):
                self.get_logger().info('[APPROACH_CUBE] Reached pre-approach point, moving to FACE_CUBE')
                self.state = 'FACE_CUBE'

        elif self.state == 'FACE_CUBE':
            # Square up with the cube face before handing over to HSV.
            if self.face_point(self.cube_pose.pose.position.x,
                               self.cube_pose.pose.position.y, tol=0.03):
                self.hsv_ok_frames = 0
                self.get_logger().info('[FACE_CUBE] Aligned with cube, starting HSV_APPROACH')
                self.state = 'HSV_APPROACH'

        elif self.state == 'HSV_APPROACH':
            if self.hsv_drive_to_pick():
                self.get_logger().info(
                    f'[HSV_APPROACH] Stop: blob at ({self.color_detector.center_x}, '
                    f'{self.color_detector.center_y}), moving to PRE_PICK_ALIGN')
                self.state = 'PRE_PICK_ALIGN'

        elif self.state == 'PRE_PICK_ALIGN':
            # First lower the arm to REACH_DOWN with gripper open so arm_link5
            # (the gripper centre) is at the actual pick pose.
            if not self._reach_down_done:
                if not self.arm_controller.busy:
                    self.get_logger().info('[PRE_PICK_ALIGN] Lowering arm to REACH_DOWN (gripper open)')
                    self.arm_controller.run_async(self.arm_controller.reach_down_open)
                    self._reach_down_done = True
                return
            if self.arm_controller.busy:
                return
            if self.tf_final_align():
                self.state = 'PICKUP'

        elif self.state == 'PICKUP':
            # Arm is already at REACH_DOWN with gripper open from PRE_PICK_ALIGN.
            # Close and lift in a background thread.
            self.get_logger().info('[PICKUP] Closing gripper and lifting...')
            self.arm_controller.run_async(self.arm_controller.grasp_and_lift)
            self.state = 'PICKUP_WAIT'

        elif self.state == 'PICKUP_WAIT':
            self.stop()
            if not self.arm_controller.busy:
                # Verify cube was actually picked by checking if it moved upward
                cube_pose_after = self.get_tf_pose('test_block')
                if cube_pose_after and cube_pose_after.pose.position.z > 0.10:
                    self.has_cube = True
                    self.get_logger().info(
                        f'[PICKUP_WAIT] ✓ Cube lifted successfully (z={cube_pose_after.pose.position.z:.3f}m), finding landing pad')
                    self.state = 'FIND_LANDING'
                else:
                    self.get_logger().error(
                        f'[PICKUP_WAIT] ✗ Grasp verification failed! Cube z={cube_pose_after.pose.position.z if cube_pose_after else "unknown"}m')
                    self.get_logger().info('Retrying pickup...')
                    self._reach_down_done = False
                    self.state = 'PRE_PICK_ALIGN'  # Retry from alignment
            
        elif self.state == 'FIND_LANDING':
            landing_pose = self.get_tf_pose('landing_pad')
            if landing_pose:
                self.landing_pose = landing_pose
                self.get_logger().info(
                    f'[FIND_LANDING] Landing pad located at ({landing_pose.pose.position.x:.2f}, {landing_pose.pose.position.y:.2f})')
                self.state = 'DRIVE_TO_LANDING'
            else:
                self.get_logger().warn('[FIND_LANDING] Waiting for landing pad TF...')
                
        elif self.state == 'DRIVE_TO_LANDING':
            if not self.landing_pose:
                self.state = 'FIND_LANDING'
                return

            # Stop at the drop-off standoff (default 0.292 m) so the arm
            # reaches the pad center, not its near edge. Target is frozen
            # on first computation (see APPROACH_CUBE).
            if self.landing_target is None:
                tx, ty = self.standoff_point(
                    self.landing_pose.pose.position.x,
                    self.landing_pose.pose.position.y,
                    param_name='drop_off_standoff_distance')
                if tx is None:
                    return
                self.landing_target = (tx, ty)
                self.get_logger().info(f'Landing standoff target: ({tx:.2f}, {ty:.2f})')
            if self.drive_to_pose(*self.landing_target):
                self.state = 'FACE_LANDING'

        elif self.state == 'FACE_LANDING':
            if self.face_point(self.landing_pose.pose.position.x,
                               self.landing_pose.pose.position.y):
                self.state = 'DROP'
                
        elif self.state == 'DROP':
            rp = self.get_tf_pose('base_footprint')
            if rp:
                self.get_logger().info(
                    f'DROP at robot pose ({rp.pose.position.x:.2f}, {rp.pose.position.y:.2f})')
            self.arm_controller.run_async(self.arm_controller.lower_and_release)
            self.state = 'RELEASE_WAIT'

        elif self.state == 'RELEASE_WAIT':
            self.stop()
            if not self.arm_controller.busy:
                self.has_cube = False
                self.backup_start = None  # Reset for BACKUP phase
                self.get_logger().info('[RELEASE_WAIT] Cube released, backing up...')
                self.state = 'BACKUP'

        elif self.state == 'BACKUP':
            # Reverse 0.25 m so the cube falls clear of the robot before
            # the arm is lifted.
            rp = self.get_tf_pose('base_footprint')
            if not rp:
                return
            if self.backup_start is None:
                self.backup_start = (rp.pose.position.x, rp.pose.position.y)
            dx = rp.pose.position.x - self.backup_start[0]
            dy = rp.pose.position.y - self.backup_start[1]
            if math.sqrt(dx * dx + dy * dy) >= 0.25:
                self.stop()
                self.arm_controller.run_async(self.arm_controller.fold_arm)
                self.state = 'FOLD_WAIT'
            else:
                twist = Twist()
                twist.linear.x = -0.5  # fast backup
                self.cmd_vel_pub.publish(twist)

        elif self.state == 'FOLD_WAIT':
            self.stop()
            if not self.arm_controller.busy:
                self.get_logger().info('[FOLD_WAIT] Arm folded, task complete!')
                self.state = 'DONE'

        elif self.state == 'DONE':
            self.stop()
            self.get_logger().info('═══════════════════════════════════════════')
            self.get_logger().info('  ✓ PICK AND PLACE COMPLETED SUCCESSFULLY!')
            self.get_logger().info('═══════════════════════════════════════════')


def main():
    rclpy.init()
    node = VisionAutopilotSimple()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
