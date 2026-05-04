#!/usr/bin/env python3
"""
Differential Drive Robot Manual Control - Keyboard Teleop with 90° Turn

This node provides manual control of the differential drive robot (X3plus) with:
- Linear movement (forward/backward)
- Angular movement (rotation left/right)
- Automated 90-degree turn with closed-loop odometry feedback

90-DEGREE TURN — CLOSED-LOOP WITH ODOMETRY FEEDBACK:
=====================================================
Instead of open-loop timing (time.sleep), this node subscribes to /odom
and tracks accumulated yaw rotation. The turn stops when the actual
measured rotation reaches π/2 radians (90°).

THEORETICAL FORMULA (for reference):
  Angular velocity: ω = 2v / L  (rad/s)
  Turn duration:    t = (π/2) / ω = π·L / (4·v)  (seconds)

ROBOT PARAMETERS (from base_link.STL mesh analysis):
  Wheel separation L = 0.2128 m  (left y=0.1064, right y=-0.1064)
  Wheel radius     r = 0.04 m    (tire diameter 0.08 m)
  Wheelbase          = 0.220 m   (front x=0.1054, back x=-0.1146)

EXAMPLE CALCULATION:
  v = 0.5 m/s (wheel speed)
  ω = 2×0.5 / 0.2128 = 4.699 rad/s
  t = π×0.2128 / (4×0.5) = 0.334 seconds (open-loop estimate)
  Actual execution uses odometry feedback → exact 90°

Usage:
    ros2 run x3plus_examples manual_control

Keyboard Controls:
    w/s  - Forward/Backward movement
    a/d  - Rotate left/right (in-place)
    Space - Emergency stop
    1    - Execute 90° left turn (in-place)
    2    - Execute 90° right turn (in-place)
    3    - Execute 90° left turn (moving forward)
    4    - Execute 90° right turn (moving forward)
    q    - Quit
"""

import math
import time
import sys
import termios
import threading
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class DifferentialDriveControl(Node):
    """Control node for differential drive robot with 90° turn capability"""
    
    def __init__(self):
        super().__init__('differential_drive_control')
        
        # ROS Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # ============ ROBOT CONFIGURATION (CUSTOMIZE AS NEEDED) ============
        # Differential drive parameters - MUST MATCH diff_drive_simulator!
        self.wheel_separation = 0.2128  # Distance between left and right wheels (m)
        self.wheel_radius = 0.04      # Radius of each wheel (m)
        
        # Speed limits
        self.max_linear_velocity = 0.8   # Maximum forward/backward speed (m/s)
        self.max_angular_velocity = 1.0  # Maximum rotation speed (rad/s)
        
        # Turn speed (used for 90° turns)
        self.turn_wheel_speed = 0.5  # Wheel speed during 90° turn (m/s)
        
        # Current velocity commands
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Yaw feedback for closed-loop turns
        self.odom_yaw = 0.0
        self.imu_yaw = None  # None until first IMU msg arrives
        self.imu_last_rx = None  # wall-clock time of last IMU msg (liveness)
        self.imu_last_sim_t = None  # sim-time of last IMU msg (integration dt)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)

        # Explicit override: pass `kinematic_mode:=true` (ROS param) when
        # launching against the RViz-only diff_drive_simulator. When true we
        # always use /odom and never apply Gazebo coast/brake compensation.
        self.declare_parameter('kinematic_mode', False)
        self.kinematic_mode_param = bool(
            self.get_parameter('kinematic_mode').get_parameter_value().bool_value
        )
        if self.kinematic_mode_param:
            self.get_logger().info('kinematic_mode=true — forcing /odom feedback, no brake pulse')
        
        # Timer for command publishing (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_velocity)
        
        # Log configuration
        self.get_logger().info(
            f"\n{'='*70}\n"
            f"DIFFERENTIAL DRIVE ROBOT CONTROL - INITIALIZED\n"
            f"{'='*70}\n"
            f"Robot Configuration:\n"
            f"  Wheel Separation (L): {self.wheel_separation} m\n"
            f"  Wheel Radius: {self.wheel_radius} m\n"
            f"  Max Linear Velocity: {self.max_linear_velocity} m/s\n"
            f"  Max Angular Velocity: {self.max_angular_velocity} rad/s\n"
            f"  Turn Speed: {self.turn_wheel_speed} m/s\n"
            f"{'='*70}\n"
        )
    
    def publish_velocity(self):
        """Timer callback: publish velocity commands to robot"""
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(twist)
    
    def odom_callback(self, msg):
        """Track current yaw from odometry (fallback for non-Gazebo)"""
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def imu_callback(self, msg):
        """Track current chassis yaw by integrating the gyro (angular_velocity).

        URDF mounts the IMU with rpy=(0, π, π/2). The Euler-yaw extraction
        atan2(2(wz+xy), 1-2(y²+z²)) on the orientation quaternion is INVALID
        for that mounting — it yields a sign-flipped value with ~3.5×
        magnitude error (gimbal-lock-adjacent decomposition). The empirical
        fingerprint: chassis ω = -0.9 rad/s gave Euler-yaw rate of +0.255.

        Use the gyro instead. Transforming sensor-frame angular velocity to
        chassis frame with R_chassis_sensor = Rz(π/2)·Ry(π) gives:
            ω_chassis_z = -ω_sensor_z
        — a pure sign flip, mounting-independent, no gimbal lock.

        We integrate ω_chassis_z using the message's header.stamp (sim time
        when use_sim_time=true). DO NOT use wall-clock time.time() here:
        when Gazebo's RTF > 1 (fast machine, simple world), wall-clock dt
        is smaller than sim dt, so the integration under-counts and the
        90° turn loop exits early — leaving the real chassis past the
        target. The terminal would still proudly print "actual: 90°"
        because both the loop limit and the printed value share the same
        broken integration. The wall-clock for stale-detection is kept
        separate (self.imu_last_rx) so a paused sim doesn't look "live".
        """
        # Sim-time stamp from the IMU message → matches Gazebo physics.
        stamp = msg.header.stamp
        now_sim = stamp.sec + stamp.nanosec * 1e-9
        omega_chassis_z = -msg.angular_velocity.z  # sign flip from URDF mount
        if self.imu_last_sim_t is None:
            self.imu_yaw = 0.0
        else:
            dt = now_sim - self.imu_last_sim_t
            # Cap dt to guard against pause/long startup gaps causing huge jumps
            if dt <= 0.0 or dt > 0.5:
                dt = 0.0
            self.imu_yaw += omega_chassis_z * dt
            # Wrap to (-π, π] for consistency with odom yaw representation
            if self.imu_yaw > math.pi:
                self.imu_yaw -= 2 * math.pi
            elif self.imu_yaw <= -math.pi:
                self.imu_yaw += 2 * math.pi
        self.imu_last_sim_t = now_sim
        self.imu_last_rx = time.time()  # wall-clock, used only for liveness

    def imu_is_live(self, max_age_s=0.5):
        """True only if an IMU message was received within the last max_age_s.
        Guards against stale topics left over from a previous Gazebo run —
        the publisher may still exist but no fresh data is arriving."""
        if self.imu_yaw is None or self.imu_last_rx is None:
            return False
        return (time.time() - self.imu_last_rx) < max_age_s

    def get_yaw(self):
        """Get current yaw: prefer live IMU (body truth) over wheel odometry.
        Falls back to /odom when IMU is stale or absent."""
        if self.imu_is_live():
            return self.imu_yaw
        return self.odom_yaw
    
    def calculate_90_turn(self, turn_type='in_place'):
        """
        Calculate parameters needed for a 90-degree turn
        
        Returns: (angular_velocity, turn_time) tuple
        
        MATHEMATICAL DERIVATION:
        ========================
        For differential drive robot:
        - Angular velocity: ω = (v_right - v_left) / L
        
        For in-place turn (point turn):
        - v_left = -v (backward)
        - v_right = +v (forward)
        - ω = (v - (-v)) / L = 2v / L
        
        Turn duration for angle θ:
        - t = θ / ω
        
        For 90° (π/2 radians):
        - t = (π/2) / (2v/L) = π*L / (4*v)
        
        NOTE: These are the theoretical open-loop values.
        Actual execution uses closed-loop odometry feedback
        for precise 90° rotation.
        """
        v = self.turn_wheel_speed  # m/s
        L = self.wheel_separation   # m
        
        # Calculate angular velocity
        omega = (2 * v) / L  # rad/s
        
        # Calculate time for 90 degrees (π/2 radians)
        angle_rad = math.pi / 2  # 90 degrees in radians
        turn_time = angle_rad / omega  # seconds
        
        return omega, turn_time
    
    def execute_90_degree_turn(self, direction='left', turn_type='in_place'):
        """
        Execute a 90-degree turn using closed-loop odometry feedback.
        
        Uses odometry yaw to track actual rotation and stops precisely at 90°.
        This eliminates timing errors from the open-loop time.sleep() approach.
        
        Args:
            direction: 'left' or 'right'
            turn_type: 'in_place' (point turn) or 'moving' (arc with forward motion)
        """
        # Calculate theoretical turn parameters (for logging)
        theoretical_omega, theoretical_time = self.calculate_90_turn(turn_type)
        
        # Turn angular velocity: gentle so the chassis doesn't shake.
        # Higher omega + skid-steer friction = chassis oscillation in Gazebo.
        turn_omega = 0.9  # rad/s commanded peak. Skid-steer scrub eats some
                          # of this; the closed-loop decel zone below brings
                          # the chassis cleanly to 90° without overshoot.
        if direction == 'right':
            turn_omega = -turn_omega
        
        # Linear velocity during turn
        if turn_type == 'moving':
            turn_linear = 0.15  # m/s forward during arc turn
        else:
            turn_linear = 0.0
        
        target_angle = math.pi / 2  # 90 degrees
        
        # Log the calculation
        self.get_logger().info(
            f"\n{'='*70}\n"
            f"90-DEGREE TURN EXECUTION (CLOSED-LOOP)\n"
            f"{'='*70}\n"
            f"Direction: {direction.upper()}\n"
            f"Type: {turn_type.upper()}\n"
            f"\n📐 THEORETICAL (open-loop):\n"
            f"────────────────────────\n"
            f"  ω = 2v/L = 2×{self.turn_wheel_speed}/{self.wheel_separation} "
            f"= {theoretical_omega:.4f} rad/s\n"
            f"  t = (π/2)/ω = {theoretical_time:.4f} s\n"
            f"\n🤖 ACTUAL EXECUTION (closed-loop with odometry):\n"
            f"────────────────────────\n"
            f"  Command ω: {abs(turn_omega):.2f} rad/s\n"
            f"  Linear: {turn_linear:.2f} m/s\n"
            f"  Target rotation: 90° (π/2 = {target_angle:.4f} rad)\n"
            f"  Feedback: /odom yaw tracking\n"
            f"{'='*70}\n"
        )
        
        # Feedback source: prefer /imu (true body orientation) over /odom
        # (Gazebo DiffDrive integrates commanded wheel velocities and over-
        # reports rotation when the wheels scrub the floor on in-place turns).
        # Fall back to /odom if IMU is stale or in kinematic_mode (RViz only).
        # The accumulator below is wrap-aware, so multi-revolution rotations
        # are tracked correctly regardless of source.
        kinematic_mode = self.kinematic_mode_param
        use_imu = (not kinematic_mode) and self.imu_is_live()
        if kinematic_mode:
            yaw_source = 'odom (kinematic)'
        elif use_imu:
            yaw_source = 'imu (body truth)'
        else:
            yaw_source = 'odom (Gazebo physics, IMU stale)'
        self.get_logger().info(f"  Yaw source: {yaw_source}")
        # +1 for left (positive omega), -1 for right (negative omega)
        direction_sign = 1.0 if turn_omega > 0 else -1.0

        # Closed-loop: track absolute yaw displacement from start.
        # Stop AT the target — no lead. The residual measure-and-trim block
        # below corrects any small overshoot/undershoot from physics inertia
        # or loop jitter. Only the kinematic (RViz) integrator gets one
        # control-step worth of lead because it is a perfect integrator and
        # would otherwise execute one extra ω·dt step past the target.
        LOOP_DT = 0.01  # control-loop poll period (matches time.sleep below)
        if kinematic_mode:
            COAST_OFFSET = abs(turn_omega) * LOOP_DT  # ~0.5° at ω=0.9
        else:
            COAST_OFFSET = 0.0  # measure-and-trim handles the rest
        TIMEOUT_S    = 12.0
        STOP_AT      = max(target_angle - COAST_OFFSET, math.radians(2.0))

        def _read_yaw():
            return self.imu_yaw if use_imu else self.odom_yaw

        # Wrap-aware accumulated rotation. We integrate per-tick deltas
        # (each ≤ ω·dt ≪ π) so multiple revolutions are tracked correctly
        # and a misbehaving yaw source can never make the loop run away.
        #
        # IMPORTANT: capture prev_yaw BEFORE the ramp-up below. The chassis
        # already rotates several degrees during the 0.3 s ramp, and that
        # rotation must be folded into accum_disp — otherwise the loop
        # exits past the target by exactly the ramp-up yaw, the residual
        # trim sees ~0 and skips, and the final "actual" log lies.
        prev_yaw = _read_yaw()
        accum_disp = 0.0  # signed in robot's commanded direction

        # Start the turn — RAMP UP from 0 to turn_omega over 0.3 s to avoid
        # commanding a step change in angular velocity that the physics
        # engine reacts to with a violent jerk (chassis shakes/jumps).
        self.linear_velocity = turn_linear
        ramp_steps = 30  # 30 × 10 ms = 0.3 s
        for i in range(1, ramp_steps + 1):
            self.angular_velocity = turn_omega * (i / ramp_steps)
            time.sleep(0.01)

        def _step_disp():
            nonlocal prev_yaw, accum_disp
            cur = _read_yaw()
            d = cur - prev_yaw
            if d > math.pi:
                d -= 2 * math.pi
            elif d < -math.pi:
                d += 2 * math.pi
            prev_yaw = cur
            accum_disp += d * direction_sign
            return accum_disp

        # Stuck-source watchdog: if the yaw source doesn't move at all in
        # the first 1.0 s of commanded rotation, abort early instead of
        # waiting the full 12 s timeout (chassis is stuck or topic dead).
        # Also log a progress line every 0.5 s so we can SEE source drift.
        loop_start = time.time()
        warned_stuck = False
        last_log = loop_start
        while True:
            time.sleep(0.01)

            if time.time() - loop_start > TIMEOUT_S:
                self.get_logger().warn(
                    f"90° turn timed out after {TIMEOUT_S}s — aborting. "
                    f"Check /odom rate; chassis may be stuck.")
                break

            displacement = _step_disp()

            # Deceleration zone: scale commanded |omega| down linearly in
            # the last DECEL_BAND of the rotation so the chassis is already
            # moving slowly when it crosses the target. Without this the
            # in-place skid turn coasts ~9° past the target after cmd=0
            # because it carries 0.7 rad/s of angular momentum at exit.
            # In kinematic mode (RViz integrator) this is harmless and
            # also stabilises the final approach.
            DECEL_BAND = math.radians(20.0)  # last 20° of the turn
            MIN_OMEGA  = 0.20                # rad/s, slowest commanded speed
            remaining = STOP_AT - displacement
            if remaining < DECEL_BAND:
                scale = max(MIN_OMEGA / abs(turn_omega), remaining / DECEL_BAND)
                self.angular_velocity = turn_omega * scale

            now = time.time()
            if now - last_log >= 0.5:
                self.get_logger().info(
                    f"  turn progress: {math.degrees(displacement):+6.1f}° "
                    f"(target {math.degrees(STOP_AT):+.1f}°, "
                    f"odom={math.degrees(self.odom_yaw):+.1f}°, "
                    f"imu={math.degrees(self.imu_yaw):+.1f}°)"
                    if self.imu_yaw is not None else
                    f"  turn progress: {math.degrees(displacement):+6.1f}° (no IMU)")
                last_log = now

            if (not warned_stuck) and (time.time() - loop_start > 1.0):
                if abs(displacement) < math.radians(2.0):
                    self.get_logger().warn(
                        'Yaw not moving 1 s into turn — source may be dead. '
                        'Will keep trying until timeout.')
                    warned_stuck = True

            if displacement >= STOP_AT:
                break
        
        # Stop the robot. The 20 Hz publish_velocity timer republishes
        # self.angular_velocity, so we MUST set it to 0 (not just publish
        # one zero twist) or the timer will immediately overwrite us.
        # No counter-brake pulse: the previous 0.5 rad/s × 0.06 s pulse
        # rotated us BACKWARD by ~1.7°, which combined with a 3° pre-target
        # lead caused the systematic ~4° undershoot. The residual trim
        # block below now does the cleanup for both Gazebo and RViz.
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        # ---- Residual measure-and-trim ----
        # Always run, regardless of feedback source. After the chassis
        # settles, integrate any further per-tick yaw deltas into accum_disp
        # and command a short slow pulse to consume exactly the remainder.
        # Up to 3 iterations, each tightening below 0.1°. Settle delay is
        # longer in Gazebo because the physics chassis coasts after cmd=0.
        settle_s = 0.03 if kinematic_mode else 0.20
        time.sleep(settle_s)
        # Fold the post-stop coast into accum_disp before measuring.
        _step_disp()
        for _ in range(3):
            residual = target_angle - accum_disp  # +ve undershoot, -ve overshoot
            if abs(residual) < math.radians(0.1):
                break  # within 0.1° → done
            # Slow trim pulse: 0.2 rad/s, time = |residual| / 0.2, capped 0.4 s
            trim_omega = math.copysign(0.2, residual) * direction_sign
            trim_time = min(abs(residual) / 0.2, 0.4)
            self.angular_velocity = trim_omega
            trim_twist = Twist()
            trim_twist.angular.z = trim_omega
            self.cmd_vel_pub.publish(trim_twist)
            time.sleep(trim_time)
            self.angular_velocity = 0.0
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.05 if kinematic_mode else 0.15)  # let chassis settle
            _step_disp()  # accumulate the trim delta

        actual_deg = math.degrees(accum_disp)
        self.get_logger().info(
            f"90° {direction} turn completed! "
            f"(actual: {actual_deg:.2f}°, "
            f"error: {actual_deg - 90.0:+.2f}°)\n"
        )
    
    @staticmethod
    def get_key():
        """Read a single keyboard character without waiting for Enter"""
        try:
            settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            except KeyboardInterrupt:
                ch = 'q'
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return ch
        except (termios.error, OSError):
            # Fallback for non-interactive terminals (CI/testing environments)
            # Read one character without requiring terminal mode
            try:
                ch = sys.stdin.read(1)
                return ch if ch else 'q'
            except:
                return 'q'


def print_help():
    """Print control help"""
    help_text = """
╔════════════════════════════════════════════════════════════════════════════╗
║            DIFFERENTIAL DRIVE ROBOT MANUAL CONTROL                         ║
║                  X3Plus Robot Manual Teleop                                ║
╚════════════════════════════════════════════════════════════════════════════╝

MOVEMENT CONTROLS:
──────────────────
  W / w   →  Move forward (1.0 m/s)
  S / s   →  Move backward (-1.0 m/s)
  A / a   →  Rotate left (2.0 rad/s)
  D / d   →  Rotate right (-2.0 rad/s)
  SPACE   →  Emergency stop

90-DEGREE TURN COMMANDS (with formula calculation):
────────────────────────────────────────────────────
  1       →  Turn 90° left (in-place point turn)
  2       →  Turn 90° right (in-place point turn)
  3       →  Turn 90° left (while moving forward)
  4       →  Turn 90° right (while moving forward)

SYSTEM:
──────
  Q / q   →  Quit application
  H / h   →  Show this help menu

╔════════════════════════════════════════════════════════════════════════════╗
║  Each turn command will automatically calculate and display the formula    ║
║  used for the 90-degree rotation. The calculation uses the differential    ║
║  drive formula: ω = 2*v/L and t = π*L/(4*v)                               ║
╚════════════════════════════════════════════════════════════════════════════╝
"""
    print(help_text)


def main(args=None):
    rclpy.init(args=args)
    
    control_node = DifferentialDriveControl()
    
    # Spin in background so timer callbacks fire
    spin_thread = threading.Thread(target=rclpy.spin, args=(control_node,), daemon=True)
    spin_thread.start()
    
    # Print welcome message
    print_help()
    print("\n✓ Control node started! Press any key to begin...")
    control_node.get_key()
    
    try:
        while rclpy.ok():
            # Read keyboard input (non-blocking)
            key = control_node.get_key()
            
            # Movement commands
            if key in ['w', 'W']:
                control_node.linear_velocity = control_node.max_linear_velocity
                control_node.angular_velocity = 0.0
                print(f"\n→ Forward ({control_node.max_linear_velocity} m/s)      ", end='\r')
            
            elif key in ['s', 'S']:
                control_node.linear_velocity = -control_node.max_linear_velocity
                control_node.angular_velocity = 0.0
                print(f"← Backward ({control_node.max_linear_velocity} m/s)     ", end='\r')
            
            elif key in ['a', 'A']:
                control_node.linear_velocity = 0.0
                control_node.angular_velocity = control_node.max_angular_velocity
                print(f"↺ Rotate left ({control_node.max_angular_velocity} rad/s)   ", end='\r')
            
            elif key in ['d', 'D']:
                control_node.linear_velocity = 0.0
                control_node.angular_velocity = -control_node.max_angular_velocity
                print(f"↻ Rotate right ({control_node.max_angular_velocity} rad/s)  ", end='\r')
            
            elif key == ' ':
                control_node.linear_velocity = 0.0
                control_node.angular_velocity = 0.0
                print("\n⏹ STOP - All motors stopped                          ", end='\r')
            
            # 90-degree turn commands
            elif key == '1':
                print("\n")
                control_node.execute_90_degree_turn(direction='left', turn_type='in_place')
            
            elif key == '2':
                print("\n")
                control_node.execute_90_degree_turn(direction='right', turn_type='in_place')
            
            elif key == '3':
                print("\n")
                control_node.execute_90_degree_turn(direction='left', turn_type='moving')
            
            elif key == '4':
                print("\n")
                control_node.execute_90_degree_turn(direction='right', turn_type='moving')
            
            # System commands
            elif key in ['q', 'Q']:
                print("\n\n🛑 Shutting down...\n")
                break
            
            elif key in ['h', 'H']:
                print_help()
            
            # Small delay to prevent CPU spinning
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user!")
    
    finally:
        # Ensure robot stops
        control_node.linear_velocity = 0.0
        control_node.angular_velocity = 0.0
        time.sleep(0.1)
        
        # Cleanup
        control_node.destroy_node()
        rclpy.try_shutdown()
        print("✓ Shutdown complete.\n")


if __name__ == '__main__':
    main()
