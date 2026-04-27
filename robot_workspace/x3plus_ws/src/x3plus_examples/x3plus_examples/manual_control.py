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
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        
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
        """Track current chassis yaw from IMU.

        The IMU is mounted with URDF rpy=(0, π, π/2), i.e. flipped upside-down
        (pitch=π) and rotated 90°. The pitch=π means the sensor's +Z axis
        points DOWN, so a clockwise chassis rotation registers as counter-
        clockwise in the sensor body frame — yaw sign is inverted.

        Empirical verification: at rest both raw and chassis yaw are ~0, so
        the offset is zero; only the sign needs flipping.
        """
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = -math.atan2(siny_cosp, cosy_cosp)
    
    def get_yaw(self):
        """Get current yaw: prefer IMU (body truth) over wheel odometry"""
        if self.imu_yaw is not None:
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
        
        # Record starting yaw (from IMU if available, else odom)
        start_yaw = self.get_yaw()
        yaw_source = 'IMU' if self.imu_yaw is not None else 'odom'
        self.get_logger().info(f"  Yaw source: {yaw_source}")
        # +1 for left (positive omega), -1 for right (negative omega)
        direction_sign = 1.0 if turn_omega > 0 else -1.0
        
        # Start the turn — RAMP UP from 0 to turn_omega over 0.3 s to avoid
        # commanding a step change in angular velocity that the physics
        # engine reacts to with a violent jerk (chassis shakes/jumps).
        self.linear_velocity = turn_linear
        ramp_steps = 30  # 30 × 10 ms = 0.3 s
        for i in range(1, ramp_steps + 1):
            self.angular_velocity = turn_omega * (i / ramp_steps)
            time.sleep(0.01)
        
        # Closed-loop: track absolute yaw displacement from start.
        # Uses IMU (actual body orientation) instead of wheel odometry
        # for accurate skid-steer turns.
        #
        # Empirical (Gazebo Fortress, 4-wheel skid-steer, omega=0.9 rad/s):
        # the chassis coasts a few degrees after we publish zero. Tune
        # COAST_OFFSET if landing is consistently off (decrease if
        # overshooting, increase if undershooting).
        COAST_OFFSET = math.radians(0.5)
        TIMEOUT_S    = 12.0
        STOP_AT      = max(target_angle - COAST_OFFSET, math.radians(2.0))

        def _signed_disp():
            d = self.get_yaw() - start_yaw
            if d > math.pi:
                d -= 2 * math.pi
            elif d < -math.pi:
                d += 2 * math.pi
            return d * direction_sign

        loop_start = time.time()
        while True:
            time.sleep(0.01)

            if time.time() - loop_start > TIMEOUT_S:
                self.get_logger().warn(
                    f"90° turn timed out after {TIMEOUT_S}s — aborting. "
                    f"Check /imu and /odom rates; chassis may be stuck in Gazebo.")
                break

            displacement = _signed_disp()
            if displacement >= STOP_AT:
                break
        
        # Stop the robot — brief counter-omega brake pulse to kill yaw
        # inertia, then zero. Without the pulse the chassis coasts
        # several degrees past target after we publish zero.
        # IMPORTANT: the 20 Hz publish_velocity timer republishes
        # self.angular_velocity, so we must set it (not just publish once)
        # otherwise the timer immediately overwrites the brake.
        self.linear_velocity = 0.0
        self.angular_velocity = -direction_sign * 0.4
        brake_twist = Twist()
        brake_twist.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(brake_twist)
        time.sleep(0.05)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        # Immediately publish stop to minimize overshoot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        final_delta = self.get_yaw() - start_yaw
        if final_delta > math.pi:
            final_delta -= 2 * math.pi
        elif final_delta < -math.pi:
            final_delta += 2 * math.pi
        actual_deg = abs(math.degrees(final_delta))
        self.get_logger().info(
            f"90° {direction} turn completed! "
            f"(actual: {actual_deg:.1f}°, "
            f"error: {actual_deg - 90.0:+.1f}°)\n"
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
