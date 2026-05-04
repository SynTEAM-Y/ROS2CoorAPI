#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNav(Node):
    """Reactive navigation node that drives a robot forward while avoiding
    obstacles using LiDAR (LaserScan) data. Publishes velocity commands
    based on distance readings in front and side sectors.
    """

    def __init__(self):
        super().__init__('reactive_nav')

        # --- Core configurable parameters ---
        self.declare_parameter('move_robot', True)        # Whether to publish /cmd_vel or just simulate
        self.declare_parameter('forward_speed', 0.22)     # Default forward linear velocity (m/s)
        self.declare_parameter('turn_speed', 1.0)         # Default angular velocity (rad/s)
        self.declare_parameter('front_clear', 0.55)       # Distance threshold for slowing/turning (m)
        self.declare_parameter('hard_stop', 0.22)         # Distance threshold for immediate stop (m)
        self.declare_parameter('min_valid_ratio', 0.05)   # Minimum ratio of valid LiDAR points for reliable data
        self.declare_parameter('left_fov_deg',  90.0)     # Left sector FOV limit (degrees)
        self.declare_parameter('right_fov_deg', -90.0)    # Right sector FOV limit (degrees)
        self.declare_parameter('front_half_deg', 20.0)    # Half-width of the front-facing sector (degrees)
        self.declare_parameter('robot', '')               # Optional robot name for namespacing topics

        # --- Load parameter values ---
        self.move_robot  = bool(self.get_parameter('move_robot').value)
        self.v_fwd       = float(self.get_parameter('forward_speed').value)
        self.v_turn      = float(self.get_parameter('turn_speed').value)
        self.front_clear = float(self.get_parameter('front_clear').value)
        self.hard_stop   = float(self.get_parameter('hard_stop').value)
        self.min_valid   = float(self.get_parameter('min_valid_ratio').value)
        self.left_fov    = float(self.get_parameter('left_fov_deg').value)
        self.right_fov   = float(self.get_parameter('right_fov_deg').value)
        self.front_half  = float(self.get_parameter('front_half_deg').value)

        # --- Resolve topics from robot param, $ROBOT, or defaults ---
        robot_param = self.get_parameter('robot').value
        if isinstance(robot_param, str):
            robot_param = robot_param.strip()
        else:
            robot_param = str(robot_param).strip() if robot_param is not None else ''
        robot_env = os.getenv('ROBOT', '').strip()

        if robot_param:
            cmd_topic = f'/{robot_param}/cmd_vel'
            scan_topic = f'/{robot_param}/scan'
        elif robot_env:
            cmd_topic = f'/{robot_env}/cmd_vel'
            scan_topic = f'/{robot_env}/scan'
        else:
            cmd_topic = '/cmd_vel'
            scan_topic = '/scan'

        # --- ROS publishers and subscribers ---
        self.cmdpub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)

        # --- Internal state ---
        self.scan = None
        self.create_timer(0.05, self.on_timer)  # 20 Hz control loop
        self._dbg_k = 0  # Counter for throttled debug logging

        self.get_logger().info(
            f"ReactiveNav started | move_robot={self.move_robot} "
            f"cmd_topic='{cmd_topic}' scan_topic='{scan_topic}'"
        )

    def on_scan(self, msg: LaserScan):
        """Callback that stores the most recent LiDAR scan message."""
        self.scan = msg

    def sector(self, msg: LaserScan, deg_min, deg_max):
        """Extracts a slice of the LiDAR scan within a given angular range (in degrees),
        filters valid distances, and returns the minimum distance and ratio of valid readings.
        """
        a0 = math.degrees(msg.angle_min)
        step = math.degrees(msg.angle_increment)
        n = len(msg.ranges)

        # Convert requested degree bounds into index range
        i0 = int(round((deg_min - a0) / step))
        i1 = int(round((deg_max - a0) / step))
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))
        if i0 > i1:
            i0, i1 = i1, i0

        # Filter out invalid or infinite readings
        vals = [r for r in msg.ranges[i0:i1+1] if math.isfinite(r) and r > 0.0]
        valid_ratio = len(vals) / max(1, (i1 - i0 + 1))
        mind = min(vals) if vals else float('inf')
        return mind, valid_ratio

    def on_timer(self):
        """Main control loop: processes LiDAR data and decides movement commands."""
        if self.scan is None:
            return

        msg = self.scan

        # --- Front sector (centered around 0°) ---
        front_min, front_valid = self.sector(msg, -self.front_half, self.front_half)

        # --- Swapped logic note ---
        # The LiDAR reports positive angles to the RIGHT (clockwise).
        # Therefore, we intentionally flip the left/right FOV when computing sectors.
        right_min, _ = self.sector(msg, 0.0, self.left_fov)
        left_min,  _ = self.sector(msg, self.right_fov, 0.0)
        # --------------------------------------------------------------

        twist = Twist()

        # --- Decision logic based on obstacle proximity ---
        if front_valid < self.min_valid:
            # Insufficient data → stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif front_min < self.hard_stop:
            # Immediate obstacle → stop and rotate away from the closest side
            turn_dir = -1.0 if left_min < right_min else 1.0
            twist.linear.x = 0.0
            twist.angular.z = turn_dir * self.v_turn
        elif front_min < self.front_clear:
            # Obstacle approaching → slow down and turn slightly
            turn_dir = -1.0 if left_min < right_min else 1.0
            twist.linear.x = 0.10
            twist.angular.z = turn_dir * (0.6 * self.v_turn)
        else:
            # Path clear → move straight forward
            twist.linear.x = self.v_fwd
            twist.angular.z = 0.0

        # --- Periodic debug logging (every 10th iteration) ---
        self._dbg_k += 1
        if self._dbg_k % 10 == 0:
            self.get_logger().info(
                f"front={front_min:.2f} left={left_min:.2f} right={right_min:.2f} "
                f"cmd=({twist.linear.x:.2f},{twist.angular.z:.2f})"
            )

        # --- Command publication ---
        if self.move_robot:
            self.cmdpub.publish(twist)

def main():
    """Initialize and run the ReactiveNav node."""
    rclpy.init()
    rclpy.spin(ReactiveNav())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
