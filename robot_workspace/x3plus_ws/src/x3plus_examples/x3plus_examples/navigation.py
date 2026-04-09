#!/usr/bin/env python3
import math, time, os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

DEG = math.pi/180.0

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

class Explorer(Node):
    """
    Simple room explorer:
      - Goes straight until an obstacle is within front_clear.
      - Turns (fixed time) toward the side with more clearance.
      - Commits to a forward burst after turning to traverse new space.
    States: FORWARD, TURN_LEFT, TURN_RIGHT, FORWARD_BURST, STOP
    """
    def __init__(self):
        super().__init__('navigation')

        # Declares navigation parameters (speeds, clearances, durations)
        self.forward = float(self.declare_parameter('forward', 0.22).value)       # m/s
        self.turn    = float(self.declare_parameter('turn', 1.0).value)           # rad/s
        self.front_clear = float(self.declare_parameter('front_clear', 0.55).value)  # m
        self.side_buffer = float(self.declare_parameter('side_buffer', 0.35).value)  # m
        self.turn_time   = float(self.declare_parameter('turn_time', 0.9).value)     # s
        self.forward_burst_time = float(self.declare_parameter('forward_burst_time', 1.2).value)  # s

        # Declares field of view limits (front-only processing)
        self.fov_left_deg  = float(self.declare_parameter('fov_left_deg',  90.0).value)
        self.fov_right_deg = float(self.declare_parameter('fov_right_deg', -90.0).value)
        self.front_half_deg = float(self.declare_parameter('front_half_deg', 20.0).value)

        # Declares safety options (hard stop with validity check)
        self.hard_stop_enable = bool(self.declare_parameter('hard_stop_enable', True).value)
        self.hard_stop = float(self.declare_parameter('hard_stop', 0.22).value)
        self.min_valid_ratio = float(self.declare_parameter('min_valid_ratio', 0.05).value)

        # Declares smoothing gain for gentle steering bias while moving
        self.bias_gain = float(self.declare_parameter('bias_gain', 0.25).value)   # rad/s

        # Declares verbosity and motion toggles
        self.verbose    = bool(self.declare_parameter('verbose', True).value)
        self.move_robot = bool(self.declare_parameter('move_robot', True).value)
        if not self.move_robot: self.verbose = True

        # Determine cmd_vel topic from robot param, env, or default
        robot_param = self.declare_parameter('robot', '').value
        if isinstance(robot_param, str):
            robot_param = robot_param.strip()
        else:
            robot_param = str(robot_param).strip() if robot_param is not None else ''
        robot_env = os.getenv('ROBOT', '').strip()

        if robot_param:
            cmd_vel_topic = f'/{robot_param}/cmd_vel'
        elif robot_env:
            cmd_vel_topic = f'/{robot_env}/cmd_vel'
        else:
            cmd_vel_topic = '/cmd_vel'

        # Creates publishers/subscribers for motion and LiDAR
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, QoSPresetProfiles.SENSOR_DATA.value)

        # Initializes the finite-state machine
        self.state = 'FORWARD'
        self.state_end = 0.0

        # Logs initial configuration if verbose
        if self.verbose:
            self.get_logger().info(
                f"room_explorer: fwd={self.forward} turn={self.turn} "
                f"front_clear={self.front_clear} turn_time={self.turn_time}s "
                f"burst={self.forward_burst_time}s FOV=[{self.fov_right_deg}°, {self.fov_left_deg}°] "
                f"hard_stop={self.hard_stop} enable={self.hard_stop_enable}"
                f"cmd_vel_topic='{cmd_vel_topic}'"
            )

    # ---- helpers ----
    def _mask_front_fov(self, msg: LaserScan):
        # Builds a mask over scan angles that fall within the configured FOV
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ang_deg = (np.degrees(angles) + 180.0) % 360.0 - 180.0  # [-180,180)
        m = (ang_deg >= self.fov_right_deg) & (ang_deg <= self.fov_left_deg)
        return m, angles, ang_deg

    def _sanitize_ranges(self, msg: LaserScan):
        # Replaces invalid, below-min, or above-max ranges with infinity
        r = np.asarray(msg.ranges, dtype=np.float32)
        invalid = ~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)
        r[invalid] = np.inf
        return r, invalid

    def _sector_stat(self, ranges, mask, method='median'):
        # Summarizes distances within a sector using a chosen statistic
        sel = ranges[mask]
        sel = sel[np.isfinite(sel)]
        if sel.size == 0: return math.inf
        if method == 'min':  return float(np.min(sel))
        if method == 'mean': return float(np.mean(sel))
        return float(np.median(sel))

    def _publish(self, cmd: Twist, label: str, note: str = ""):
        # Publishes motion command and logs a short status line
        if self.verbose:
            self.get_logger().info(f"{label:<12s} {note} | move={self.move_robot}")
        if self.move_robot:
            self.pub.publish(cmd)
        else:
            self.pub.publish(Twist())

    # ---- main callback ----
    def cb(self, msg: LaserScan):
        now = time.monotonic()

        # Computes FOV masks and cleans the range array
        fov_mask, angles, ang_deg = self._mask_front_fov(msg)
        ranges, invalid = self._sanitize_ranges(msg)

        # Computes valid sample ratio inside the FOV (for guarded hard-stop)
        fov_count = int(np.count_nonzero(fov_mask))
        valid_fov_mask = fov_mask & ~invalid
        valid_count = int(np.count_nonzero(valid_fov_mask))
        valid_ratio = (valid_count / fov_count) if fov_count > 0 else 0.0

        # Builds sector masks for front / left / right
        sec = self.front_half_deg
        front_mask = valid_fov_mask & (ang_deg >= -sec) & (ang_deg <= +sec)
        left_mask  = valid_fov_mask & (ang_deg >  +sec)
        right_mask = valid_fov_mask & (ang_deg <  -sec)

        # Computes per-sector distance summaries
        front = self._sector_stat(ranges, front_mask, method='median')
        left  = self._sector_stat(ranges, left_mask,  method='median')
        right = self._sector_stat(ranges, right_mask, method='median')

        # Applies hard stop if something is dangerously close and data is trustworthy
        if self.hard_stop_enable and valid_ratio >= self.min_valid_ratio:
            nearest = np.min(ranges[valid_fov_mask]) if valid_count > 0 else math.inf
            if nearest < self.hard_stop:
                self.state = 'STOP'
                self.state_end = now + 0.2
                self._publish(Twist(), 'STOP', f"HARD r={nearest:.2f}")
                return

        cmd = Twist()

        # Runs the finite-state machine to choose the next motion
        if self.state == 'FORWARD':
            # Drives straight, adding a small bias away from the closer side
            cmd.linear.x = self.forward
            if left < self.side_buffer and right >= self.side_buffer:
                cmd.angular.z = -self.bias_gain
            elif right < self.side_buffer and left >= self.side_buffer:
                cmd.angular.z = +self.bias_gain

            # Switches to a timed turn if the front gets too close
            if front < self.front_clear:
                turn_side = 'LEFT' if left > right else 'RIGHT'
                self.state = f'TURN_{turn_side}'
                self.state_end = now + self.turn_time

        if self.state.startswith('TURN_'):
            # Turns in place for a fixed duration
            cmd = Twist()
            cmd.angular.z = +self.turn if self.state == 'TURN_LEFT' else -self.turn
            if now >= self.state_end:
                self.state = 'FORWARD_BURST'
                self.state_end = now + self.forward_burst_time

        elif self.state == 'FORWARD_BURST':
            # Pushes forward after turning to enter new space
            cmd.linear.x = self.forward
            if now >= self.state_end:
                self.state = 'FORWARD'

        elif self.state == 'STOP':
            # Waits briefly, then attempts a turn toward the more open side
            if now >= self.state_end:
                turn_side = 'LEFT' if left > right else 'RIGHT'
                self.state = f'TURN_{turn_side}'
                self.state_end = now + self.turn_time
                cmd.angular.z = +self.turn if self.state == 'TURN_LEFT' else -self.turn

        # Logs a short status line with sector distances and validity
        label = self.state
        note  = f"front={front:.2f} left={left:.2f} right={right:.2f} valid={valid_ratio:.2f}"
        self._publish(cmd, label, note)

def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stops the robot on shutdown and cleans up
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
