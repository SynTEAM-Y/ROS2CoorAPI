#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

class Avoid(Node):
    def __init__(self):
        super().__init__('avoid_reflex')

        # Declares movement speeds and distance thresholds
        self.forward = self.declare_parameter('forward', 0.22).value
        self.turn    = self.declare_parameter('turn', 0.9).value
        self.stop_range = self.declare_parameter('stop_range', 0.55).value
        self.hard_stop  = self.declare_parameter('hard_stop', 0.25).value
        self.hard_stop_enable = bool(self.declare_parameter('hard_stop_enable', True).value)
        self.min_valid_ratio  = float(self.declare_parameter('min_valid_ratio', 0.05).value)  # 5%

        # Declares field of view angles
        self.fov_left_deg  = float(self.declare_parameter('fov_left_deg',  90.0).value)
        self.fov_right_deg = float(self.declare_parameter('fov_right_deg', -90.0).value)
        self.front_half_deg = float(self.declare_parameter('front_half_deg', 30.0).value)

        # Declares how obstacle distances are summarized and smoothed
        self.sector_method = str(self.declare_parameter('sector_method', 'median').value) # median|min|mean
        self.smooth_alpha  = float(self.declare_parameter('smooth_alpha', 0.4).value)

        # Declares anti-wiggle settings (minimum times for turns/forward motion)
        self.min_turn_time = float(self.declare_parameter('min_turn_time', 0.5).value)
        self.min_fwd_time  = float(self.declare_parameter('min_fwd_time',  0.5).value)
        self.turn_bias_margin = float(self.declare_parameter('turn_bias_margin', 0.05).value)

        # Declares toggles for verbose logging and actual robot movement
        self.verbose    = bool(self.declare_parameter('verbose', False).value)
        self.move_robot = bool(self.declare_parameter('move_robot', True).value)
        if not self.move_robot:
            self.verbose = True

        # Creates publisher for /cmd_vel and subscriber for /scan
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, QoSPresetProfiles.SENSOR_DATA.value)

        # Sets initial state variables
        self.last_cmd = 'stop'
        self.last_switch_t = time.monotonic()
        self.smooth_vals = {'front': None, 'left': None, 'right': None}
        self.last_turn_side = None

        # Prints initial settings if verbose is enabled
        if self.verbose:
            self.get_logger().info(
                f"avoid_reflex: fwd={self.forward} turn={self.turn} stop={self.stop_range} "
                f"hard={self.hard_stop} enable_hard={self.hard_stop_enable} "
                f"min_valid_ratio={self.min_valid_ratio} FOV=[{self.fov_right_deg}°, {self.fov_left_deg}°]"
            )

    def _mask_front_fov(self, msg: LaserScan):
        # Creates a boolean mask for scan readings inside the configured FOV
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ang_deg = (np.degrees(angles) + 180.0) % 360.0 - 180.0
        m = (ang_deg >= self.fov_right_deg) & (ang_deg <= self.fov_left_deg)
        return m, angles, ang_deg

    def _stat(self, arr):
        # Returns a summary statistic of valid distances (median by default)
        arr = arr[np.isfinite(arr)]
        if arr.size == 0: return math.inf
        if self.sector_method == 'min':  return float(np.min(arr))
        if self.sector_method == 'mean': return float(np.mean(arr))
        return float(np.median(arr))

    def _ewma(self, key, val):
        # Applies exponential smoothing to reduce noise in distance values
        if not math.isfinite(val): return val
        prev = self.smooth_vals[key]
        if prev is None or not math.isfinite(prev):
            self.smooth_vals[key] = val
        else:
            a = clamp(self.smooth_alpha, 0.0, 1.0)
            self.smooth_vals[key] = a*val + (1-a)*prev
        return self.smooth_vals[key]

    def _min_hold(self, want_cmd):
        # Ensures the robot holds a command for a minimum time to avoid jitter
        elapsed = time.monotonic() - self.last_switch_t
        need = self.min_turn_time if 'turn' in self.last_cmd else self.min_fwd_time
        return (self.last_cmd == want_cmd) or (elapsed >= need)

    def cb(self, msg: LaserScan):
        mask_fov, angles, ang_deg = self._mask_front_fov(msg)

        # Sanitizes ranges: replaces invalid values with infinity
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        invalid = ~np.isfinite(ranges) | (ranges < msg.range_min) | (ranges > msg.range_max)
        ranges[invalid] = np.inf

        # Computes fraction of valid readings in the FOV
        fov_count = int(np.count_nonzero(mask_fov))
        valid_fov_mask = mask_fov & (~invalid)
        valid_count = int(np.count_nonzero(valid_fov_mask))
        valid_ratio = (valid_count / fov_count) if fov_count > 0 else 0.0

        # Applies hard stop if something is dangerously close
        if self.hard_stop_enable and valid_ratio >= self.min_valid_ratio:
            hard_val = np.min(ranges[valid_fov_mask]) if valid_count > 0 else math.inf
            if hard_val < self.hard_stop:
                self._publish('stop', Twist(), note=f"HARD STOP r={hard_val:.2f} (valid_ratio={valid_ratio:.2f})")
                return

        # Splits scan into left, right, and front sectors
        sec = self.front_half_deg
        left_mask  = valid_fov_mask & (ang_deg >= sec) & (ang_deg <= min(self.fov_left_deg, 120.0))
        right_mask = valid_fov_mask & (ang_deg <= -sec) & (ang_deg >= max(self.fov_right_deg, -120.0))
        front_mask = valid_fov_mask & (ang_deg >= -sec) & (ang_deg <= +sec)

        # Computes sector statistics and smooths them
        left  = self._stat(ranges[left_mask])
        right = self._stat(ranges[right_mask])
        front = self._stat(ranges[front_mask])
        left  = self._ewma('left', left)
        right = self._ewma('right', right)
        front = self._ewma('front', front)

        # Decides movement command
        cmd = Twist(); want = 'forward'
        note = f"front={front:.2f} left={left:.2f} right={right:.2f} (valid={valid_ratio:.2f})"

        if front < self.stop_range:
            # Chooses turn direction, with bias to prevent indecision
            if self.last_turn_side == 'left' and (right - left) < self.turn_bias_margin:
                turn_side = 'left'
            elif self.last_turn_side == 'right' and (left - right) < self.turn_bias_margin:
                turn_side = 'right'
            else:
                turn_side = 'left' if left > right else 'right'
            want = f"turn_{turn_side}"
            cmd.angular.z = +self.turn if turn_side == 'left' else -self.turn
        else:
            # Drives forward, adding slight bias toward clearer side
            cmd.linear.x = self.forward
            if left < right:   cmd.angular.z = -0.2
            elif right < left: cmd.angular.z = +0.2

        # Enforces minimum hold time to avoid wiggle
        if not self._min_hold(want):
            want = self.last_cmd
            if 'turn_left' in want:   cmd.angular.z = +self.turn
            elif 'turn_right' in want:cmd.angular.z = -self.turn
            elif 'forward' in want:   cmd.linear.x = self.forward
            else:                     cmd = Twist()
            note += " | hold"

        if want.startswith('turn_'):
            self.last_turn_side = 'left' if 'left' in want else 'right'

        self._publish(want, cmd, note)

    def _publish(self, want_cmd, cmd: Twist, note=""):
        # Publishes the command and logs details if verbose is enabled
        if self.verbose:
            self.get_logger().info(f"{want_cmd.upper():>10s} | {note} | move={self.move_robot}")
        if self.move_robot:
            self.pub.publish(cmd)
        else:
            self.pub.publish(Twist())
        if want_cmd != self.last_cmd:
            self.last_cmd = want_cmd
            self.last_switch_t = time.monotonic()

def main():
    rclpy.init()
    node = Avoid()
    try:
        rclpy.spin(node)  # Keeps the node alive until interrupted
    except KeyboardInterrupt:
        pass
    finally:
        # Stops the robot on exit
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
