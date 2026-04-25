#!/usr/bin/env python3
"""
X3Plus Arm Controller — Keyboard Teleop for 5-DOF Arm + Gripper

Publishes joint positions to /joint_states so robot_state_publisher
can visualize the arm in both RViz and Gazebo simulation modes.

Joint mapping:
  arm_joint1 : base rotation          [-π/2,  π/2]
  arm_joint2 : shoulder               [-π/2,  π/2]
  arm_joint3 : elbow upper            [-π/2,  π/2]
  arm_joint4 : elbow lower            [-π/2,  π/2]
  arm_joint5 : wrist                  [-π/2,  π  ]
  grip_joint : gripper (open=0, closed=-π/2) [-π/2, 0]

Usage:
    ros2 run x3plus_examples arm_controller

Keyboard Controls:
    1-5  — Select arm joint 1-5
    6    — Select gripper joint
    W/w  — Increase selected joint angle (+0.1 rad)
    S/s  — Decrease selected joint angle (-0.1 rad)
    O/o  — Open gripper  (grip_joint → 0.0)
    C/c  — Close gripper (grip_joint → -π/2)
    A/a  — Home pose     (all joints → 0)
    Z/z  — Init pose     (natural ready position)
    B/b  — Down pose     (arm folded down)
    P/p  — Execute pick-and-place sequence (10 steps)
    H/h  — Show this help menu
    Q/q  — Quit
"""

import math
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# Joint definitions
# ---------------------------------------------------------------------------
ARM_JOINTS = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
GRIP_JOINT = 'grip_joint'
ALL_JOINTS = ARM_JOINTS + [GRIP_JOINT]

JOINT_LIMITS = {
    'arm_joint1': (-math.pi / 2, math.pi / 2),
    'arm_joint2': (-math.pi / 2, math.pi / 2),
    'arm_joint3': (-math.pi / 2, math.pi / 2),
    'arm_joint4': (-math.pi / 2, math.pi / 2),
    'arm_joint5': (-math.pi / 2, math.pi),
    # Match URDF exactly: lower=-1.54, upper=0.0
    'grip_joint': (-1.54, 0.0),
}

GRIP_CLOSED = JOINT_LIMITS['grip_joint'][0]

# Preset poses  {joint_name: angle_rad}
HOME_POSE = {j: 0.0 for j in ALL_JOINTS}

INIT_POSE = {
    'arm_joint1': 0.0,
    'arm_joint2': math.pi / 4,   #  45° – shoulder raised
    'arm_joint3': -math.pi / 4,  # -45° – elbow bent
    'arm_joint4': 0.0,
    'arm_joint5': 0.0,
    'grip_joint': 0.0,           #  gripper open
}

DOWN_POSE = {
    'arm_joint1': 0.0,
    'arm_joint2': -math.pi / 2,  # shoulder fully down
    'arm_joint3': math.pi / 4,
    'arm_joint4': math.pi / 4,
    'arm_joint5': 0.0,
    'grip_joint': -math.pi / 4,
}

# Pick-and-place waypoints (list of pose dicts, 10 steps)
PICK_PLACE_SEQUENCE = [
    # Step 1: home
    HOME_POSE,
    # Step 2: open gripper
    {**HOME_POSE, 'grip_joint': 0.0},
    # Step 3: approach above object
    {'arm_joint1': 0.0, 'arm_joint2': math.pi / 4, 'arm_joint3': -math.pi / 6,
     'arm_joint4': 0.0, 'arm_joint5': 0.0, 'grip_joint': 0.0},
    # Step 4: lower to pick position
    {'arm_joint1': 0.0, 'arm_joint2': math.pi / 3, 'arm_joint3': -math.pi / 3,
     'arm_joint4': -math.pi / 6, 'arm_joint5': 0.0, 'grip_joint': 0.0},
    # Step 5: close gripper
    {'arm_joint1': 0.0, 'arm_joint2': math.pi / 3, 'arm_joint3': -math.pi / 3,
     'arm_joint4': -math.pi / 6, 'arm_joint5': 0.0, 'grip_joint': GRIP_CLOSED},
    # Step 6: lift object
    {'arm_joint1': 0.0, 'arm_joint2': math.pi / 4, 'arm_joint3': -math.pi / 6,
     'arm_joint4': 0.0, 'arm_joint5': 0.0, 'grip_joint': GRIP_CLOSED},
    # Step 7: rotate to place position
    {'arm_joint1': math.pi / 3, 'arm_joint2': math.pi / 4, 'arm_joint3': -math.pi / 6,
     'arm_joint4': 0.0, 'arm_joint5': 0.0, 'grip_joint': GRIP_CLOSED},
    # Step 8: lower to place height
    {'arm_joint1': math.pi / 3, 'arm_joint2': math.pi / 3, 'arm_joint3': -math.pi / 3,
     'arm_joint4': -math.pi / 6, 'arm_joint5': 0.0, 'grip_joint': GRIP_CLOSED},
    # Step 9: open gripper (release)
    {'arm_joint1': math.pi / 3, 'arm_joint2': math.pi / 3, 'arm_joint3': -math.pi / 3,
     'arm_joint4': -math.pi / 6, 'arm_joint5': 0.0, 'grip_joint': 0.0},
    # Step 10: return to home
    HOME_POSE,
]

STEP_DURATION = 1.0  # seconds per pick-and-place step
INTERPOLATION_STEPS = 20  # smoother motion between waypoints

# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------
HELP_MENU = """
╔══════════════════════════════════════════════════════════════╗
║                  X3PLUS ARM CONTROLLER                       ║
╚══════════════════════════════════════════════════════════════╝

JOINT SELECTION:
  1 → arm_joint1 (base rotation)
  2 → arm_joint2 (shoulder)
  3 → arm_joint3 (elbow upper)
  4 → arm_joint4 (elbow lower)
  5 → arm_joint5 (wrist)
  6 → grip_joint (gripper)

JOINT ADJUSTMENT (selected joint):
  W / w  →  + 0.1 rad
  S / s  →  - 0.1 rad

GRIPPER SHORTCUTS:
  O / o  →  Open  gripper (0.0 rad)
  C / c  →  Close gripper (-π/2 rad)

PRESET POSES:
  A / a  →  Home  (all joints 0)
  Z / z  →  Init  (natural ready position)
  B / b  →  Down  (arm folded down)

SEQUENCE:
  P / p  →  Execute pick-and-place (10 steps × 1 s each)

SYSTEM:
  H / h  →  Show this help menu
  Q / q  →  Quit
"""


class ArmController(Node):
    """Keyboard-controlled arm node — publishes to /joint_states."""

    STEP_SIZE = 0.1  # radians per key press

    def __init__(self):
        super().__init__('arm_controller')

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # Current joint angles
        self.positions = {j: 0.0 for j in ALL_JOINTS}

        # Currently selected joint (0-indexed into ALL_JOINTS)
        self.selected = 0  # arm_joint1 by default

        self._running = True
        self._pick_place_busy = False

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self._publish)

        self.get_logger().info('\n' + HELP_MENU)
        self._print_status()

    # ------------------------------------------------------------------
    # Publisher
    # ------------------------------------------------------------------
    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [self.positions[j] for j in ALL_JOINTS]
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------
    def _print_status(self):
        sel_name = ALL_JOINTS[self.selected]
        lines = [f'\n  Selected joint: [{self.selected + 1}] {sel_name}']
        for i, j in enumerate(ALL_JOINTS):
            marker = '►' if i == self.selected else ' '
            lo, hi = JOINT_LIMITS[j]
            lines.append(
                f'  {marker} {j:14s} {self.positions[j]:+.3f} rad'
                f'  [{lo:+.3f}, {hi:+.3f}]'
            )
        print('\n'.join(lines) + '\n')

    # ------------------------------------------------------------------
    # Pose application
    # ------------------------------------------------------------------
    def _apply_pose(self, pose: dict, label: str):
        for j, val in pose.items():
            self.positions[j] = self._clamp(j, val)
        self.get_logger().info(f'Pose applied: {label}')
        self._print_status()

    def _clamp(self, joint: str, value: float) -> float:
        lo, hi = JOINT_LIMITS[joint]
        return max(lo, min(hi, value))

    # ------------------------------------------------------------------
    # Pick-and-place sequence (runs in background thread)
    # ------------------------------------------------------------------
    def _run_pick_place(self):
        self._pick_place_busy = True
        self.get_logger().info(
            f'Starting pick-and-place sequence ({len(PICK_PLACE_SEQUENCE)} steps) …'
        )
        for step_idx, pose in enumerate(PICK_PLACE_SEQUENCE):
            if not self._running:
                break
            self.get_logger().info(
                f'  Step {step_idx + 1}/{len(PICK_PLACE_SEQUENCE)}'
            )
            # Interpolate from current pose to target to avoid abrupt jumps.
            start_pose = {j: self.positions[j] for j in ALL_JOINTS}
            target_pose = {
                j: self._clamp(j, pose.get(j, self.positions[j]))
                for j in ALL_JOINTS
            }

            for i in range(1, INTERPOLATION_STEPS + 1):
                if not self._running:
                    break
                alpha = i / INTERPOLATION_STEPS
                for j in ALL_JOINTS:
                    self.positions[j] = (
                        start_pose[j] + alpha * (target_pose[j] - start_pose[j])
                    )
                time.sleep(STEP_DURATION / INTERPOLATION_STEPS)

            self._print_status()
        self.get_logger().info('Pick-and-place sequence complete.')
        self._pick_place_busy = False

    # ------------------------------------------------------------------
    # Key handling
    # ------------------------------------------------------------------
    def handle_key(self, key: str):
        if self._pick_place_busy and key not in ('q', 'Q'):
            print('  [Pick-and-place running — press Q to quit]')
            return

        if key in '123456':
            self.selected = int(key) - 1
            print(f'\n  → Selected: [{key}] {ALL_JOINTS[self.selected]}')
            self._print_status()

        elif key in ('w', 'W'):
            j = ALL_JOINTS[self.selected]
            self.positions[j] = self._clamp(j, self.positions[j] + self.STEP_SIZE)
            print(f'  ↑ {j}  {self.positions[j]:+.3f} rad')
            self._print_status()

        elif key in ('s', 'S'):
            j = ALL_JOINTS[self.selected]
            self.positions[j] = self._clamp(j, self.positions[j] - self.STEP_SIZE)
            print(f'  ↓ {j}  {self.positions[j]:+.3f} rad')
            self._print_status()

        elif key in ('o', 'O'):
            self.positions['grip_joint'] = 0.0
            print('  ○ Gripper OPEN')

        elif key in ('c', 'C'):
            self.positions['grip_joint'] = JOINT_LIMITS['grip_joint'][0]
            print('  ● Gripper CLOSED')

        elif key in ('a', 'A'):
            self._apply_pose(HOME_POSE, 'HOME')

        elif key in ('z', 'Z'):
            self._apply_pose(INIT_POSE, 'INIT')

        elif key in ('b', 'B'):
            self._apply_pose(DOWN_POSE, 'DOWN')

        elif key in ('p', 'P'):
            if self._pick_place_busy:
                print('  [Pick-and-place already running]')
            else:
                t = threading.Thread(target=self._run_pick_place, daemon=True)
                t.start()

        elif key in ('h', 'H'):
            print(HELP_MENU)

        elif key in ('q', 'Q'):
            self._running = False

    def shutdown(self):
        self._running = False


# ---------------------------------------------------------------------------
# Keyboard reader (raw mode, non-blocking char read)
# ---------------------------------------------------------------------------
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    old_settings = termios.tcgetattr(sys.stdin)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while node._running and rclpy.ok():
            key = get_key(old_settings)
            node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
