#!/usr/bin/env python3
"""
Arm Controller Node

Interactive keyboard control for the 5-DOF robot arm and gripper.

Controls:
  1-5: Select arm joint (arm_joint1 through arm_joint5)
  6: Select gripper
  W/S: Increase/decrease selected joint position
  O/C: Open/close gripper
  A: Home pose
  Z: Init pose
  B: Down pose
  P: Pick and place sequence
  H: Help menu
  Q: Quit
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select
import math
import threading


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Arm joint names
        self.arm_joints = [
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'arm_joint4',
            'arm_joint5'
        ]
        
        # Create publishers for each arm joint
        self.arm_pubs = {}
        for joint_name in self.arm_joints:
            topic_name = f'/{joint_name}_cmd_pos'
            self.arm_pubs[joint_name] = self.create_publisher(
                Float64,
                topic_name,
                10
            )
        
        # Gripper publisher
        self.gripper_pub = self.create_publisher(
            Float64,
            '/grip_joint_cmd_pos',
            10
        )
        
        # Subscribe to joint states for feedback. Use SensorDataQoS so we are
        # compatible with gripper_mimic_relay's BEST_EFFORT publisher (avoids
        # the 'incompatible QoS' warning and missed updates).
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        
        # Current joint positions
        self.current_positions = {
            'arm_joint1': 0.0,
            'arm_joint2': 0.0,
            'arm_joint3': 0.0,
            'arm_joint4': 0.0,
            'arm_joint5': 0.0,
            'grip_joint': 0.0
        }
        
        # Control parameters
        self.selected_joint_idx = 0  # 0-4 for arm joints, 5 for gripper
        self.joint_increment = 0.15  # radians per W/S press
        # Gripper convention (matches manufacturer SRDF):
        #   grip_joint =  0.00  -> fingers CLOSED (servo center 90°)
        #   grip_joint = -1.54  -> fingers fully OPEN (matches SRDF "open" state)
        self.gripper_open_pos   = -1.54
        self.gripper_closed_pos =  0.00

        # Predefined poses — arm_joint2 NEGATIVE = tilts FORWARD (away from car body)
        #                    arm_joint2 POSITIVE = tilts BACKWARD (into car body — AVOID)
        # All poses keep arm_joint2 <= 0 so the arm stays in the forward half-space.
        self.home_pose  = [0.0,  0.0,    0.0,    0.0,  0.0]  # arm straight up (safe)
        self.init_pose  = [0.0, -0.5,   -0.4,    0.0,  0.0]  # arm forward-raised (safe ready)
        self.down_pose  = [0.0, -1.0,   -0.3,   -0.3,  0.0]  # arm reaching forward-down

        # Smooth trajectory: target positions, velocity & acceleration limits
        # Tuned for fast pick-and-place while staying within what the (lightly
        # -tuned) Gazebo PIDs can track without overshoot or base shake.
        # Increased from 5.0 / 18.0 → 9.0 / 35.0 for ~1.7× faster motion.
        self.smooth_speed = 9.0    # max joint speed (rad/s)
        self.smooth_accel = 35.0   # max joint acceleration (rad/s²)
        # _smooth_pos is the internal integrator — NEVER overwritten by callbacks
        self._smooth_pos = dict(self.current_positions)
        # Gripper starts CLOSED to match diff_drive_simulator initial state
        self._smooth_pos['grip_joint'] = self.gripper_closed_pos
        # Per-joint velocity state for trapezoidal profile
        self._smooth_vel = {k: 0.0 for k in self._smooth_pos}
        # target_positions is set by key presses
        self.target_positions = dict(self._smooth_pos)
        self._pick_place_running = False  # guard against double-start

        # 200 Hz timer drives the smooth interpolation (was 100 Hz)
        self.motion_timer = self.create_timer(0.005, self._motion_step)
        
        self.get_logger().info('========================================')
        self.get_logger().info('ARM CONTROLLER INITIALIZED')
        self.get_logger().info('========================================')
        self.print_help()
    
    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]
    
    def print_help(self):
        """Print help menu"""
        print('\n' + '='*60)
        print('ARM & GRIPPER CONTROL')
        print('='*60)
        print('Joint Selection:')
        print('  1-5     : Select arm joint (arm_joint1 to arm_joint5)')
        print('  6       : Select gripper')
        print('')
        print('Movement:')
        print('  W       : Increase selected joint position (+0.1 rad)')
        print('  S       : Decrease selected joint position (-0.1 rad)')
        print('')
        print('Gripper:')
        print('  O       : Open gripper')
        print('  C       : Close gripper')
        print('')
        print('Predefined Poses:')
        print('  A       : Home pose (all joints to zero)')
        print('  Z       : Init pose (ready position)')
        print('  B       : Down pose (reaching down)')
        print('  P       : Pick and place sequence (automated)')
        print('')
        print('System:')
        print('  H       : Show this help')
        print('  Q       : Quit')
        print('='*60)
        self.print_status()
    
    def print_status(self):
        """Print current status"""
        if self.selected_joint_idx < 5:
            joint_name = self.arm_joints[self.selected_joint_idx]
            current_pos = self._smooth_pos[joint_name]
            target_pos  = self.target_positions[joint_name]
            print(f'\nSelected: {joint_name} (pos: {current_pos:.2f} → {target_pos:.2f} rad)')
        else:
            current_pos = self._smooth_pos['grip_joint']
            status = 'OPEN' if current_pos < -0.5 else 'CLOSED'
            print(f'\nSelected: Gripper (current: {current_pos:.2f} rad, status: {status})')
    
    def _step_joint(self, name, publisher, dt):
        """Trapezoidal-velocity step toward target for one joint.
        Limits both velocity (smooth_speed) and acceleration (smooth_accel),
        and decelerates so the joint arrives at target with zero velocity."""
        pos = self._smooth_pos[name]
        vel = self._smooth_vel[name]
        target = self.target_positions[name]
        error = target - pos

        # Snap-to-target deadzone: when both error and velocity are small, stop cleanly
        # Prevents asymptotic oscillation around the target.
        if abs(error) < 5e-3 and abs(vel) < 0.2:
            if pos != target or vel != 0.0:
                self._smooth_pos[name] = target
                self._smooth_vel[name] = 0.0
                msg = Float64()
                msg.data = target
                publisher.publish(msg)
            return

        # Desired velocity that allows decel-to-zero at the target:
        #   v_max_to_stop = sqrt(2 * a * |error|), signed by error direction
        v_stop = math.copysign(math.sqrt(2.0 * self.smooth_accel * abs(error)), error)
        # Cap by absolute speed limit
        v_des = max(-self.smooth_speed, min(self.smooth_speed, v_stop))

        # Apply acceleration limit when changing velocity
        dv_max = self.smooth_accel * dt
        dv = v_des - vel
        if dv > dv_max:
            dv = dv_max
        elif dv < -dv_max:
            dv = -dv_max
        vel = vel + dv

        # Integrate position
        pos = pos + vel * dt

        self._smooth_pos[name] = pos
        self._smooth_vel[name] = vel

        msg = Float64()
        msg.data = pos
        publisher.publish(msg)

    def _motion_step(self):
        """Called at 200 Hz — trapezoidal profile for every joint."""
        dt = 0.005
        for joint_name in self.arm_joints:
            self._step_joint(joint_name, self.arm_pubs[joint_name], dt)
        self._step_joint('grip_joint', self.gripper_pub, dt)

    def set_joint_target(self, joint_idx, position):
        """Set smooth target for a single joint."""
        if joint_idx < 5:
            joint_name = self.arm_joints[joint_idx]
            self.target_positions[joint_name] = position
            self.get_logger().info(f'Target {joint_name}: {position:.2f} rad ({math.degrees(position):.1f}°)')
        else:
            self.target_positions['grip_joint'] = position
            status = 'OPEN' if position < -0.5 else 'CLOSED'
            self.get_logger().info(f'Target gripper: {position:.2f} rad ({status})')

    def publish_joint_position(self, joint_idx, position):
        """Set smooth target for a joint (kept for backwards compatibility)."""
        self.set_joint_target(joint_idx, position)

    def publish_pose(self, positions, gripper_pos=None):
        """Set smooth targets for a full arm pose."""
        for i, pos in enumerate(positions):
            joint_name = self.arm_joints[i]
            self.target_positions[joint_name] = pos
        if gripper_pos is not None:
            self.target_positions['grip_joint'] = gripper_pos
    
    def execute_pick_and_place(self):
        """Launch pick and place in a background thread so the motion timer keeps running."""
        if self._pick_place_running:
            self.get_logger().warn('Pick and place already running — ignoring')
            return
        # Sync the trajectory generator's internal state to the latest measured
        # joint positions so the first step doesn't see a phantom step from 0.
        for joint_name in list(self._smooth_pos.keys()):
            if joint_name in self.current_positions:
                self._smooth_pos[joint_name] = self.current_positions[joint_name]
                self._smooth_vel[joint_name] = 0.0
                self.target_positions[joint_name] = self.current_positions[joint_name]
        t = threading.Thread(target=self._pick_place_thread, daemon=True)
        t.start()

    def _motion_done(self, tol=0.02):
        """Return True when all joints are at their targets and stopped."""
        for joint_name in self.arm_joints:
            if abs(self._smooth_pos[joint_name] - self.target_positions[joint_name]) > tol:
                return False
            if abs(self._smooth_vel[joint_name]) > 0.01:
                return False
        if abs(self._smooth_pos['grip_joint'] - self.target_positions['grip_joint']) > tol:
            return False
        if abs(self._smooth_vel['grip_joint']) > 0.01:
            return False
        return True

    def _wait_motion_done(self, timeout=6.0, settle=0.02):
        """Block (in background thread) until motion completes or timeout.
        Adds a short settle delay after completion for physical stability."""
        import time
        # Allow timer to register new target before checking
        time.sleep(0.02)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._motion_done():
                time.sleep(settle)
                return True
            time.sleep(0.005)
        return False  # timed out

    def _pick_place_thread(self):
        """Runs in a background thread — waits for each motion to complete before next step.
        Trajectory is designed so the arm always stays in the FORWARD half-space,
        avoiding collision with the camera tower behind base_link."""
        import time
        self._pick_place_running = True
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING PICK AND PLACE SEQUENCE')
        self.get_logger().info('='*60)

        # All poses keep arm_joint2 <= 0 (forward-tilting, never backward into car body).
        # reach_down = manufacturer servo [90, 2, 60, 40, 90] → URDF rad conversion
        pre_pick   = [0.0,  -0.8,  -0.4, -0.3,  0.0]  # arm forward-raised, approach
        reach_down = [0.0,  -1.536, -0.524, -0.873, 0.0]  # mfr reach-down (servo 2,60,40)
        lifted     = [0.0,  -0.8,  -0.4, -0.3,  0.0]  # same as pre_pick, holding object up
        # Rotate through home (arm_joint2=0) so arm stays vertical during turn — avoids sweep collision
        rot_lifted = [1.2,  -0.8,  -0.4, -0.3,  0.0]  # rotated ~69° left, arm raised forward
        rot_reach  = [1.2,  -1.536, -0.524, -0.873, 0.0]  # rotated, reaching down to place

        # (description, arm_pos, grip_pos, post_pause_s)
        # Pauses kept only where physically necessary (gripper grasp/release
        # need a brief settle so the contact registers); other inter-step
        # pauses removed — the trapezoidal profile already decelerates to
        # zero before the next target is loaded.
        steps = [
            ('1. Home — arm up, gripper open',     self.home_pose, self.gripper_open_pos,   0.0),
            ('2. Pre-pick approach',               pre_pick,       self.gripper_open_pos,   0.0),
            ('3. Reach down to object',            reach_down,     self.gripper_open_pos,   0.1),
            ('4. Close gripper',                   reach_down,     self.gripper_closed_pos, 0.4),
            ('5. Lift to carry height',            lifted,         self.gripper_closed_pos, 0.0),
            ('6. Return to home (safe for rotate)',self.home_pose, self.gripper_closed_pos, 0.0),
            ('7. Rotate to place side',            rot_lifted,     self.gripper_closed_pos, 0.0),
            ('8. Lower to place location',         rot_reach,      self.gripper_closed_pos, 0.1),
            ('9. Open gripper — release',          rot_reach,      self.gripper_open_pos,   0.4),
            ('10. Lift from place',                rot_lifted,     self.gripper_open_pos,   0.0),
            ('11. Return toward home',             self.home_pose, self.gripper_open_pos,   0.0),
            ('12. Close gripper at home',          self.home_pose, self.gripper_closed_pos, 0.2),
        ]

        for step_name, arm_pos, grip_pos, post_pause in steps:
            self.get_logger().info(step_name)
            self.publish_pose(arm_pos, grip_pos)
            if not self._wait_motion_done(timeout=8.0):
                self.get_logger().warn(f'Timeout on: {step_name}')
            if post_pause > 0.0:
                time.sleep(post_pause)

        self.get_logger().info('='*60)
        self.get_logger().info('PICK AND PLACE SEQUENCE COMPLETED')
        self.get_logger().info('='*60)
        self._pick_place_running = False
    
    def handle_keypress(self, key):
        """Handle keyboard input"""
        # Joint selection (1-6)
        if key in ['1', '2', '3', '4', '5']:
            self.selected_joint_idx = int(key) - 1
            self.print_status()
            return True
        elif key == '6':
            self.selected_joint_idx = 5
            self.print_status()
            return True
        
        # Movement (W/S)
        elif key.lower() == 'w':
            if self.selected_joint_idx < 5:
                joint_name = self.arm_joints[self.selected_joint_idx]
                new_target = self.target_positions[joint_name] + self.joint_increment
                self.set_joint_target(self.selected_joint_idx, new_target)
            else:
                # W: decrease grip value toward gripper_open_pos (-1.0)
                new_target = max(self.target_positions['grip_joint'] - self.joint_increment, self.gripper_open_pos)
                self.set_joint_target(5, new_target)
            return True

        elif key.lower() == 's':
            if self.selected_joint_idx < 5:
                joint_name = self.arm_joints[self.selected_joint_idx]
                new_target = self.target_positions[joint_name] - self.joint_increment
                self.set_joint_target(self.selected_joint_idx, new_target)
            else:
                # S: increase grip value toward gripper_closed_pos (0.0)
                new_target = min(self.target_positions['grip_joint'] + self.joint_increment, self.gripper_closed_pos)
                self.set_joint_target(5, new_target)
            return True
        
        # Gripper control (O/C)
        elif key.lower() == 'o':
            self.publish_joint_position(5, self.gripper_open_pos)
            return True
        
        elif key.lower() == 'c':
            self.publish_joint_position(5, self.gripper_closed_pos)
            return True
        
        # Predefined poses (A/Z/B/P)
        elif key.lower() == 'a':
            self.get_logger().info('Moving to HOME pose')
            self.publish_pose(self.home_pose)
            return True
        
        elif key.lower() == 'z':
            self.get_logger().info('Moving to INIT pose')
            self.publish_pose(self.init_pose)
            return True
        
        elif key.lower() == 'b':
            self.get_logger().info('Moving to DOWN pose')
            self.publish_pose(self.down_pose)
            return True
        
        elif key.lower() == 'p':
            self.execute_pick_and_place()
            return True
        
        # Help and quit
        elif key.lower() == 'h':
            self.print_help()
            return True
        
        elif key.lower() == 'q':
            self.get_logger().info('Quitting...')
            return False
        
        return True


def get_key(settings):
    """Get a single keypress from terminal.

    Uses cbreak mode (not raw) so OPOST output processing stays enabled —
    that keeps '\\n' translating to '\\r\\n' on stdout, so log lines from
    rclpy don't stair-step across the terminal.
    """
    tty.setcbreak(sys.stdin.fileno())
    # Use select to check if input is available (non-blocking with timeout)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        while rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Check for keyboard input
            key = get_key(settings)
            if key:
                if not node.handle_keypress(key):
                    break
    
    except Exception as e:
        print(f'Error: {e}')
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
