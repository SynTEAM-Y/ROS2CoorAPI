#!/usr/bin/env python3
"""
Controls
========
Chassis:
  w/s/a/d : drive (forward/back/left/right)
  q/e     : rotate left/right
  space   : stop

Arm joints (1..6):
  '1' '2' '3' '4' '5' '6'  -> decrease that joint by step
  '! \" # ¤ % &'            -> increase that joint by step   (Shift+1..6)

Presets:
  '7' : all up
  '8' : track
  '9' : flat

Pick / place (depends on gripper state):
  'p' :
    - if gripper open  -> pick  (down open -> wait -> close -> up closed)
    - if gripper closed -> place (down closed -> wait -> open -> up open)

Gripper only:
  'g' : toggle gripper open / close (last joint only)

Misc:
  'b' : beep
  'j' : print current joint angles
  'x' : exit program
"""
import sys
import tty
import termios
import select
import time
from typing import Optional

from Rosmaster_Lib import Rosmaster

# ====================== Config ======================
SPEED = 25
JOINT_STEP = 2              # degrees per loop
MIN_ANGLE = 0
MAX_ANGLE = 180

# Slower motions for presets and sequences
POSE_DURATION_MS = 4000     # long smooth moves for presets
TICK_SEC = 0.05             # main loop tick
STOP_TIMEOUT = 0.15         # auto-stop chassis if no movement key held

# Gripper and pick sequence config
GRIPPER_JOINT_INDEX = 5      # last joint (0 based index)
GRIP_OPEN_ANGLE = 75         # open
GRIP_CLOSED_ANGLE = 137      # closed

# Up / down base poses (first 5 joints)
UP_BASE = [90, 140, 0, 0, 90]
DOWN_BASE = [84, 12, 49, 27, 89]

# Special block pick pose base (first 5 joints)
BLOCK_PICK_BASE = [90, 94, 88, 126, 90]

# Sequence timings
PICK_MOVE_DURATION_MS = 2000    # slow arm up/down motions
PICK_WAIT_SEC = 3.0             # wait time at bottom before grip or release
GRIP_MOVE_DURATION_MS = 800     # slower gripper move
# ====================================================

def make_pose(base, grip_angle: int):
    return base + [grip_angle]

# Preset poses for all joints (1..6)
PRESET_POSES = {
    #'7': make_pose([90, 90, 90, 90, 90], GRIP_OPEN_ANGLE),     # All up
    #'8': make_pose([90, 140, 0, 0, 90], GRIP_OPEN_ANGLE),    # Track (your original)
    #'9': make_pose([0, 0, 75, 35, 75], GRIP_OPEN_ANGLE),     # Flat  (your original)
}

# Map single key -> chassis state
CHASSIS_MAP = {
    'w': 1,  # forward
    's': 2,  # back
    'a': 3,  # left
    'd': 4,  # right
    'q': 5,  # rotate left
    'e': 6,  # rotate right
}

# Joint key bindings:
# digits 1..6 decrease that joint; shifted symbols increase it
INC_KEYS = {
    '!': 0,
    '"': 1,
    '#': 2,
    '¤': 3,
    '%': 4,
    '&': 5,
}
DEC_KEYS = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, '6': 5}

class RawInput:
    """Context manager to put tty into raw mode and poll single characters non blocking."""
    def __init__(self, fileobj):
        self.fd = fileobj.fileno()
        self.fileobj = fileobj
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_char(self, timeout: float = 0.0) -> Optional[str]:
        """Return a single character if available within timeout, else None."""
        rlist, _, _ = select.select([self.fileobj], [], [], timeout)
        if rlist:
            ch = self.fileobj.read(1)
            return ch
        return None

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def update_joints(bot: Rosmaster, joint_angles, preset: bool = False):
    """Send joint array to the arm. If 'preset' is True, use a longer duration for smooth motion."""
    time_param = POSE_DURATION_MS if preset else 200  # ms (must be >= 20 per SDK requirement)
    bot.set_uart_servo_angle_array(joint_angles, time_param)
    if preset:
        time.sleep(min(POSE_DURATION_MS / 1000.0, 0.3))

def go_to_pose(bot: Rosmaster, joint_angles, pose, duration_ms: int):
    """Blocking move to a full pose and keep joint_angles in sync."""
    bot.set_uart_servo_angle_array(pose, duration_ms)
    time.sleep(duration_ms / 1000.0)
    for i, v in enumerate(pose):
        joint_angles[i] = v

def set_gripper(bot: Rosmaster, joint_angles, angle: int, duration_ms: int = GRIP_MOVE_DURATION_MS):
    """Blocking move of gripper joint only, joint_angles stays in sync."""
    joint_angles[GRIPPER_JOINT_INDEX] = clamp(angle, MIN_ANGLE, MAX_ANGLE)
    bot.set_uart_servo_angle_array(joint_angles, duration_ms)
    time.sleep(duration_ms / 1000.0)

def pick_or_place_sequence(bot: Rosmaster,
                           joint_angles,
                           gripper_closed: bool) -> bool:
    """
    Use gripper state to decide:
      - if gripper_closed is False (open)  -> pick
      - if gripper_closed is True (closed) -> place

    Returns the new gripper_closed state at the end.
    """
    if not gripper_closed:
        # PICK: assume no object, gripper open
        down_pose = make_pose(DOWN_BASE, GRIP_OPEN_ANGLE)
        go_to_pose(bot, joint_angles, down_pose, PICK_MOVE_DURATION_MS)
        time.sleep(PICK_WAIT_SEC)

        # close gripper (grab)
        set_gripper(bot, joint_angles, GRIP_CLOSED_ANGLE)
        time.sleep(0.3)

        # go up with closed gripper
        up_pose = make_pose(UP_BASE, GRIP_CLOSED_ANGLE)
        go_to_pose(bot, joint_angles, up_pose, PICK_MOVE_DURATION_MS)
        return True
    else:
        # PLACE: assume we have object, gripper closed
        down_pose = make_pose(DOWN_BASE, GRIP_CLOSED_ANGLE)
        go_to_pose(bot, joint_angles, down_pose, PICK_MOVE_DURATION_MS)
        time.sleep(PICK_WAIT_SEC)

        # open gripper (release)
        set_gripper(bot, joint_angles, GRIP_OPEN_ANGLE)
        time.sleep(0.3)

        # go up with open gripper
        up_pose = make_pose(UP_BASE, GRIP_OPEN_ANGLE)
        go_to_pose(bot, joint_angles, up_pose, PICK_MOVE_DURATION_MS)
        return False

def special_block_pick(bot: Rosmaster,
                       joint_angles,
                       gripper_closed: bool) -> bool:
    """
    Fixed sequence for capital 'P':
      - if gripper is closed: open it first
      - move to BLOCK_PICK_BASE with open gripper
      - close gripper (pick block)
      - move to UP_BASE with closed gripper

    Returns new gripper_closed state (always True at the end).
    """
    # Ensure chassis is stopped before doing a long motion
    # (This is called from the main loop after stopping the base, just in case.)
    if gripper_closed:
        # open before going there
        set_gripper(bot, joint_angles, GRIP_OPEN_ANGLE)
        gripper_closed = False

    # move to block pick position with open gripper
    block_pose_open = make_pose(BLOCK_PICK_BASE, GRIP_OPEN_ANGLE)
    go_to_pose(bot, joint_angles, block_pose_open, PICK_MOVE_DURATION_MS)

    # small wait if needed for stability
    time.sleep(0.3)

    # close gripper to grab block
    set_gripper(bot, joint_angles, GRIP_CLOSED_ANGLE)
    gripper_closed = True

    # go back to UP_BASE with closed gripper
    up_closed = make_pose(UP_BASE, GRIP_CLOSED_ANGLE)
    go_to_pose(bot, joint_angles, up_closed, PICK_MOVE_DURATION_MS)

    return gripper_closed

def main():
    bot = Rosmaster(debug=False)
    bot.create_receive_threading()
    bot.set_car_type(bot.CARTYPE_X3_PLUS)

    car_stabilize_state = 0

    # Initialize joint angles (1..6) and move to a safe default (up with open gripper)
    joint_angles = make_pose(UP_BASE, GRIP_OPEN_ANGLE)
    bot.set_uart_servo_angle_array(joint_angles, 8000)
    time.sleep(2.0)

    # Shared flag for gripper state, used by both p, P and g
    gripper_closed = False  # starts open

    print("""
        Controls:
        w/s/a/d: move  q/e: rotate  space: stop  x: exit
        1..6: joint-   !\" # ¤ % &: joint+  7/8/9: presets
        p: pick/place (based on gripper)   g: gripper toggle
        b: beep   j: print angles
        """)

    last_move_time = 0.0

    with RawInput(sys.stdin) as ri:
        try:
            while True:
                start = time.time()
                preset_triggered = False
                move_applied = False

                # Drain all available chars this tick to capture multi presses
                while True:
                    ch = ri.read_char(timeout=0.0)
                    if ch is None:
                        break

                    # Normalize control characters
                    if ch == '\x03':  # Ctrl C
                        raise KeyboardInterrupt
                    if ch == '\x1b':  # ESC
                        continue
                    if ch == '\r' or ch == '\n':
                        continue

                    # Chassis
                    if ch in CHASSIS_MAP:
                        bot.set_car_run(CHASSIS_MAP[ch], SPEED, car_stabilize_state)
                        last_move_time = time.time()
                        move_applied = True
                        continue
                    if ch == ' ':
                        bot.set_car_run(0, SPEED, car_stabilize_state)
                        continue

                    # Joints
                    if ch in DEC_KEYS:
                        idx = DEC_KEYS[ch]
                        joint_angles[idx] = clamp(joint_angles[idx] - JOINT_STEP,
                                                  MIN_ANGLE, MAX_ANGLE)
                        continue
                    if ch in INC_KEYS:
                        idx = INC_KEYS[ch]
                        joint_angles[idx] = clamp(joint_angles[idx] + JOINT_STEP,
                                                  MIN_ANGLE, MAX_ANGLE)
                        continue

                    # Presets
                    if ch in PRESET_POSES:
                        joint_angles = PRESET_POSES[ch].copy()
                        preset_triggered = True
                        continue

                    # Special fixed block pick (capital P)
                    if ch == 'P':
                        bot.set_car_run(0, SPEED, car_stabilize_state)
                        last_move_time = 0.0
                        gripper_closed = special_block_pick(
                            bot,
                            joint_angles,
                            gripper_closed
                        )
                        continue

                    # Pick / place toggle (small p), based on gripper state
                    if ch == 'p':
                        # Hard stop chassis before long blocking arm motion
                        bot.set_car_run(0, SPEED, car_stabilize_state)
                        last_move_time = 0.0
                        gripper_closed = pick_or_place_sequence(
                            bot,
                            joint_angles,
                            gripper_closed
                        )
                        continue

                    # Gripper only toggle (small g)
                    if ch == 'g':
                        # Also stop chassis while doing gripper motion
                        bot.set_car_run(0, SPEED, car_stabilize_state)
                        last_move_time = 0.0
                        if gripper_closed:
                            # currently closed -> open
                            set_gripper(bot, joint_angles, GRIP_OPEN_ANGLE)
                            gripper_closed = False
                        else:
                            # currently open -> close
                            set_gripper(bot, joint_angles, GRIP_CLOSED_ANGLE)
                            gripper_closed = True
                        continue

                    # Misc
                    if ch == 'b':
                        bot.set_beep(100)
                        continue
                    if ch == 'j':
                        print("\nJoint Angles:", joint_angles)
                        continue
                    if ch == 'x':
                        print("Exiting...")
                        bot.set_car_run(0, SPEED, car_stabilize_state)
                        return

                # Auto stop if no movement key pressed recently
                if not move_applied and (time.time() - last_move_time) > STOP_TIMEOUT:
                    bot.set_car_run(0, SPEED, car_stabilize_state)

                # Push joint updates
                update_joints(bot, joint_angles, preset=preset_triggered)

                # Tick pacing
                elapsed = time.time() - start
                if elapsed < TICK_SEC:
                    time.sleep(TICK_SEC - elapsed)

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            bot.set_car_run(0, SPEED, car_stabilize_state)

if __name__ == "__main__":
    main()
