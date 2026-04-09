#!/usr/bin/env python3
import math
import time
import os
import rclpy
from rclpy.node import Node
import rclpy.qos
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist

# Yahboom SDK
from Rosmaster_Lib import Rosmaster

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class RosmasterBaseBridge(Node):
    """
    Bridge geometry_msgs/Twist (/<ns>/cmd_vel) -> Yahboom Rosmaster set_car_run(state, speed%, stabilize)
      States per Yahboom: 0=stop, 1=fwd, 2=back, 3=left, 4=right, 5=rotate left, 6=rotate right

    Speed policy:
      - For translation:  speed% = min( |vx| / v_max, 1.0 ) * 100
      - For rotation:     speed% = min( |az| / w_max, 1.0 ) * 100
      - Rotation has priority when |az| > ang_deadband.
    """

    def __init__(self):
        super().__init__('rosmaster_base_bridge')

        # --- Tunables ---
        # velocity scales (pick realistic maxima for your robot)
        self.v_max = self.declare_parameter('v_max', 0.50).value      # m/s that maps to 100%
        self.w_max = self.declare_parameter('w_max', 2.00).value      # rad/s that maps to 100%

        # deadbands to ignore tiny noises
        self.lin_deadband = self.declare_parameter('lin_deadband', 0.05).value  # m/s
        self.ang_deadband = self.declare_parameter('ang_deadband', 0.15).value  # rad/s

        # stabilization flag passed to SDK (0/1 as per Yahboom docs)
        self.stabilize = int(bool(self.declare_parameter('stabilize', 1).value))

        # optional accel limit: max change in speed% per command step (0 disables)
        self.accel_limit = float(self.declare_parameter('accel_limit_pct_per_s', 150.0).value)

        # safety: stop if no cmd received for this many seconds (0 disables)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)

        # --- SDK bring-up ---
        self.bot = Rosmaster(debug=False)
        self.bot.create_receive_threading()
        try:
            self.bot.set_car_type(self.bot.CARTYPE_X3_PLUS)
        except Exception:
            pass

        # Determine namespaced /cmd_vel for robot
        robot_param = self.declare_parameter('robot', '').value.strip()
        robot_env = os.getenv('ROBOT', '').strip()

        if robot_param:
            cmd_vel_topic = f'/{robot_param}/cmd_vel'
        elif robot_env:
            cmd_vel_topic = f'/{robot_env}/cmd_vel'
        else:
            cmd_vel_topic = '/cmd_vel'

        # --- Subscriptions ---
        self.sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cb_cmd, QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # --- State ---
        self.last_state = 0
        self.last_speed_pct = 0.0
        self.last_cmd_time = time.monotonic()

        # periodic watchdog for timeout stopping
        self.timer = self.create_timer(0.05, self.watchdog)  # 20 Hz

        self.get_logger().info(
            f"Rosmaster base bridge started | v_max={self.v_max:.2f} m/s, "
            f"w_max={self.w_max:.2f} rad/s, deadbands=(lin {self.lin_deadband}, "
            f"ang {self.ang_deadband}), stabilize={self.stabilize}, "
            f"accel_limit%/s={self.accel_limit}, timeout={self.cmd_timeout}s, "
            f"cmd_vel_topic='{cmd_vel_topic}'"
        )

    # --- Helpers ---

    def speed_from_linear(self, vx: float) -> int:
        pct = abs(vx) / max(self.v_max, 1e-6) * 100.0
        return int(round(clamp(pct, 0.0, 100.0)))

    def speed_from_angular(self, az: float) -> int:
        pct = abs(az) / max(self.w_max, 1e-6) * 100.0
        return int(round(clamp(pct, 0.0, 100.0)))

    def rate_limit(self, target_pct: float, dt: float) -> float:
        if self.accel_limit <= 0.0 or dt <= 0.0:
            return target_pct
        max_step = self.accel_limit * dt
        diff = target_pct - self.last_speed_pct
        if abs(diff) <= max_step:
            return target_pct
        return self.last_speed_pct + math.copysign(max_step, diff)

    def set_motion(self, state: int, speed_pct: int):
        # Avoid redundant calls; always clamp 0..100
        speed_pct = int(clamp(speed_pct, 0, 100))
        if state != self.last_state or speed_pct != int(self.last_speed_pct):
            self.bot.set_car_run(state, speed_pct, self.stabilize)
            self.last_state = state
            self.last_speed_pct = float(speed_pct)

    # --- Callbacks / timers ---

    def cb_cmd(self, msg: Twist):
        now = time.monotonic()
        self.last_cmd_time = now

        vx = msg.linear.x
        az = msg.angular.z

        # apply deadbands
        if abs(vx) < self.lin_deadband:
            vx = 0.0
        if abs(az) < self.ang_deadband:
            az = 0.0

        # choose motion mode; rotation has priority
        if az != 0.0:
            # rotate left/right with speed from |az|
            target_pct = self.speed_from_angular(az)
            # rate limit
            dt = 0.0  # we're directly commanding on receive, watchdog handles smoothing if needed
            target_pct = self.rate_limit(target_pct, dt)
            state = 5 if az > 0.0 else 6
            self.set_motion(state, int(round(target_pct)))
            return

        # translation forward/back
        if vx != 0.0:
            target_pct = self.speed_from_linear(vx)
            dt = 0.0
            target_pct = self.rate_limit(target_pct, dt)
            state = 1 if vx > 0.0 else 2
            self.set_motion(state, int(round(target_pct)))
            return

        # stop
        self.set_motion(0, 0)

    def watchdog(self):
        """Stop the robot if no commands have been heard recently."""
        if self.cmd_timeout <= 0.0:
            return
        if (time.monotonic() - self.last_cmd_time) > self.cmd_timeout:
            if self.last_state != 0 or self.last_speed_pct != 0.0:
                self.set_motion(0, 0)

def main():
    rclpy.init()
    node = RosmasterBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.set_motion(0, 0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
