#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose

# -------------------------------------------------------------------
# Waypoint list: predefined patrol path with positions and orientations.
# Each entry contains x, y, and quaternion components (qz, qw).
# TODO: Move this to a YAML file for easier reconfiguration.
# -------------------------------------------------------------------

WAYPOINTS = [  # square_map.pgm
    {'x': -3.5, 'y': -3.5, 'qz':  0.0,    'qw': 1.0},
    {'x':  2.5, 'y': -3.5, 'qz':  0.7071, 'qw': 0.7071},
    {'x':  2.5, 'y':  2.5, 'qz':  1.0,    'qw': 0.0},
    {'x': -3.5, 'y':  2.5, 'qz': -0.7071, 'qw': 0.7071},
]

# WAYPOINTS = [ # circular_map.pgm
#     {'x': -1.784150, 'y': -1.396880, 'qz': -0.160491, 'qw': 0.987037},
#     {'x': -0.024653, 'y': -2.064980, 'qz':  0.170171, 'qw': 0.985415},
#     {'x':  1.682020, 'y': -1.479300, 'qz':  0.635610, 'qw': 0.772010},
#     {'x':  1.931800, 'y': -0.071620, 'qz':  0.771016, 'qw': 0.636815},
#     {'x':  1.594430, 'y':  1.544170, 'qz':  0.990803, 'qw': 0.135311},
#     {'x':  0.136965, 'y':  2.011860, 'qz': -0.995484, 'qw': 0.094929},
#     {'x': -1.538640, 'y':  1.650700, 'qz': -0.825484, 'qw': 0.564425},
#     {'x': -2.242360, 'y': -0.017370, 'qz': -0.537106, 'qw': 0.843514},
# ]

# -------------------------------------------------------------------
# PatrolRescue node
# - Runs a waypoint patrol loop using FollowWaypoints action
# - Supports interrupting patrol with a "rescue" command
# - Automatically resumes patrol after rescue if re-enabled
# -------------------------------------------------------------------
class PatrolRescue(Node):
    def __init__(self):
        # Retrieve namespace parameter (robot_ns) from temporary node
        tmp = Node('tmp_read_robot_ns')
        tmp.declare_parameter('robot_ns', 'robot')
        ns = tmp.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        tmp.destroy_node()

        # Main node runs within the robot namespace
        super().__init__('patrol_rescue', namespace=f'/{ns}')
        self.ns = ns

        # Parameters
        self.declare_parameter('rescue_xy', [0.0, 0.0])        # coordinates for rescue
        self.declare_parameter('waypoints_yaml', 'waypoints.yaml')

        # Internal state flags
        self.patrol_enabled = True
        self.patrol_active = False
        self.current_goal = None
        self.next_start_wp = 0  # next waypoint index after completion/resume

        # Action clients for navigation and waypoint following
        self.follow_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.nav_client    = ActionClient(self, NavigateToPose,   'navigate_to_pose')

        # Services for external control
        self.enable_srv = self.create_service(SetBool, 'patrol_enable', self.handle_enable)
        self.rescue_srv = self.create_service(Trigger,  'patrol_rescue', self.handle_rescue)

        # Wait for required action servers
        self.follow_client.wait_for_server()
        self.nav_client.wait_for_server()

        # Print control info and start initial patrol cycle
        self._print_banner()
        self.start_patrol()

    # ---------------- Service callbacks ----------------
    def handle_enable(self, req, res):
        """Enable or disable the patrol loop via /patrol_enable service."""
        if req.data:
            # Enable patrol if not already running
            if self.patrol_active:
                res.success, res.message = True, 'Patrol already running'
                return res
            self.patrol_enabled = True
            self.start_patrol()
            res.success, res.message = True, f'Patrol enabled from index {self.next_start_wp}'
        else:
            # Stop patrol and cancel any active goals
            self.patrol_enabled = False
            self.cancel_current_goal()
            res.success, res.message = True, 'Patrol stopped'
        return res

    def handle_rescue(self, _req, res):
        """Interrupt patrol and navigate to a rescue point."""
        xy = self.get_parameter('rescue_xy').get_parameter_value().double_array_value
        if len(xy) < 2:
            res.success, res.message = False, 'Invalid rescue_xy parameter'
            return res

        x, y = float(xy[0]), float(xy[1])
        self.get_logger().info(f'Rescue triggered -> ({x:.3f}, {y:.3f})')

        # Stop patrol and cancel active goal
        self.patrol_enabled = False
        self.cancel_current_goal()

        # Send direct navigation goal to the rescue point
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        goal.pose = ps

        self.nav_client.send_goal_async(goal)
        res.success, res.message = True, f'Moving to rescue point ({x:.3f}, {y:.3f})'
        return res

    # ---------------- Patrol management ----------------
    def build_follow_goal_from(self, start_idx: int):
        """Build a FollowWaypoints.Goal starting from the given waypoint index."""
        n = len(WAYPOINTS)
        if n == 0:
            return FollowWaypoints.Goal()

        start_idx = max(0, min(start_idx, n - 1))
        order = list(range(start_idx, n)) + list(range(0, start_idx))

        g = FollowWaypoints.Goal()
        for i in order:
            p = WAYPOINTS[i]
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(p['x'])
            ps.pose.position.y = float(p['y'])
            ps.pose.orientation.z = float(p['qz'])
            ps.pose.orientation.w = float(p['qw'])
            g.poses.append(ps)
        return g

    def start_patrol(self):
        """Begin the patrol cycle if enabled and not already active."""
        if not self.patrol_enabled:
            return
        if self.patrol_active:
            self.get_logger().warn('Patrol start requested but already active.')
            return

        self.next_start_wp = 0 # This disables continuation when resuming patrol. It instead resets from the first waypoint.
        goal = self.build_follow_goal_from(self.next_start_wp)
        self.get_logger().info(f'Starting patrol from waypoint index {self.next_start_wp}...')
        self.patrol_active = True
        self.follow_client.send_goal_async(
            goal,
            feedback_callback=self._on_follow_feedback
        ).add_done_callback(self._on_follow_sent)

    # ---------------- Action callbacks ----------------
    def _on_follow_feedback(self, feedback_msg):
        """Update next starting index based on current waypoint progress."""
        try:
            idx = int(feedback_msg.feedback.current_waypoint)
            self.next_start_wp = (idx + 1) % len(WAYPOINTS)
        except Exception:
            pass

    def _on_follow_sent(self, fut):
        """Handle action goal acceptance or rejection."""
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Patrol goal rejected')
            self.patrol_active = False
            return
        self.current_goal = gh
        gh.get_result_async().add_done_callback(self._on_follow_done)

    def _on_follow_done(self, _fut):
        """Called when patrol cycle finishes; restarts if still enabled."""
        self.get_logger().info('Patrol cycle complete.')
        self.patrol_active = False
        if self.patrol_enabled:
            # Restart patrol after short delay
            self.create_timer(1.0, self._repeat_once)

    def _repeat_once(self):
        """Timer callback to trigger next patrol cycle."""
        timers = list(self.timers)
        if timers:
            self.destroy_timer(timers[0])
        self.start_patrol()

    def cancel_current_goal(self):
        """Cancel the currently active patrol goal if any."""
        if self.current_goal:
            try:
                self.current_goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Cancel failed: {e}')
        self.patrol_active = False

    # ---------------- Info banner ----------------
    def _print_banner(self):
        """Print helpful ROS 2 commands for controlling patrol and rescue."""
        ns = self.ns
        lines = [
            "",
            "================= patrol_rescue controls =================",
            f" Robot namespace: {ns}",
            "",
            " Stop patrol:",
            f"   ros2 service call /{ns}/patrol_enable std_srvs/srv/SetBool \"{{data: false}}\"",
            "",
            " Resume patrol from last waypoint:",
            f"   ros2 service call /{ns}/patrol_enable std_srvs/srv/SetBool \"{{data: true}}\"",
            "",
            " Send rescue to (x, y):",
            f"   ros2 param set /{ns}/patrol_rescue rescue_xy \"[x, y]\"",
            f"   ros2 service call /{ns}/patrol_rescue std_srvs/srv/Trigger \"{{}}\"",
            "",
            " Notes:",
            "  - Only one patrol cycle runs at a time per robot",
            "  - After rescue, re-enable patrol to continue from the last waypoint",
            "===========================================================",
            ""
        ]
        for L in lines:
            print(L)


# ---------------- Entry point ----------------
def main():
    rclpy.init()
    node = PatrolRescue()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
