#!/usr/bin/env python3
"""Trajectory bridge: FollowJointTrajectory action server -> Float64 publishers.

Converts FollowJointTrajectory action goals into Float64 position commands
on the Ignition bridge topics (/arm_joint*_cmd_pos, /grip_joint_cmd_pos).
Each trajectory point is published at its time_from_start so the arm moves
smoothly rather than jumping instantly to the final position.
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64
from builtin_interfaces.msg import Duration
import threading


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')

        self.arm_publishers = {}
        for i in range(1, 6):
            topic = f'/arm_joint{i}_cmd_pos'
            self.arm_publishers[f'arm_joint{i}'] = self.create_publisher(Float64, topic, 10)
        self.grip_pub = self.create_publisher(Float64, '/grip_joint_cmd_pos', 10)

        self.arm_joint_names = [f'arm_joint{i}' for i in range(1, 6)]

        self._arm_server = ActionServer(
            self, FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory',
            execute_callback=self._execute_arm,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self._grip_server = ActionServer(
            self, FollowJointTrajectory,
            '/gripper_group_controller/follow_joint_trajectory',
            execute_callback=self._execute_gripper,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self._cancel_requested = False
        self._lock = threading.Lock()
        self.get_logger().info('Trajectory bridge started')

    def _goal_cb(self, _):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _):
        with self._lock:
            self._cancel_requested = True
        return CancelResponse.ACCEPT

    def _publish_arm(self, joint_names, positions):
        for name, pos in zip(joint_names, positions):
            pub = self.arm_publishers.get(name)
            if pub:
                pub.publish(Float64(data=float(pos)))

    def _sec(self, dur: Duration) -> float:
        return dur.sec + dur.nanosec * 1e-9

    def _execute_timed(self, goal_handle, publish_fn, traj, label):
        """Execute a trajectory for a PID position controller.

        PID controllers (Ignition JointPositionController) drive smoothly
        toward a commanded position.  Sending 100+ intermediate points
        faster than the physics step causes the PID to chase a moving
        target and never settle, producing jerky "strange" motion.

        The correct approach is to send only a few evenly-spaced
        waypoints with enough time between them for the PID to catch up.
        """
        with self._lock:
            self._cancel_requested = False

        num = len(traj.points)
        self.get_logger().info(f'{label} trajectory: {num} point(s)')
        result = FollowJointTrajectory.Result()

        if num == 0:
            self.get_logger().warning(f'{label}: empty trajectory')
            goal_handle.succeed()
            result.error_code = result.SUCCESSFUL
            return result

        # Use only a small number of waypoints so the PID can settle
        # between commands.  5-8 waypoints is plenty for a PID.
        MAX_WAYPOINTS = 6
        if num > MAX_WAYPOINTS:
            indices = [int(round(i * (num - 1) / (MAX_WAYPOINTS - 1)))
                       for i in range(MAX_WAYPOINTS)]
            waypoints = [traj.points[i] for i in indices]
        else:
            waypoints = traj.points

        n_wp = len(waypoints)
        total_time = self._sec(traj.points[-1].time_from_start)
        has_timing = total_time > 0.05

        if has_timing:
            duration = total_time
        else:
            duration = 3.0  # seconds — slow enough to see the motion

        step_dt = duration / max(n_wp, 1)
        self.get_logger().info(
            f'{label}: sending {n_wp} waypoints over {duration:.1f}s '
            f'(dt={step_dt:.2f}s)'
        )

        t0 = time.monotonic()
        prev_time = 0.0

        for i, pt in enumerate(waypoints):
            if self._check_cancel(goal_handle, result):
                return result

            target_rel = (i + 1) * step_dt
            wait_s = target_rel - prev_time
            if wait_s > 0.001:
                time.sleep(wait_s)

            publish_fn(pt)
            prev_time = target_rel

            pos_str = ', '.join(f'{p:.3f}' for p in pt.positions[:3])
            self.get_logger().info(
                f'{label} wp {i + 1}/{n_wp}: [{pos_str}...]'
            )

        elapsed = time.monotonic() - t0
        self.get_logger().info(f'{label}: finished in {elapsed:.2f}s')
        goal_handle.succeed()
        result.error_code = result.SUCCESSFUL
        return result

    def _execute_arm(self, goal_handle):
        traj = goal_handle.request.trajectory
        return self._execute_timed(
            goal_handle,
            lambda pt: self._publish_arm(traj.joint_names, pt.positions),
            traj,
            'Arm'
        )

    def _execute_gripper(self, goal_handle):
        traj = goal_handle.request.trajectory
        return self._execute_timed(
            goal_handle,
            lambda pt: self.grip_pub.publish(Float64(data=float(pt.positions[0]))) if pt.positions else None,
            traj,
            'Gripper'
        )

    def _check_cancel(self, goal_handle, result):
        with self._lock:
            if self._cancel_requested:
                self.get_logger().info('Trajectory cancelled')
                goal_handle.canceled()
                result.error_code = result.SUCCESSFUL
                return True
        return False


def main():
    rclpy.init()
    node = TrajectoryBridge()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
