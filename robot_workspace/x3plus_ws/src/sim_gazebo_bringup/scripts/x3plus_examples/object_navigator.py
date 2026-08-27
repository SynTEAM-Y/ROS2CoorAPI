#!/usr/bin/env python3
"""Navigate to detected objects using Nav2.

Subscribes to /detected_object_pose (PoseStamped in camera frame),
transforms to map frame via TF2, sends goal to Nav2 /navigate_to_pose.

Usage:
  ros2 run sim_gazebo_bringup object_navigator
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros
import tf2_geometry_msgs


class ObjectNavigator(Node):
    def __init__(self):
        super().__init__('object_navigator')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('pose_topic', '/detected_object_pose')
        self.declare_parameter('action_server', '/navigate_to_pose')
        self.declare_parameter('cooldown_sec', 3.0)
        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('debounce_m', 0.5)

        self._target_frame = self.get_parameter('target_frame').value
        self._cooldown_sec = self.get_parameter('cooldown_sec').value
        self._goal_tolerance = self.get_parameter('goal_tolerance_m').value
        self._debounce_m = self.get_parameter('debounce_m').value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._action_client = ActionClient(
            self, NavigateToPose, self.get_parameter('action_server').value)

        self._sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self._on_pose, 1)

        self._last_goal = None
        self._last_goal_time = self.get_clock().now()
        self._goal_pending = False
        self._goal_handle = None

        self.get_logger().info(
            f'Listening on /detected_object_pose, sending goals to map frame')

    def _on_pose(self, msg: PoseStamped) -> None:
        # Debounce: skip if we already have a goal pending at similar position
        if self._goal_pending:
            return
        now = self.get_clock().now()
        if (now - self._last_goal_time).nanoseconds < self._cooldown_sec * 1e9:
            return
        if self._last_goal is not None:
            dx = msg.pose.position.x - self._last_goal.pose.position.x
            dy = msg.pose.position.y - self._last_goal.pose.position.y
            if (dx * dx + dy * dy) ** 0.5 < self._debounce_m:
                return

        # Look up transform from source frame -> target frame
        try:
            t = self._tf_buffer.lookup_transform(
                self._target_frame, msg.header.frame_id,
                rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return

        pose_map = tf2_geometry_msgs.do_transform_pose_stamped(msg, t)

        # Check distance to goal tolerance
        if self._last_goal is not None:
            dx = pose_map.pose.position.x - self._last_goal.pose.position.x
            dy = pose_map.pose.position.y - self._last_goal.pose.position.y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < self._goal_tolerance:
                self.get_logger().info(
                    f'Object already at goal (within {dist:.2f}m tolerance), skipping')
                return

        self.get_logger().info(
            f'Navigating to object at '
            f'({pose_map.pose.position.x:.2f}, {pose_map.pose.position.y:.2f}) '
            f'in {self._target_frame} frame')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_map

        self._goal_pending = True
        self._last_goal = pose_map
        self._last_goal_time = now

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        pass

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by Nav2')
            self._goal_pending = False
            return
        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().info(f'Navigation finished with status {status}')
        self._goal_pending = False
        self._goal_handle = None


def main():
    rclpy.init()
    node = ObjectNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
