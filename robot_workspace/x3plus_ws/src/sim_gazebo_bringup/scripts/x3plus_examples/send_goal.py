#!/usr/bin/env python3
"""Send a navigation goal to the robot via Nav2."""

import argparse
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.068]
        self._init_pub.publish(msg)
        self.get_logger().info(f'Initial pose set: ({x}, {y}, {yaw})')

    def send_goal(self, x, y, yaw, timeout=30.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quat(yaw)

        self.get_logger().info(f'Waiting for Nav2 action server...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Nav2 action server not available')
            return False

        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info(f'Goal accepted: ({x}, {y}, {yaw})')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
            return True
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')
            return False


def main():
    parser = argparse.ArgumentParser(
        description='Send a navigation goal to the robot via Nav2.')
    parser.add_argument('x', type=float, help='Goal X position')
    parser.add_argument('y', type=float, help='Goal Y position')
    parser.add_argument('yaw', nargs='?', type=float, default=0.0,
                        help='Goal orientation yaw (radians, default: 0)')
    parser.add_argument('--init-x', type=float, default=None,
                        help='Initial pose X (omit to skip setting initial pose)')
    parser.add_argument('--init-y', type=float, default=None,
                        help='Initial pose Y')
    parser.add_argument('--init-yaw', type=float, default=0.0,
                        help='Initial pose yaw (default: 0)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Seconds to wait for Nav2 server (default: 30)')

    args = parser.parse_args()

    rclpy.init()
    node = GoalSender()

    if args.init_x is not None:
        node.set_initial_pose(args.init_x, args.init_y or 0.0, args.init_yaw)

    success = node.send_goal(args.x, args.y, args.yaw, args.timeout)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
