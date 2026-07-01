#!/usr/bin/env python3
"""
Gripper Cube Attach/Detach Service
================================
Attaches the test_block cube to the gripper's rlink2 link via a fixed
joint when the gripper is at REACH_DOWN, and detaches it (removes the
joint) when the gripper opens at PLACE_DOWN. This sidesteps the broken
gripper physics by treating the gripper as a rigid "claw".

Uses the standard Ignition Gazebo services via the ros_gz_interfaces
wrappers:
  /world/<world>/create   ros_gz_interfaces/srv/SpawnEntity  (add joint)
  /world/<world>/delete   ros_gz_interfaces/srv/DeleteEntity  (remove joint)

The "joint" is spawned as a tiny SDF model containing a single <joint>.
The model name is unique per attach, so we can remove it on detach.

Service interface (callable from the autopilot):
  ~/attach_cube   std_srvs/Trigger  - attach cube to rlink2 (gripper pad)
  ~/detach_cube   std_srvs/Trigger  - detach (release) cube
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import Bool
import time


class CubeAttachDetach(Node):
    def __init__(self):
        super().__init__('cube_attach_detach')

        # Match multi_robot_scene.world from the launch file
        self.world_name = 'multi_robot_scene'
        # The picker robot's rlink2 is the gripper pad (parent of joint)
        self.robot_ns = 'robot_1'
        self.cube_name = 'test_block'

        # Track the attach entity name
        self.attached_entity = None

        # Service servers
        self.attach_srv = self.create_service(
            Trigger, '~/attach_cube', self.attach_cb)
        self.detach_srv = self.create_service(
            Trigger, '~/detach_cube', self.detach_cb)

        # Publish attached state
        self.attached_pub = self.create_publisher(Bool, '~/attached', 10)

        # Clients
        self.spawn_cli = self.create_client(
            SpawnEntity, f'/world/{self.world_name}/create')
        self.delete_cli = self.create_client(
            DeleteEntity, f'/world/{self.world_name}/delete')

        self.get_logger().info(
            f'CubeAttachDetach ready. World={self.world_name} '
            f'joint: {self.robot_ns}/rlink2 <-> {self.cube_name}')

    def _build_sdf(self, joint_name):
        """Build a minimal SDF that creates a fixed joint between
        rlink2 (gripper pad) and the cube (test_block)."""
        return (
            f'<?xml version="1.0"?>'
            f'<sdf version="1.6">'
            f'  <model name="{joint_name}">'
            f'    <link name="attach_link"/>'
            f'    <joint name="attach_joint" type="fixed">'
            f'      <parent>{self.robot_ns}/rlink2</parent>'
            f'      <child>{self.cube_name}</child>'
            f'    </joint>'
            f'  </model>'
            f'</sdf>'
        )

    def attach_cb(self, req, resp):
        if self.attached_entity is not None:
            self.get_logger().info(
                f'Already attached as {self.attached_entity}')
            resp.success = True
            resp.message = f'already attached ({self.attached_entity})'
            return resp

        joint_name = f'attached_cube_{int(time.time() * 1000)}'
        sdf = self._build_sdf(joint_name)

        if not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SpawnEntity service not available')
            resp.success = False
            resp.message = 'SpawnEntity service not available'
            return resp

        req_msg = SpawnEntity.Request()
        req_msg.entity_factory.name = joint_name
        req_msg.entity_factory.sdf = sdf

        fut = self.spawn_cli.call_async(req_msg)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or not fut.result().success:
            err = fut.result() if fut.done() else 'timeout'
            self.get_logger().error(f'SpawnEntity failed: {err}')
            resp.success = False
            resp.message = f'spawn failed: {err}'
            return resp

        self.attached_entity = joint_name
        self.get_logger().info(
            f'Cube attached: {joint_name} (rlink2 <-> {self.cube_name})')
        resp.success = True
        resp.message = joint_name
        self.attached_pub.publish(Bool(data=True))
        return resp

    def detach_cb(self, req, resp):
        if self.attached_entity is None:
            resp.success = True
            resp.message = 'not attached'
            return resp

        if not self.delete_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('DeleteEntity service not available')
            resp.success = False
            resp.message = 'DeleteEntity service not available'
            return resp

        from ros_gz_interfaces.msg import Entity as GzEntity
        req_msg = DeleteEntity.Request()
        # Entity type 0 = MODEL (from ros_gz_interfaces.msg.Entity constants)
        # We use MODEL for the SDF model we created
        req_msg.entity.name = self.attached_entity
        req_msg.entity.type = 2  # MODEL = 2 in gz Entity_Type enum

        fut = self.delete_cli.call_async(req_msg)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done():
            self.get_logger().error('DeleteEntity timeout')
            resp.success = False
            resp.message = 'timeout'
            return resp
        result = fut.result()
        if result is None or not result.success:
            self.get_logger().error(f'DeleteEntity failed: {result}')
            resp.success = False
            resp.message = f'delete failed: {result}'
            return resp

        self.get_logger().info(
            f'Cube detached: {self.attached_entity} removed')
        self.attached_entity = None
        resp.success = True
        resp.message = 'detached'
        self.attached_pub.publish(Bool(data=False))
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = CubeAttachDetach()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
