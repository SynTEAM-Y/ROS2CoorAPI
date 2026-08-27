#!/usr/bin/env python3
"""
Gripper Cube Attach/Detach Service
==================================
Glues the test_block cube to robot_1's gripper pad during carry.

WHY this is not a joint: gz-sim's /world/<world>/create service only accepts
a single top-level <model>/<light>/<actor>, and SDF joints can only connect
links *inside the same model*.  Two separate top-level models (a robot and a
free cube) cannot be rigidly joined at runtime, so the old fixed-joint attach
always failed and robot_1 could never lift the cube.

Instead, while "attached" (armed) we continuously re-set the cube's world
pose to follow the gripper pad (robot_1/rlink2) using the world set_pose
service once the pad is actually at the cube.  The pad-relative offset is
captured the moment glue engages so the cube does not pop or snap while
the arm is still descending.  On detach the glue loop stops and gravity
takes over.

Service interface (callable from the autopilot):
  ~/attach_cube   std_srvs/Trigger  - glue cube to rlink2 (gripper pad)
  ~/detach_cube   std_srvs/Trigger  - release cube (stop glue loop)
  ~/attached      std_msgs/Bool     - current glued state
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Quaternion, Point
import tf2_ros

GLUE_HZ = 50.0       # pose-follow rate; high enough that gravity sag
                     # between set_pose ticks (~0.5 mm) is invisible
GRIP_FOLLOW_Z_TOL = 0.05  # m; fast path: engage when the pad is vertically
                          # AT the cube (REACH_DOWN fully converged).
GRIP_ENGAGE_Z_MAX = 0.25  # m; fallback path: engage when the pad bottoms out
                          # and starts rising (trough->rise), so the glue
                          # works even if the arm never fully converges to
                          # REACH_DOWN (slow sim RTF).  Only accepted if the
                          # trough is low enough to actually be over the cube.
GRIP_RISE_DELTA = 0.006   # m; pad must rise this much above its running
                          # minimum before we call the descent "done".
SET_POSE_TIMEOUT = 0.5    # s; max time a single set_pose request may stay
                          # in flight before we drop it and resume the loop.
TF_TIMEOUT = 2.0     # seconds to wait for a TF lookup


def _inv_quat(q):
    """Inverse of a unit quaternion (conjugate for unit quats)."""
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _quat_mult(a, b):
    """Hamilton product a * b.  Returns geometry_msgs/Quaternion."""
    w, x, y, z = a.w, a.x, a.y, a.z
    w2, x2, y2, z2 = b.w, b.x, b.y, b.z
    return Quaternion(
        x=w*x2 + x*w2 + y*z2 - z*y2,
        y=w*y2 - x*z2 + y*w2 + z*x2,
        z=w*z2 + x*y2 - y*x2 + z*w2,
        w=w*w2 - x*x2 - y*y2 - z*z2,
    )


def _rot_vec(x, y, z, q):
    """Rotate vector (x,y,z) by unit quaternion q (returns a 3-tuple)."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    r00 = 1 - 2*(qy*qy + qz*qz)
    r01 = 2*(qx*qy - qw*qz)
    r02 = 2*(qx*qz + qw*qy)
    r10 = 2*(qx*qy + qw*qz)
    r11 = 1 - 2*(qx*qx + qz*qz)
    r12 = 2*(qy*qz - qw*qx)
    r20 = 2*(qx*qz - qw*qy)
    r21 = 2*(qy*qz + qw*qx)
    r22 = 1 - 2*(qx*qx + qy*qy)
    return (
        r00*x + r01*y + r02*z,
        r10*x + r11*y + r12*z,
        r20*x + r21*y + r22*z,
    )


def _trans_to_pose(tr):
    """Convert a geometry_msgs/TransformStamped into a geometry_msgs/Pose.

    `translation` is a Vector3; Pose.position is a Point — copy fields.
    """
    p = Pose()
    p.position.x = tr.transform.translation.x
    p.position.y = tr.transform.translation.y
    p.position.z = tr.transform.translation.z
    p.orientation = tr.transform.rotation
    return p


def _rel_to_base(pose, base):
    """Return pose expressed relative to base (i.e. base^-1 * pose)."""
    rel_q = _quat_mult(_inv_quat(base.orientation), pose.orientation)
    dx = pose.position.x - base.position.x
    dy = pose.position.y - base.position.y
    dz = pose.position.z - base.position.z
    v = _rot_vec(dx, dy, dz, _inv_quat(base.orientation))
    out = Pose()
    out.position = Point(x=v[0], y=v[1], z=v[2])
    out.orientation = rel_q
    return out


def _compose(base, offset):
    """World pose = base * offset (both geometry_msgs/Pose)."""
    out = Pose()
    v = _rot_vec(offset.position.x, offset.position.y, offset.position.z,
                 base.orientation)
    out.position = Point(
        x=base.position.x + v[0],
        y=base.position.y + v[1],
        z=base.position.z + v[2])
    out.orientation = _quat_mult(base.orientation, offset.orientation)
    return out


class CubeAttachDetach(Node):
    def __init__(self):
        super().__init__('cube_attach_detach')

        # Match multi_robot_scene.world from the launch file.
        self.world_name = 'multi_robot_scene'
        # The picker robot's wrist link — the pose-follow target.
        # Was robot_1_rlink2 (the finger pad), but teleporting the cube onto
        # the 1-gram pad link at 50 Hz physically hammers rlink_joint2 and
        # kicks it against its limits (verified at runtime: rlink_joint2
        # pegged at ±1.8 rad while every other mimic joint tracked fine).
        # arm_link5 is stiff and steady, so the glue is invisible to the
        # finger PID loops. The offset is captured at attach time, so the
        # cube still rides visually between the pads.
        self.pad_link = 'robot_1_arm_link5'   # namespaced URDF frame
        self.cube_name = 'test_block'

        # Glue state
        self._glued = False
        self._offset = None        # cube pose relative to pad at capture
        self._pending_fut = None   # in-flight set_pose request (max 1)
        self._pending_sent = None  # rclpy time when the request was sent
        self._min_pad_z = None     # running min pad z during current descent
        self._was_descending = False

        # Service servers
        self.attach_srv = self.create_service(
            Trigger, '~/attach_cube', self.attach_cb)
        self.detach_srv = self.create_service(
            Trigger, '~/detach_cube', self.detach_cb)
        self.attached_pub = self.create_publisher(Bool, '~/attached', 10)

        # TF buffer: need odom -> pad_link and odom -> cube to compute the
        # pad-relative offset at attach time.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Client for the world set_pose service.
        self.set_pose_cli = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')

        # Periodic glue loop while attached.
        self.create_timer(1.0 / GLUE_HZ, self._glue_tick)

        self.get_logger().info(
            f'CubeAttachDetach ready. World={self.world_name} '
            f'glue target: {self.pad_link} <-> {self.cube_name}')

    # ------------------------------------------------------------------
    def _lookup(self, target, parent='odom'):
        """Return world pose (geometry_msgs/Pose) for a TF lookup or None."""
        try:
            tr = self.tf_buffer.lookup_transform(
                parent, target, rclpy.time.Time(),
                timeout=Duration(seconds=TF_TIMEOUT))
            return _trans_to_pose(tr)
        except Exception:
            return None

    def _glue_tick(self):
        if not self._glued:
            self._pending_fut = None
            return
        pad = self._lookup(self.pad_link)
        if pad is None:
            return
        cube = self._lookup(self.cube_name)
        if cube is None:
            return
        if self._offset is None:
            # Engage the glue when the pad is AT the cube.  Two paths:
            #  1. Fast path: pad z within GRIP_FOLLOW_Z_TOL of cube z —
            #     the arm fully converged to REACH_DOWN.
            #  2. Fallback: pad bottoms out then starts rising (trough->
            #     rise).  On this system the sim runs at ~40% RTF and the
            #     arm sometimes never reaches REACH_DOWN, so the pad never
            #     gets within the z window — wait for the lift instead.
            #     Only accepted when the pad is horizontally over the cube
            #     and its trough is low enough to be over the cube.
            z = pad.position.z
            if self._min_pad_z is None:
                self._min_pad_z = z
                self._was_descending = False
            if z < self._min_pad_z - GRIP_RISE_DELTA:
                self._min_pad_z = z
                self._was_descending = True
            elif z > self._min_pad_z + GRIP_RISE_DELTA:
                if self._was_descending:
                    self._was_descending = False   # rise confirmed
            w_in = abs(z - cube.position.z) <= GRIP_FOLLOW_Z_TOL
            # Tight XY gate: the pads must be AT the cube (7 cm), not merely
            # near it.  The old 20 cm gate let the glue engage while the
            # robot was still approaching, so the cube hung in mid-air
            # beside the gripper and the robot never actually reached it.
            over_cube = (abs(pad.position.x - cube.position.x) < 0.07 and
                          abs(pad.position.y - cube.position.y) < 0.07)
            trough_done = (self._min_pad_z is not None and
                            self._min_pad_z <= GRIP_ENGAGE_Z_MAX and
                            z > self._min_pad_z + GRIP_RISE_DELTA and
                            not self._was_descending)
            # BOTH paths require over_cube: engaging with only the z-window
            # true glued the cube from 1.6 m away (offset z=1.643, cube
            # later teleported through the floor).  XY proximity is
            # mandatory.
            if not over_cube or not (w_in or trough_done):
                return
            # World-XY sanity (a stale/bogus engage once pinned the cube
            # 0.49 m to the side; detaching there ejected it 78 m).
            wdx = cube.position.x - pad.position.x
            wdy = cube.position.y - pad.position.y
            if abs(wdx) > 0.10 or abs(wdy) > 0.10:
                self.get_logger().error(
                    f'Glue offset insane world({wdx:.2f},{wdy:.2f}) — '
                    f'refusing to engage')
                return
            # SNAP the cube INTO the gripper rather than keeping the
            # arbitrary captured offset (which left it visibly floating
            # beside the pads).  The grip geometry (from the autopilot's
            # FK calibration) is: cube centre = wrist + 0.019 m along the
            # chassis forward axis + 0.019 m up — exactly between the
            # pads.  Teleport the cube there now so the visual matches a
            # real grasp from the first frame.
            base = self._lookup('robot_1_base_footprint')
            if base is not None:
                q = base.orientation
                byaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                  1.0 - 2.0*(q.y*q.y + q.z*q.z))
                desired = Pose()
                desired.position = Point(
                    x=pad.position.x + 0.019*math.cos(byaw),
                    y=pad.position.y + 0.019*math.sin(byaw),
                    z=pad.position.z + 0.019)
                desired.orientation = cube.orientation
                self._offset = _rel_to_base(desired, pad)
                self._call_set_pose(desired)
            else:
                self._offset = _rel_to_base(cube, pad)
            self.get_logger().info(
                f'Glue engaged (pad z={pad.position.z:.3f}, '
                f'snapped into gripper, '
                f'path={"z-window" if w_in else "trough-rise"})')
        if self._pending_fut is not None:
            if not self._pending_fut.done():
                # Watchdog: if a set_pose request has been in flight for
                # too long (lost response / bridge hiccup), drop it and
                # resume the loop instead of freezing the cube mid-air.
                dt = (self.get_clock().now() -
                      self._pending_sent).nanoseconds / 1e9
                if dt < SET_POSE_TIMEOUT:
                    return
                self.get_logger().warning(
                    f'glue: set_pose response stuck {dt:.1f}s; dropping')
                self._pending_fut = None
                return
            self._pending_fut = None
        ok = self._call_set_pose(_compose(pad, self._offset))
        self._tick_count = getattr(self, '_tick_count', 0) + 1
        if self._tick_count % 25 == 0:
            self.get_logger().info(
                f'glue: pad=({pad.position.x:.2f},{pad.position.y:.2f},'
                f'{pad.position.z:.3f}) cube=({cube.position.x:.2f},'
                f'{cube.position.y:.2f},{cube.position.z:.3f}) '
                f'set_pose={"ok" if ok else "SKIP"}')

    def _call_set_pose(self, pose):
        """Fire the world set_pose request asynchronously (non-blocking)."""
        if not self.set_pose_cli.service_is_ready():
            return False
        req = SetEntityPose.Request()
        req.entity.name = self.cube_name
        req.entity.type = 2  # MODEL
        req.pose = pose
        self._pending_fut = self.set_pose_cli.call_async(req)
        self._pending_sent = self.get_clock().now()
        return True

    # ------------------------------------------------------------------
    def attach_cb(self, req, resp):
        if self._glued:
            resp.success = True
            resp.message = 'already glued'
            return resp

        self._glued = True
        self._offset = None        # captured lazily by the glue loop once the
                                   # pad is at the cube
        self._min_pad_z = None     # reset trough tracking
        self._was_descending = False
        self._pending_fut = None
        self._pending_sent = None
        self.attached_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'Attach requested: will glue {self.cube_name} -> '
            f'{self.pad_link}')
        resp.success = True
        resp.message = 'glue armed'
        return resp

    def detach_cb(self, req, resp):
        if not self._glued:
            resp.success = True
            resp.message = 'not attached'
            return resp
        # Final placement before unglue: teleport the cube to its natural
        # RESTING pose.  A cube released from a pinned mid-air pose gets
        # ejected by the contact solver (measured: up to 78 m).  The basin
        # interior floor is at world z ~ 0.01-0.03 (cube measured resting
        # there at z=0.032 in a successful run); z=0.032 sits the cube in
        # the basin with no drop and no interpenetration.
        cube = self._lookup(self.cube_name)
        if cube is not None:
            final = Pose()
            final.position = Point(x=cube.position.x,
                                   y=cube.position.y,
                                   z=0.032)
            final.orientation = cube.orientation
            self._call_set_pose(final)
        self._glued = False
        self._offset = None
        self._min_pad_z = None
        self._was_descending = False
        self.attached_pub.publish(Bool(data=False))
        self.get_logger().info('Cube released at basin resting height.')
        resp.success = True
        resp.message = 'released'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = CubeAttachDetach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
