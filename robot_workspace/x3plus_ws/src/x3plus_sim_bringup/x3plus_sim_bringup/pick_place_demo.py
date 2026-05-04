#!/usr/bin/env python3
# RViz-only pick & place using MoveIt services (Humble).
# Uses /apply_planning_scene, /compute_ik, /plan_kinematic_path, /execute_kinematic_path.

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    PlanningScene, CollisionObject, AttachedCollisionObject,
    Constraints, JointConstraint, RobotState
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetPositionIK, GetMotionPlan
)

# ---- tweak to match your setup ----
ARM_GROUP      = "arm_group"      # Name of the MoveIt planning group (SRDF)
EEF_LINK       = "arm_link5"      # End-effector link used for IK and planning
PLANNING_FRAME = "base_link"      # Global planning frame (e.g., base_link or map)
ARM_JOINTS: List[str] = [         # Ordered joint names belonging to ARM_GROUP (from SRDF)
    "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"
]
TABLE_SIZE = (0.6, 0.8, 0.04)     # Table size (x, y, z) in meters
CUBE_SIZE  = 0.05                 # Cube edge length in meters
# -----------------------------------

def quat_from_rpy(r, p, y):
    """Convert roll-pitch-yaw (radians) to quaternion (x, y, z, w)."""
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*cp*sy)

def pose(frame, x,y,z, r,p,yaw):
    """Helper to build a PoseStamped in `frame` at (x,y,z) with RPY orientation."""
    qx,qy,qz,qw = quat_from_rpy(r,p,yaw)
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x,y,z
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx,qy,qz,qw
    return ps

class PickPlace(Node):
    """Minimal MoveIt services-based pick-and-place pipeline.

    Key services/actions:
      - /apply_planning_scene (ApplyPlanningScene): add/move/remove collision objects.
      - /compute_ik (GetPositionIK): cartesian pose → joint solution (RobotState).
      - /plan_kinematic_path (GetMotionPlan): joint-space plan to goal constraints.
      - /execute_trajectory (ExecuteTrajectory action): execute a previously planned path.

    Assumptions:
      - MoveIt is already running and has a valid SRDF/Kinematics config.
      - TF tree includes PLANNING_FRAME and EEF_LINK.
      - Joint names in ARM_JOINTS exactly match the planning group order.
    """

    def __init__(self):
        super().__init__("pick_place_services")

        # Create service clients used in the sequence
        self.apply_scene   = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.compute_ik    = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_kin_path = self.create_client(GetMotionPlan, "/plan_kinematic_path")

        # Action client for executing trajectories planned via GetMotionPlan
        self.exec_traj_ac  = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        # Ensure servers are available before proceeding (fail fast if not)
        for cli, name in [
            (self.apply_scene, "apply_planning_scene"),
            (self.compute_ik, "compute_ik"),
            (self.plan_kin_path, "plan_kinematic_path"),
        ]:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"Service /{name} not available")

        if not self.exec_traj_ac.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Action /execute_trajectory not available")


    def apply_objects(self):
        """Add a table and a cube into the world as collision objects.
        Returns:
            float: cube top height (z) in planning frame for convenience.
        """
        scene = PlanningScene(is_diff=True)

        # Add table as a box primitive, centered at (0.6, 0, z/2) in PLANNING_FRAME
        table = CollisionObject()
        table.id = "table"; table.header.frame_id = PLANNING_FRAME
        prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX; prim.dimensions = [*TABLE_SIZE]
        table.primitives = [prim]
        table.primitive_poses = [pose(PLANNING_FRAME, 0.6, 0.0, TABLE_SIZE[2]/2.0, 0,0,0).pose]
        table.operation = CollisionObject.ADD

        # Add cube on top of the table
        cube = CollisionObject()
        cube.id = "cube"; cube.header.frame_id = PLANNING_FRAME
        prim2 = SolidPrimitive(); prim2.type = SolidPrimitive.BOX; prim2.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]
        cube_z = TABLE_SIZE[2] + CUBE_SIZE/2.0
        cube.primitives = [prim2]
        cube.primitive_poses = [pose(PLANNING_FRAME, 0.6, 0.0, cube_z, 0,0,0).pose]
        cube.operation = CollisionObject.ADD

        scene.world.collision_objects = [table, cube]
        self.apply_scene.call(ApplyPlanningScene.Request(scene=scene))
        time.sleep(0.2)  # small settle time to let RViz/PlanningScene sync
        return cube_z

    def ik(self, target_pose: PoseStamped):
        """Compute IK for the end effector to reach `target_pose`.
        Raises:
            RuntimeError: if IK fails or service returns an error.
        Returns:
            RobotState: joints solution for the planning group.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = ARM_GROUP
        req.ik_request.ik_link_name = EEF_LINK
        req.ik_request.pose_stamped = target_pose
        req.ik_request.attempts = 8
        req.ik_request.timeout.sec = 2
        res = self.compute_ik.call(req)
        if not res or res.error_code.val != res.error_code.SUCCESS:
            raise RuntimeError(f"IK failed: {res.error_code.val if res else 'no response'}")
        return res.solution  # RobotState

    def plan_to_state(self, goal_state: RobotState):
        """Plan a path to a desired joint configuration.
        Converts RobotState → JointConstraints and requests a motion plan.
        Raises:
            RuntimeError: if planning fails.
        Returns:
            RobotTrajectory: time-parameterized trajectory ready for execution.
        """
        # Map the goal joints into name→position for convenience
        joint_positions = dict(zip(goal_state.joint_state.name, goal_state.joint_state.position))

        # Build a tight (±1e-3) joint constraint set for every joint in the group
        jc = []
        for j in ARM_JOINTS:
            c = JointConstraint()
            c.joint_name = j
            c.position = float(joint_positions.get(j, 0.0))
            c.tolerance_above = 1e-3
            c.tolerance_below = 1e-3
            c.weight = 1.0
            jc.append(c)

        # Construct motion plan request (joint-space goal constraints)
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = ARM_GROUP
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.goal_constraints = [Constraints(joint_constraints=jc)]

        res = self.plan_kin_path.call(req)
        if not res or res.motion_plan_response.error_code.val != res.motion_plan_response.error_code.SUCCESS:
            raise RuntimeError("Planning failed")
        return res.motion_plan_response.trajectory

    def execute(self, traj):
        """Execute a pre-computed trajectory via the ExecuteTrajectory action.
        Raises:
            RuntimeError: if goal is rejected or execution fails.
        """
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        # Send goal and wait for acceptance
        send_future = self.exec_traj_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Execution goal rejected")

        # Wait for result from the controller
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        # Check error code (SUCCESS on success)
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(f"Execution failed (code {result.error_code.val})")

    def attach_cube(self):
        """Attach the cube to the EEF (treat it as grasped) in the PlanningScene."""
        scene = PlanningScene(is_diff=True)
        at = AttachedCollisionObject()
        at.link_name = EEF_LINK
        at.object = CollisionObject()
        at.object.id = "cube"
        at.object.operation = CollisionObject.MOVE  # move from world → attached
        scene.robot_state.attached_collision_objects = [at]
        self.apply_scene.call(ApplyPlanningScene.Request(scene=scene))

    def detach_cube(self):
        """Detach the cube from the EEF (release) in the PlanningScene."""
        scene = PlanningScene(is_diff=True)
        at = AttachedCollisionObject()
        at.link_name = EEF_LINK
        at.object = CollisionObject()
        at.object.id = "cube"
        at.object.operation = CollisionObject.REMOVE  # remove from attached list
        scene.robot_state.attached_collision_objects = [at]
        self.apply_scene.call(ApplyPlanningScene.Request(scene=scene))

def main():
    """Entry point that sequences a simple pick-and-place demo."""
    rclpy.init()
    node = PickPlace()

    # Stage the scene and remember the cube height for waypoints
    cube_z = node.apply_objects()

    def go_xyzrpy(x,y,z, r,p,yaw):
        """Convenience: IK → plan → execute for a single Cartesian waypoint."""
        tgt = pose(PLANNING_FRAME, x,y,z, r,p,yaw)
        goal_state = node.ik(tgt)
        traj = node.plan_to_state(goal_state)
        node.execute(traj)

    # Pregrasp above cube (flip tool about Y by π to keep z-axis pointing down)
    go_xyzrpy(0.6, 0.0, cube_z + 0.15, 0, math.pi, 0)
    # Approach to grasp height
    go_xyzrpy(0.6, 0.0, cube_z + 0.02, 0, math.pi, 0)

    # Gripper actuation is external; we mark the cube as attached in the scene
    node.attach_cube()

    # Retreat, translate to drop pose, descend, then release
    go_xyzrpy(0.6, 0.0, cube_z + 0.20, 0, math.pi, 0)
    go_xyzrpy(0.4, -0.25, cube_z + 0.20, 0, math.pi, -math.pi/2)
    go_xyzrpy(0.4, -0.25, cube_z + 0.02, 0, math.pi, -math.pi/2)

    node.detach_cube()
    go_xyzrpy(0.4, -0.25, cube_z + 0.20, 0, math.pi, -math.pi/2)

    print("Pick-and-place complete.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
