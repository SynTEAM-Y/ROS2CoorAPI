#!/usr/bin/env python3
"""Minimal MoveGroup launch for x3plus — runs entirely from sim_gazebo_bringup.

Loads only the URDF, SRDF, kinematics, and joint limits.  No ros2_control,
no PILZ, no octomap, no controller manager — just arm planning.

Usage:
  ros2 launch sim_gazebo_bringup move_group.launch.py
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("sim_gazebo_bringup")
    cfg = os.path.join(pkg_dir, "config")
    urdf_file = os.path.join(pkg_dir, "urdf", "yahboomcar_X3plus.urdf.xacro")

    # ── URDF via xacro ────────────────────────────────────
    result = subprocess.run(
        ["xacro", urdf_file, "ns:="],
        capture_output=True, text=True, check=True,
    )
    urdf = result.stdout

    # Strip leading slashes that the empty ns:= expansion leaves on
    # name/parent/child/link/reference/joint attributes.  Without this
    # the URDF parser can't match arm_joint1 → arm_link1.
    import re
    for attr in ["name", "parent", "child", "link", "reference", "joint"]:
        urdf = re.sub(rf'{attr}="/', rf'{attr}="', urdf)

    # MoveIt Humble can segfault when parsing <dynamics> tags on planning-group
    # joints (damping/friction are Ignition-specific anyway). Strip them.
    urdf = re.sub(r'<dynamics[^/]*/>', '', urdf)

    # ── SRDF ──────────────────────────────────────────────
    with open(os.path.join(cfg, "yahboomcar_X3plus.srdf")) as f:
        srdf = f.read()

    # ── Kinematics ────────────────────────────────────────
    with open(os.path.join(cfg, "kinematics.yaml")) as f:
        kinematics = yaml.safe_load(f)

    # ── Joint limits (velocity / acceleration) ────────────
    with open(os.path.join(cfg, "joint_limits.yaml")) as f:
        joint_limits = yaml.safe_load(f)

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Publish /robot_description as a topic so MoveIt's planning scene
    # monitor and other nodes can retrieve it.
    robot_description_publisher_node = Node(
        package="sim_gazebo_bringup",
        executable="robot_description_publisher",
        name="robot_description_publisher",
        output="screen",
        parameters=[
            {"robot_description": urdf},
            {"use_sim_time": use_sim_time},
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {
                "robot_description": urdf,
                "robot_description_semantic": srdf,
                "robot_description_kinematics": kinematics,
                "robot_description_planning": joint_limits,
                "use_sim_time": use_sim_time,
                # Disable trajectory execution — pick_and_place.py
                # uses plan_only and sends trajectories directly to
                # trajectory_bridge.
                "allow_trajectory_execution": False,
                # Tell MoveGroup which pipelines exist
                "planning_pipelines": {
                    "pipeline_names": ["ompl"],
                },
                # Configure the OMPL pipeline namespace — parameters here
                # are read when pipeline_id="ompl" is requested.
                "ompl": {
                    "planning_plugin": "ompl_interface/OMPLPlanner",
                    "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
                    "start_state_max_bounds_error": 0.1,
                    "planner_configs": {
                        "RRTConnectkConfigDefault": {
                            "type": "geometric::RRTConnect",
                            "range": 0.0,
                        }
                    },
                    "default_planner_config": "RRTConnectkConfigDefault",
                },
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        robot_description_publisher_node,
        move_group_node,
    ])
