#!/usr/bin/env python3
"""Full autonomous pick-and-place demo: Gazebo + Nav2 + MoveIt + perception.

Usage:
  ros2 launch sim_gazebo_bringup pick_and_place.launch.py
  ros2 launch sim_gazebo_bringup pick_and_place.launch.py world:=office map:=plain_map

This launch file automatically:
  1. Starts Gazebo with the office world + Nav2 + MoveIt
  2. Spawns a blue test cube at (2.0, 0.0) — far from robot
  3. Starts object detection (mono + depth camera)
  4. Waits 25 s for everything to initialise, then starts pick_and_place

The robot self-drives to the cube using lidar/Nav2, detects it with
 cameras, picks it, drives to the green landing pad at (2.0, 1.2)
 in front of the static wall, and places the blue cube there.
"""

import os
import sys

# CRITICAL: Disable the interactive picker in gazebo.launch.py BEFORE any
# imports that might trigger it.  When gazebo.launch.py is included as a
# sub-launch, its generate_launch_description() runs in the same process
# and the picker fires if these env vars are not set.
# We use DIRECT ASSIGNMENT (not setdefault) to override any existing values.
os.environ['SIM_GAZEBO_BRINGUP_NO_PROMPT'] = '1'
os.environ['SIM_GAZEBO_BRINGUP_WORLD'] = 'office'
os.environ['SIM_GAZEBO_BRINGUP_MAP'] = 'plain_map'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _get_cli_arg(name, default):
    """Read a command-line argument like 'world:=office'.

    ROS2 launch passes arguments via sys.argv as 'name:=value' strings.
    We parse them directly so we can use concrete Python strings in the
    launch description instead of LaunchConfiguration substitutions,
    which avoids the substitution-chain bugs that caused the wrong world
    to load.
    """
    prefix = f'{name}:='
    for a in sys.argv[1:]:
        if a.startswith(prefix):
            return a.split(':=', 1)[1]
    return default


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')

    # Parse command-line arguments into concrete Python strings.
    # DO NOT use LaunchConfiguration for values passed to sub-launches —
    # the substitution chain can resolve to wrong defaults.
    world_str = _get_cli_arg('world', 'office')
    map_str = _get_cli_arg('map', 'plain_map')

    # Keep DeclareLaunchArgument for documentation and ROS2 arg introspection,
    # but use the concrete strings above for the actual include.
    world_arg = DeclareLaunchArgument(
        'world', default_value='office',
        description='Gazebo world (office required for pick-and-place; contains the static wall at 2,2)')
    map_arg = DeclareLaunchArgument(
        'map', default_value='plain_map',
        description='Map for Nav2 navigation')

    # Gazebo + Nav2.  The pick_and_place node drives via direct cmd_vel
    # (no NavigateToPose goals), but we keep Nav2 running so AMCL and
    # trajectory_bridge stay alive for arm/gripper controllers.
    gazebo_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_gazebo_bringup_dir, 'launch', 'gazebo_nav2.launch.py')
        ),
        launch_arguments={
            'world': world_str,
            'map': map_str,
        }.items()
    )

    # MoveIt move_group (Phase III) — DISABLED for pick-and-place.
    # The pick_and_place node uses FollowJointTrajectory action servers
    # directly, so MoveIt is not required.  Keeping move_group running
    # alongside Nav2 + Gazebo overloads the system and causes Nav2
    # initialization to hang / TF buffer to be cleared repeatedly.
    # move_group_launch = IncludeLaunchDescription(...)

    # Static map->odom transform.  AMCL normally publishes this after
    # localising, but the Gazebo LiDAR is not publishing /scan in this
    # Fortress setup, so AMCL never localises.  We publish a static
    # identity transform (robot spawns at map origin) so Nav2 has a
    # valid TF tree.  Uses /tf_static so it does NOT conflict with AMCL
    # (which publishes on /tf).
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    # Object detector (Phase II)
    object_detector_node = Node(
        package='sim_gazebo_bringup',
        executable='object_detector',
        name='object_detector',
        output='screen',
    )

    # Spawn the blue test cube — 2 m in front of the robot so it has to
    # self-drive.  Delayed 15 s so Gazebo is fully loaded with the correct world.
    spawn_test_object_node = Node(
        package='sim_gazebo_bringup',
        executable='spawn_test_object',
        name='spawn_test_object',
        output='screen',
        parameters=[{
            'x': 2.0,
            'y': 0.0,
            'z': 0.03,
            'world': world_str,
        }],
    )
    spawn_delayed = TimerAction(period=20.0, actions=[spawn_test_object_node])

    # Landing pad — green square on the floor in front of the wall.
    # The robot navigates here (drop-off zone) and places the blue cube
    # against the wall.  Coordinates (2.0, 1.2) place it just south of
    # the wall at (2, 2).
    spawn_landing_pad_node = Node(
        package='sim_gazebo_bringup',
        executable='spawn_test_object',
        name='spawn_landing_pad',
        output='screen',
        parameters=[{
            'x': 2.0,
            'y': 1.2,
            'z': 0.001,
            'world': world_str,
            'name': 'landing_pad',
            'model_path': os.path.join(sim_gazebo_bringup_dir, 'models', 'landing_pad', 'model.sdf'),
        }],
    )
    spawn_landing_pad_delayed = TimerAction(period=22.0, actions=[spawn_landing_pad_node])

    # Pick-and-place orchestrator (Phase III)
    # NOTE: object_navigator is intentionally NOT launched here because
    # pick_and_place already handles its own navigation to the object and
    # to the drop-off zone.  Running both would create conflicting Nav2 goals.
    # Delayed 25 s so Gazebo spawns the robot (5 s), Nav2 fully initialises
    # (15-20 s), and TF is available before pick_and_place starts.
    pick_and_place_node = Node(
        package='sim_gazebo_bringup',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            # Sensor-guided object approach.
            # use_fixed_object=false lets the camera refine the target pose.
            # If no detection is available at start, the fixed coords are used
            # as fallback until the camera sees the object.
            {'use_fixed_object': False},
            {'object_x': 2.0},
            {'object_y': 0.0},
            {'object_z': 0.03},
            # Drop-off at the green landing pad in front of the static wall.
            # Landing pad is at (2.0, 1.2) — south of the wall at (2, 2).
            # Robot parks facing north, arm extends to place the blue cube
            # onto the pad against the wall.  See pick_and_place.py docstring.
            {'drop_off_x': 2.0},
            {'drop_off_y': 1.2},
            {'drop_off_yaw': 0.0},
        ],
    )
    pick_and_place_delayed = TimerAction(period=25.0, actions=[pick_and_place_node])

    return LaunchDescription([
        world_arg, map_arg,
        gazebo_nav2_launch,
        static_map_to_odom,
        # pick_and_place drives via direct cmd_vel, not Nav2 goals
        object_detector_node,
        spawn_delayed,
        spawn_landing_pad_delayed,
        pick_and_place_delayed,
    ])
