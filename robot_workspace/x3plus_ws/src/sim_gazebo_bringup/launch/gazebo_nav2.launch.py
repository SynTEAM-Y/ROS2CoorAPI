#!/usr/bin/env python3
"""Launch Gazebo simulation with x3plus and Nav2 navigation stack."""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_map(sim_gazebo_bringup_dir):
    """Resolve the map argument to an absolute path."""
    for arg in sys.argv[1:]:
        if arg.startswith('map:='):
            val = arg.split(':=', 1)[1]
            if os.path.isabs(val) and os.path.isfile(val):
                return val
            candidate = os.path.join(sim_gazebo_bringup_dir, 'maps', val + '.yaml')
            if os.path.isfile(candidate):
                return candidate
            return val
    return os.path.join(sim_gazebo_bringup_dir, 'maps', 'plain_map.yaml')


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file_path = _resolve_map(sim_gazebo_bringup_dir)

    world_arg = DeclareLaunchArgument(
        'world', default_value='office',
        description='Gazebo world to load (basename or absolute path).'
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value='plain_map',
        description='Map file for Nav2 (basename in maps/ or absolute path).'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_gazebo_bringup_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world, 'use_sim_time': use_sim_time, 'use_rviz': 'false',
            'use_nav2': 'true',  # Disable static map→odom — AMCL handles it
        }.items()
    )

    set_world_env = SetEnvironmentVariable('SIM_GAZEBO_BRINGUP_WORLD', world)
    set_map_env = SetEnvironmentVariable('SIM_GAZEBO_BRINGUP_MAP', LaunchConfiguration('map'))

    # Explicitly pass the BT XML so nav2_bringup doesn't fall back to the
    # through-poses variant (which references RemovePassedGoals, unavailable
    # on some Humble patch versions).
    nav2_bt_xml = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': os.path.join(sim_gazebo_bringup_dir, 'config', 'nav2_params.yaml'),
            'bt_xml_file': nav2_bt_xml,
        }.items()
    )

    trajectory_bridge_node = Node(
        package='sim_gazebo_bringup',
        executable='trajectory_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        world_arg, map_arg, use_sim_time_arg,
        set_world_env, set_map_env,
        gazebo_launch, nav2_launch,
        trajectory_bridge_node,
    ])
