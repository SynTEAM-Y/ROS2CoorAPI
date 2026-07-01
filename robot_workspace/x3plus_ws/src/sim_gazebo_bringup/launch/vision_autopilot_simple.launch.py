#!/usr/bin/env python3
"""
Simplified Vision Autopilot Launch File
========================================

Launches:
  - Gazebo simulation with the x3plus robot (no Nav2, no RViz)
  - The blue test cube (spawned at (2.0, 0.0, 0.03))
  - The green landing pad (spawned at (2.0, 1.2, 0.001))
  - Two gazebo_pose_tf_relay nodes that re-publish the Gazebo PosePublisher
    ground-truth poses as 'odom -> test_block' and 'odom -> landing_pad' on /tf
  - The vision_autopilot_simple state machine

The cube and pad positions are obtained from Gazebo ground truth (PosePublisher
plugin on each model SDF), not from the camera. The wrist camera (HSV blob)
is only used for the final ~0.3 m of the pick. To port this to a real robot,
replace the two gazebo_pose_tf_relay nodes with a perception node that
publishes the same 'odom -> test_block' and 'odom -> landing_pad' TFs from
camera + fiducials (e.g. AprilTag).

This is the manufacturer-style simple autopilot. The full system (Nav2 +
MoveIt + object_detector) is in vision_autopilot.launch.py.

Usage:
  ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py
  ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=office
"""

import os
import sys

os.environ['SIM_GAZEBO_BRINGUP_NO_PROMPT'] = '1'
os.environ['SIM_GAZEBO_BRINGUP_WORLD'] = 'office'
os.environ['SIM_GAZEBO_BRINGUP_MAP'] = 'plain_map'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _get_cli_arg(name, default):
    prefix = f'{name}:='
    for a in sys.argv[1:]:
        if a.startswith(prefix):
            return a.split(':=', 1)[1]
    return default


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    
    world_str = _get_cli_arg('world', 'office')
    map_str = _get_cli_arg('map', 'plain_map')
    
    world_arg = DeclareLaunchArgument('world', default_value='office',
        description='Gazebo world')
    map_arg = DeclareLaunchArgument('map', default_value='plain_map',
        description='Map for navigation')
    
    # Plain Gazebo (no Nav2): the autopilot drives via odom TF + /cmd_vel
    # directly. Nav2 would add competing /cmd_vel publishers.
    gazebo_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_gazebo_bringup_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_str, 'use_rviz': 'false'}.items()
    )
    
    # Spawn the blue test cube at (2.0, 0.0)
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
    spawn_delayed = TimerAction(period=15.0, actions=[spawn_test_object_node])
    
    # Spawn green landing pad at (2.0, 1.2)
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
    spawn_landing_pad_delayed = TimerAction(period=17.0, actions=[spawn_landing_pad_node])
    
    # The world pose stream (ground truth for ALL entities) is already bridged
    # to /gz_pose_tf by gazebo.launch.py. The relays below filter by child name.
    
    # TF relay for test_block
    test_block_relay = Node(
        package='sim_gazebo_bringup',
        executable='gazebo_pose_tf_relay',
        name='test_block_tf_relay',
        parameters=[
            {'use_sim_time': True},
            {'parent_frame': 'odom'},
            {'child_frame': 'test_block'},
            {'input_topic': '/gz_pose_tf'},
            {'source_child': 'test_block'},
        ],
    )
    
    # TF relay for landing_pad
    landing_pad_relay = Node(
        package='sim_gazebo_bringup',
        executable='gazebo_pose_tf_relay',
        name='landing_pad_tf_relay',
        parameters=[
            {'use_sim_time': True},
            {'parent_frame': 'odom'},
            {'child_frame': 'landing_pad'},
            {'input_topic': '/gz_pose_tf'},
            {'source_child': 'landing_pad'},
        ],
    )
    
    test_block_tf_delayed = TimerAction(period=16.0, actions=[test_block_relay])
    landing_pad_tf_delayed = TimerAction(period=18.0, actions=[landing_pad_relay])
    
    # Vision autopilot node (delayed to allow Gazebo to start).
    # standoff_distance = 0.292 m = gripper finger-center forward reach at
    # REACH_DOWN pose (FK-verified). drop_off_standoff_distance is the same
    # by default — both use the gripper reach. hsv_stop_y = 410 is calibrated
    # for the 2 cm cube (the SDF is 0.02 m, see models/test_block/model.sdf);
    # the 4 cm cube used 440, which is past the 2 cm blob's reach in the
    # image and the HSV loop would never converge.
    vision_autopilot_node = Node(
        package='sim_gazebo_bringup',
        executable='vision_autopilot_simple',
        parameters=[
            {'standoff_distance': 0.292},
            {'drop_off_standoff_distance': 0.292},
            {'approach_speed': 0.80},       # very fast driving
            {'max_linear_speed': 0.90},
            {'max_angular_speed': 1.5},
            {'pre_approach_distance': 0.65},
            {'hsv_stop_y': 410},
            {'hsv_x_tol': 10},
        ],
        output='screen',
    )
    
    # Delay autopilot start by 25 seconds
    vision_autopilot_delayed = TimerAction(period=25.0, actions=[vision_autopilot_node])
    
    return LaunchDescription([
        world_arg,
        map_arg,
        gazebo_nav2_launch,
        spawn_delayed,
        spawn_landing_pad_delayed,
        test_block_tf_delayed,
        landing_pad_tf_delayed,
        vision_autopilot_delayed,
    ])
