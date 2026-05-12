#!/usr/bin/env python3
"""
Launch file for RViz visualization of x3plus robot (headless - no GUI components).

This is a simplified version without the joint_state_publisher_gui broker,
but it still uses local package resources and does not depend on x3plus_examples.

Usage:
    ros2 launch sim_gazebo_bringup robot_rviz_headless.launch.py

Then separately, publish joint states:
    ros2 run joint_state_publisher joint_state_publisher

Or control the robot:
    ros2 run sim_gazebo_bringup manual_control
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package shares
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')

    # Paths
    xacro_file = os.path.join(sim_gazebo_bringup_dir, 'urdf', 'yahboomcar_X3plus.urdf.xacro')
    rviz_config_file = os.path.join(sim_gazebo_bringup_dir, 'rviz', 'gazebo_view.rviz')

    if not os.path.isfile(xacro_file):
        raise RuntimeError(f"Missing in-package URDF xacro: {xacro_file}")

    # Process XACRO using local package URDF
    try:
        result = subprocess.run(
            ['xacro', xacro_file, 'ns:='],
            capture_output=True,
            text=True,
            check=True,
        )
        robot_description_content = result.stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to process XACRO file with xacro: {e.stderr}")
    except FileNotFoundError:
        raise RuntimeError(
            "xacro command not found. Please install: sudo apt install ros-humble-xacro"
        )

    robot_description_content = robot_description_content.replace(
        'package://yahboomcar_description/meshes/',
        'package://sim_gazebo_bringup/meshes/'
    )

    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    robot_description_publisher_node = Node(
        package='sim_gazebo_bringup',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    
    # Joint State Publisher node (headless, no GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        robot_description_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
