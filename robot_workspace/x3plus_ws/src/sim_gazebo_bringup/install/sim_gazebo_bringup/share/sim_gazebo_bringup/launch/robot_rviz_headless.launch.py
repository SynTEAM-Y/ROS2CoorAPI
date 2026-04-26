#!/usr/bin/env python3
"""
Launch file for RViz visualization of x3plus robot (headless - no GUI components).

This is a simplified version without the joint_state_publisher_gui and direct RViz launch
to avoid libpthread library issues on systems with snap conflicts.

Usage:
    ros2 launch sim_gazebo_bringup robot_rviz_headless.launch.py
    
Then separately, publish joint states:
    ros2 run joint_state_publisher_gui joint_state_publisher_gui   (if you want GUI)
    or
    ros2 run joint_state_publisher joint_state_publisher           (headless)

Or control the robot:
    ros2 run x3plus_examples manual_control
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package shares
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    yahboomcar_description_dir = get_package_share_directory('yahboomcar_description')
    
    # Paths
    urdf_file = os.path.join(yahboomcar_description_dir, 'urdf', 'yahboomcar_X3plus.urdf')
    rviz_config_file = os.path.join(yahboomcar_description_dir, 'rviz', 'yahboomcar.rviz')
    
    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz visualization'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
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
        condition=DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz'
        ).condition
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
