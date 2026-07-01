#!/usr/bin/env python3
"""Launch Gazebo simulation with x3plus and SLAM Toolbox for online mapping."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz visualization'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world to load (basename or absolute path).'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    world_value = LaunchConfiguration('world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_gazebo_bringup_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_value,
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
            'use_nav2': 'true',  # SLAM Toolbox publishes map→odom
        }.items()
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'max_laser_range': 10.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 4000000,
            'enable_interactive_mode': True,
        }]
    )

    rviz_config_file = os.path.join(sim_gazebo_bringup_dir, 'rviz', 'gazebo_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        world_arg,
        gazebo_launch,
        slam_toolbox_node,
        rviz_node,
    ])
