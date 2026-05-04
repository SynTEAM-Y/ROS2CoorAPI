#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

# Example usage:
# ros2 launch x3plus_sim_bringup nav2_on_static_map.launch.py map:=/absolute/path/to/map.yaml

def generate_launch_description():
    map_yaml     = LaunchConfiguration('map')           # Path to the map.yaml file
    use_sim_time = LaunchConfiguration('use_sim_time')  # Toggle simulated time for Gazebo or offline sim
    base_frame   = LaunchConfiguration('base_frame')    # Robot’s base frame (e.g., base_footprint)
    odom_frame   = LaunchConfiguration('odom_frame')    # Odometry frame name

    # Locate URDF and RViz configs
    urdf = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'urdf', 'yahboomcar_X3plus.urdf'
    ])
    navigation_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch', 'navigation_launch.py'
    ])
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('x3plus_sim_bringup'),
        'rviz', 'sim_view.rviz'
    ])

    return LaunchDescription([
        # --- Launch Arguments ---
        DeclareLaunchArgument('map', description='Path to the map.yaml file to load.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulated time (set true when running in Gazebo or playback).'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
            description='Robot base frame (e.g., base_link or base_footprint).'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry reference frame.'),
        DeclareLaunchArgument('alias_base_link', default_value='false',
            description='If true, publish static base_footprint->base_link identity TF.'),

        # --- TF from URDF ---
        # Publishes robot’s kinematic tree (TFs) based on URDF description.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf],
            output='screen',
        ),

        # Joint state publisher to provide joint positions (needed for TF publishing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # --- Odometry source (odom -> base_frame) ---
        # Integrates /cmd_vel and publishes /odom + TF link.
        Node(
            package='x3plus_sim_bringup',
            executable='odom_integrator',
            name='odom_integrator',
            output='screen',
            parameters=[{'odom_frame': odom_frame, 'base_frame': base_frame}],
        ),

        # --- Static Map Server (loads pre-saved map) ---
        # Provides /map topic and metadata, used by Nav2 for localization/planning.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'frame_id': 'map',
                'topic_name': 'map'
            }],
        ),

        # --- Static Transform: map -> odom ---
        # Bypasses AMCL/fake_localization when running static map without localization.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=['0', '0', '0', '0', '0', '0', 'map', LaunchConfiguration('odom_frame')],
            output='screen',
        ),

        # --- Lifecycle Manager for Map Server ---
        # Automatically transitions map_server through its managed states.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        # --- Nav2 Core Stack ---
        # Includes planner, controller, behavior tree nodes, etc.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # --- RViz Visualization ---
        # Opens RViz preloaded with navigation view and map visualization.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ])
