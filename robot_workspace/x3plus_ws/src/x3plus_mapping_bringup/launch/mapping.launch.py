#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base_frame  = LaunchConfiguration('base_frame')
    odom_frame  = LaunchConfiguration('odom_frame')
    max_range   = LaunchConfiguration('max_range')
    simulate    = LaunchConfiguration('simulate')
    start_lidar = LaunchConfiguration('start_lidar')
    display_map_live = LaunchConfiguration('display_map_live')

    pkg_share  = get_package_share_directory('x3plus_mapping_bringup')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'yahboomcar_X3plus.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('x3plus_mapping_bringup'), 'rviz', 'gmapping_view.rviz'
    ])

    ydlidar_params_file = os.path.join(
        get_package_share_directory('x3plus_lidar_bringup'),
        'params',
        'tg30.yaml',
    )

    return LaunchDescription([
        # Launch arguments (show with --show-args)
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
            description='Robot base frame (e.g., base_link or base_footprint).'
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry reference frame used by SLAM and TF.'
        ),
        DeclareLaunchArgument('max_range', default_value='30.0',
            description='Maximum LiDAR range considered by gmapping (meters).'
        ),
        DeclareLaunchArgument('simulate', default_value='false',
            description='If true, enables sim helpers and skips hardware-only nodes.'
        ),
        DeclareLaunchArgument('start_lidar', default_value='true',
            description='If true, start the YDLIDAR driver from this launch.'
        ),
        DeclareLaunchArgument('range_min', default_value='0.18',
            description='Driver min usable range for LiDAR (meters).'
        ),
        DeclareLaunchArgument('range_max', default_value='10.0',
            description='Driver max usable range for LiDAR (meters).'
        ),
        DeclareLaunchArgument('display_map_live', default_value='false',
            description='If true, displays the map live using Rviz2. Disable to save computation power or to display it elsewhere.'
        ),

        # URDF -> TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Joint state publisher (simulation only)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(simulate),
        ),

        # Optional LiDAR (start on real robot if you want this launch to be self-contained)
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar',
            output='screen',
            parameters=[
                ydlidar_params_file, {
                    'range_min': LaunchConfiguration('range_min'),
                    'range_max': LaunchConfiguration('range_max'),
            }],
            condition=IfCondition(start_lidar),
            # condition=IfCondition(
            #     PythonExpression([
            #         '("', simulate, '".lower() == "false") and ("', start_lidar, '".lower() == "true")'
            #     ])
            # ),
        ),

        # Odom source (dead-reckon from /cmd_vel). Use teleop to move while mapping.
        Node(
            package='x3plus_mapping_bringup',
            executable='odom_integrator',
            name='odom_integrator',
            output='screen',
            parameters=[{'odom_frame': odom_frame, 'base_frame': base_frame}],
        ),

        # SLAM (subscribes to /scan; needs TF: odom->base_link and base_link->laser_link)
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'map_update_interval': 2.0,
                'linearUpdate': 0.3,
                'angularUpdate': 0.35,
                'temporalUpdate': 1.0,
                'tf_delay': 0.10,
                'maxUrange': 8.0,
                'maxRange':  10.0,
                'occ_thresh': 0.35,
            }],
        ),

        # RViz (Fixed Frame = map; add TF, LaserScan(/scan), Map(/map))
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            condition=IfCondition(display_map_live)
        ),
    ])
