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
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    max_range  = LaunchConfiguration('max_range')
    move_robot = LaunchConfiguration('move_robot')
    simulate   = LaunchConfiguration('simulate')
    start_lidar = LaunchConfiguration('start_lidar')

    pkg_share = get_package_share_directory('x3plus_mapping_bringup')

    urdf_file = os.path.join(pkg_share, 'urdf', 'yahboomcar_X3plus.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('x3plus_mapping_bringup'),
        'rviz',
        'gmapping_view.rviz'
    ])

    ydlidar_params_file = os.path.join(
        get_package_share_directory('x3plus_lidar_bringup'),
        'params',
        'tg30.yaml',
    )

    return LaunchDescription([
        # Launch arguments (show with --show-args)
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
            description='Robot body frame (e.g. base_link or base_footprint).'
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry reference frame.'
        ),
        DeclareLaunchArgument('max_range', default_value='30.0',
            description='Maximum LiDAR range for gmapping (in meters).'
        ),
        DeclareLaunchArgument('move_robot', default_value='true',
            description='Whether ReactiveNav should send velocity commands.'
        ),
        DeclareLaunchArgument('simulate', default_value='false',
            description='If true, skips hardware-specific nodes (e.g. base bridge).'
        ),
        DeclareLaunchArgument('start_lidar', default_value='true',
            description='If true, start the LiDAR driver from this launch.'
        ),

        # Robot model and TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # Odometry source (integrates /cmd_vel into /odom)
        Node(
            package='x3plus_mapping_bringup',
            executable='odom_integrator',
            name='odom_integrator',
            output='screen',
            parameters=[{'odom_frame': odom_frame, 'base_frame': base_frame}],
        ),

        # LiDAR driver (optional, runs only if enabled)
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar',
            output='screen',
            parameters=[
                ydlidar_params_file,
            ],
            # condition=IfCondition(move_robot), # & UnlessCondition(simulate) & IfCondition(start_lidar),

            # condition=IfCondition('PythonExpression([
            # '(', simulate, ' == "f'alse") and (', start_lidar, ' == "true")'
            #])) TODO
        ),

        # SLAM (GMapping)
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'map_update_interval': 2.0,
                'linearUpdate': 0.2,
                'angularUpdate': 0.2,
                'temporalUpdate': 1.0,
                'tf_delay': 0.05,
                'maxUrange': max_range,
                'maxRange':  max_range,
                'occ_thresh': 0.25,
            }],
        ),

        # Reactive obstacle avoidance
        Node(
            package='x3plus_mapping_bringup',
            executable='reactive_nav',
            name='reactive_nav',
            output='screen',
            parameters=[{
                'move_robot': move_robot,
                'forward_speed': 0.12,
                'turn_speed': 0.8,
                'front_clear': 0.55,
                'hard_stop': 0.22,
                'min_valid_ratio': 0.05,
                'left_fov_deg': 90.0,
                'right_fov_deg': -90.0,
                'front_half_deg': 20.0,
            }],
        ),

        # Hardware bridge to motors (real robot only)
        Node(
            package='x3plus_examples',
            executable='rosmaster_base_bridge',
            name='rosmaster_base_bridge',
            output='screen',
            parameters=[{
                'v_max': 0.6,
                'w_max': 2.5,
                'lin_deadband': 0.03,
                'ang_deadband': 0.10,
                'accel_limit_pct_per_s': 200.0,
                'cmd_timeout': 0.6,
                'stabilize': 1,
            }],
            condition=UnlessCondition(simulate),
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        ),
    ])
