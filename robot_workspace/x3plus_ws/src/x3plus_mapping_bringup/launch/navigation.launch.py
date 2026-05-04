#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base_frame  = LaunchConfiguration('base_frame')
    odom_frame  = LaunchConfiguration('odom_frame')
    move_robot  = LaunchConfiguration('move_robot')
    simulate    = LaunchConfiguration('simulate')
    robot    = LaunchConfiguration('robot')

    pkg_share  = get_package_share_directory('x3plus_mapping_bringup')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'yahboomcar_X3plus.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    ydlidar_params_file = os.path.join(
        get_package_share_directory('x3plus_lidar_bringup'),
        'params',
        'tg30.yaml',
    )

    robot_env = os.getenv('ROBOT', '').strip() or ''

    return LaunchDescription([
        # Launch arguments (show with --show-args)
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
            description='Robot body frame (e.g. base_link or base_footprint).'
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry reference frame.'
        ),
        DeclareLaunchArgument('move_robot', default_value='true',
            description='Whether ReactiveNav should send velocity commands.'
        ),
        DeclareLaunchArgument('simulate', default_value='false',
            description='If true, enables simulation mode and skips hardware-specific nodes.'
        ),
        DeclareLaunchArgument('robot', default_value=robot_env,
            description='Robot namespace, used to namespace /scan and /cmd_vel.'
        ),

        # TF publisher (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_x3',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(simulate),
        ),

        # Joint publisher (simulation only)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(simulate),
        ),

        # Simulated odometry integrator
        Node(
            package='x3plus_mapping_bringup',
            executable='odom_integrator',
            name='odom_integrator',
            output='screen',
            parameters=[{'odom_frame': odom_frame, 'base_frame': base_frame}],
            condition=IfCondition(simulate),
        ),

        # LiDAR driver (only when on real hardware and move_robot is true)
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar',
            output='screen',
            parameters=[ydlidar_params_file],
            remappings=[('/scan', [TextSubstitution(text='/'), robot, TextSubstitution(text='/scan')])],
            condition=IfCondition(move_robot),
        ),

        # Reactive navigation (reads /scan, publishes /cmd_vel)
        Node(
            package='x3plus_mapping_bringup',
            executable='reactive_nav',
            name='reactive_nav',
            output='screen',
            parameters=[{
                'move_robot': move_robot,
                'forward_speed': 0.12,
                'turn_speed': 1.0,
                'front_clear': 0.55,
                'hard_stop': 0.22,
                'min_valid_ratio': 0.05,
                'left_fov_deg': 90.0,
                'right_fov_deg': -90.0,
                'front_half_deg': 20.0,
            }],
        ),

        # Base bridge to motors (real robot only)
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
    ])
