#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch gripper calibration test with Gazebo infrastructure.
    
    Sequence:
    1. t=0s:   Gazebo starts
    2. t=15s:  Blue test_block spawned at (2.0, 0.0, 0.03)
    3. t=16s:  TF relay for test_block activated
    4. t=20s:  Gripper calibration node starts
    """
    
    # Get package directories
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    # Find the empty world file
    empty_world = os.path.join(sim_gazebo_bringup_dir, 'worlds', 'empty.sdf')
    
    # Get the test_block model path
    test_block_model = os.path.join(sim_gazebo_bringup_dir, 'models', 'test_block', 'model.sdf')
    
    ld = LaunchDescription()
    
    # ===== Start Gazebo (using ros_gz_sim) =====
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s {empty_world}'
        }.items()
    )
    ld.add_action(gazebo_launch)
    
    # ===== Spawn Robot in Gazebo =====
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'x3plus',
            '-world', 'empty'
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)
    
    # ===== Robot State Publisher =====
    urdf_path = os.path.join(sim_gazebo_bringup_dir, 'urdf', 'yahboomcar_X3plus.urdf.xacro')
    # If only xacro file exists, use a pre-compiled urdf from another package
    if not os.path.exists(urdf_path.replace('.xacro', '.urdf')):
        # Try to find a compiled URDF from other packages
        try:
            multi_bringup_dir = get_package_share_directory('x3plus_multi_bringup')
            urdf_path = os.path.join(multi_bringup_dir, 'urdf', 'yahboomcar_X3plus.urdf')
        except:
            pass
    
    if urdf_path.endswith('.urdf'):
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
    else:
        # For xacro files, would need xacro processor
        with open(urdf_path.replace('.xacro', '.urdf'), 'r') as f:
            robot_description = f.read()
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )
    ld.add_action(rsp_node)
    
    # ===== ROS-Gazebo Bridge =====
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/x3plus/cmd_vel@geometry_msgs/Twist]ignition.msgs.Twist',
            '/arm_joint1_cmd_pos@std_msgs/Float64]ignition.msgs.Double',
            '/arm_joint2_cmd_pos@std_msgs/Float64]ignition.msgs.Double',
            '/arm_joint3_cmd_pos@std_msgs/Float64]ignition.msgs.Double',
            '/arm_joint4_cmd_pos@std_msgs/Float64]ignition.msgs.Double',
            '/arm_joint5_cmd_pos@std_msgs/Float64]ignition.msgs.Double',
            '/grip_joint_cmd_pos@std_msgs/Float64]ignition.msgs.Double',
            '/world/empty/pose/info@tf2_msgs/TFMessage[ignition.msgs.Pose_V',
        ],
        output='screen'
    )
    ld.add_action(bridge_node)
    
    # ===== TF Relay for x3plus =====
    x3plus_tf_relay = Node(
        package='sim_gazebo_bringup',
        executable='gazebo_pose_tf_relay',
        arguments=['/gz_pose_tf', 'odom', 'x3plus'],
        output='screen',
        name='x3plus_tf_relay'
    )
    ld.add_action(x3plus_tf_relay)
    
    # ===== Static TF map->odom =====
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_tf)
    
    # ===== Gripper Mimic Relay =====
    gripper_relay = Node(
        package='sim_gazebo_bringup',
        executable='gripper_mimic_relay',
        output='screen'
    )
    ld.add_action(gripper_relay)
    
    # ===== Spawn Test Cube (t=15s) =====
    spawn_cube = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-world', 'empty',
                     '-file', test_block_model,
                     '-name', 'test_block',
                     '-x', '2.0', '-y', '0.0', '-z', '0.03'],
                output='screen'
            )
        ]
    )
    ld.add_action(spawn_cube)
    
    # ===== TF Relay for Test Block (t=16s) =====
    block_tf_relay = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='sim_gazebo_bringup',
                executable='gazebo_pose_tf_relay',
                arguments=['/gz_pose_tf', 'odom', 'test_block'],
                output='screen',
                name='test_block_tf_relay'
            )
        ]
    )
    ld.add_action(block_tf_relay)
    
    # ===== Start Gripper Calibration Test (t=20s) =====
    calibrator = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='sim_gazebo_bringup',
                executable='test_gripper_calibration',
                output='screen',
                name='gripper_calibrator'
            )
        ]
    )
    ld.add_action(calibrator)
    
    return ld
