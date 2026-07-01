#!/usr/bin/env python3
"""Launch vision-based autonomous pick-and-place system.

Launches:
  - Gazebo simulation with x3plus and NavNav2 navigation stack (via gazebo_nav2.launch.py)
  - Static identity transform map->odom
  - Object detector node (vision-based detection via mono/depth cameras)
  - Blue test cube (spawned at 2.0, 0.0)
  - Green landing pad (spawned at 2.0, 1.2)
  - Vision pick-and-place autopilot node (runs the state machine)

Usage:
  ros2 launch sim_gazebo_bringup vision_autopilot.launch.py
  ros2 launch sim_gazebo_bringup vision_autopilot.launch.py world:=office map:=plain_map
"""

import os
import sys

# CRITICAL: Disable the interactive picker in gazebo.launch.py BEFORE any
# imports that might trigger it. When gazebo.launch.py is included as a
# sub-launch, its generate_launch_description() runs in the same process
# and the picker fires if these env vars are not set.
os.environ['SIM_GAZEBO_BRINGUP_NO_PROMPT'] = '1'
os.environ['SIM_GAZEBO_BRINGUP_WORLD'] = 'office'
os.environ['SIM_GAZEBO_BRINGUP_MAP'] = 'plain_map'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _get_cli_arg(name, default):
    """Read a command-line argument like 'world:=office' directly."""
    prefix = f'{name}:='
    for a in sys.argv[1:]:
        if a.startswith(prefix):
            return a.split(':=', 1)[1]
    return default


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')

    # Parse command-line arguments into concrete Python strings.
    world_str = _get_cli_arg('world', 'office')
    map_str = _get_cli_arg('map', 'plain_map')

    # Keep DeclareLaunchArgument for documentation and ROS2 arg introspection,
    # but use the concrete strings above for the actual include.
    world_arg = DeclareLaunchArgument(
        'world', default_value='office',
        description='Gazebo world (office required for pick-and-place; contains the static wall at 2,2)')
    map_arg = DeclareLaunchArgument(
        'map', default_value='plain_map',
        description='Map for Nav2 navigation')

    # Gazebo + Nav2. We keep Nav2 running so AMCL and trajectory_bridge stay alive
    # for arm/gripper controllers.
    gazebo_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_gazebo_bringup_dir, 'launch', 'gazebo_nav2.launch.py')
        ),
        launch_arguments={
            'world': world_str,
            'map': map_str,
        }.items()
    )

    # NOTE: map->odom TF is already published by Nav2 (amcl_node and others).
    # Do NOT add a static publisher here; it will conflict and cause TF lookup errors.
    # Removing static_map_to_odom from return list below.

    # Bridge gazebo's namespaced sensor frames into the URDF-derived TF tree.
    # Gazebo publishes images with frame_id = "<model>::<link>::<sensor>" which
    # is NOT in the URDF tree, so any tf lookup that uses these frames fails.
    # Gazebo camera sensors publish with frame_id =
    # <model>/<link>/<sensor>, e.g.  x3plus/camera_link/depth_camera.
    # The image data follows the ROS optical convention (x=right, y=down,
    # z=forward) while the URDF body frames (camera_link / mono_link) use
    # x=forward, y=left, z=up. The rpy=(-pi/2, 0, -pi/2) static rotation
    # maps the URDF body axis to the Gazebo optical axis so TF can connect
    # the Gazebo-namespaced camera frame (child, optical) through the URDF
    # tree (camera_link -> ... -> map) on the parent side.
    # Depth camera optical frame bridge. Gazebo publishes images with frame_id
    # x3plus/camera_link/depth_camera (Gazebo model hierarchy), but the URDF
    # tree has camera_link as the parent. We need transform FROM Gazebo frame
    # TO URDF frame so the camera data can be used in the URDF TF chain.
    # The rpy=(-pi/2, 0, -pi/2) rotation maps Gazebo optical convention
    # (x=right, y=down, z=forward) to URDF camera convention.
    static_depth_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_depth_camera_frame',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.5707963267948966',
                   '--pitch', '0',
                   '--yaw', '-1.5707963267948966',
                   '--frame-id', 'x3plus/camera_link/depth_camera',
                   '--child-frame-id', 'camera_link'],
    )
    static_depth_camera_frame_legacy = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_depth_camera_frame_legacy',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.5707963267948966',
                   '--pitch', '0',
                   '--yaw', '-1.5707963267948966',
                   '--frame-id', 'x3plus/base_footprint/depth_camera',
                   '--child-frame-id', 'camera_link'],
    )
    # Wrist camera: similar bridge from Gazebo frame to URDF mono_link frame
    static_wrist_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_wrist_camera_frame',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.5707963267948966',
                   '--pitch', '0',
                   '--yaw', '-1.5707963267948966',
                   '--frame-id', 'x3plus/mono_link/wrist_mono_camera',
                   '--child-frame-id', 'mono_link'],
    )

# Object detector
    object_detector_node = Node(
        package='sim_gazebo_bringup',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Use front mono camera
            'camera_topic': '/mono_camera/image_raw',
            'depth_topic': '/depth_camera/depth_image',
            # Use camera_info from the remapped topic to get consistent frame_id
            'camera_info_topic': '/mono_camera/camera_info',
            # Gazebo blue cube RGB(0, 0.5, 1.0) → HSV around hue 100-120
            # Widen range to catch from different angles/lighting
            'hsv_lower_h': 70,
            'hsv_lower_s': 30,
            'hsv_lower_v': 30,
            'hsv_upper_h': 130,
            'hsv_upper_s': 255,
            'hsv_upper_v': 255,
            # Lower min_area to detect cube at any distance
            'min_area': 30,
        }],
    )

    # Spawn the blue test cube — 2 m in front of the robot. Delayed 20 s.
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
    spawn_delayed = TimerAction(period=20.0, actions=[spawn_test_object_node])

    # Landing pad — green square on the floor in front of the wall. Delayed 22 s.
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
    spawn_landing_pad_delayed = TimerAction(period=22.0, actions=[spawn_landing_pad_node])

    # ── test_block ground-truth TF ──────────────────────────────────────────
    # The test_block model.sdf includes an Ignition PosePublisher plugin
    # that publishes /model/test_block/tf (Pose_V → TFMessage via bridge).
    # We relay that into the ROS TF tree as 'odom -> test_block' so that
    # vision_pick_place._cube_is_lifted() can read the cube's z height and
    # _drive_to_face_cube() can know where to drive.
    # Delayed 21 s — just after spawn_test_object fires at 20 s.
    #
    # NOTE: Gazebo may publish to /world/{world_name}/model/test_block/tf
    # depending on version. We bridge both possible topic names.
    test_block_tf_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='test_block_tf_bridge',
        output='screen',
        arguments=[
            '/model/test_block/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/model/test_block/tf', '/gz_test_block_tf'),
        ],
        parameters=[{'use_sim_time': True}],
    )
    # Fallback bridge for world-namespaced topic (some Gazebo versions)
    test_block_tf_bridge_world_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='test_block_tf_bridge_world',
        output='screen',
        arguments=[
            f'/world/{world_str}/model/test_block/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            (f'/world/{world_str}/model/test_block/tf', '/gz_test_block_tf_world'),
        ],
        parameters=[{'use_sim_time': True}],
    )
    test_block_tf_relay_node = Node(
        package='sim_gazebo_bringup',
        executable='gazebo_pose_tf_relay',
        name='test_block_tf_relay',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'parent_frame': 'odom'},
            {'child_frame': 'test_block'},
            {'input_topic': '/gz_test_block_tf'},
            {'input_type': 'tf'},
            # Ignition publishes model pose with child_frame_id == model name.
            {'source_child': 'test_block'},
        ],
    )
    test_block_tf_delayed = TimerAction(
        period=21.0,
        actions=[test_block_tf_bridge_node, test_block_tf_bridge_world_node, test_block_tf_relay_node],
    )

    # Vision-based autopilot node.
    # Delayed 25 s so Gazebo, Nav2, and TF are fully loaded and initialized.
    vision_pick_place_node = Node(
        package='sim_gazebo_bringup',
        executable='vision_pick_place',
        name='vision_pick_place',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'target_frame': 'base_footprint'},
            {'camera_frame': 'camera_link'},
            {'pose_topic': '/detected_object_pose'},
            {'nav_action': '/navigate_to_pose'},
            {'arm_action': '/arm_group_controller/follow_joint_trajectory'},
            {'gripper_action': '/gripper_group_controller/follow_joint_trajectory'},
            {'cmd_vel_topic': '/cmd_vel'},
            # Target object default (fallback if camera doesn't detect initially)
            {'object_x': 2.0},
            {'object_y': 0.0},
            {'object_z': 0.03},
            # Destination / Placing coordinates
            {'drop_off_x': 2.0},
            {'drop_off_y': 1.2},
            # Use default drop_off_yaw from vision_pick_place.py (π/2 for perpendicular approach)
            # Approach settings
            {'approach_distance_m': 0.5},
            {'approach_speed': 0.1},
            {'approach_timeout_sec': 10.0},
            {'arm_pid_p': 2.0},
            {'arm_pid_i': 0.0},
            {'arm_pid_d': 0.5},
            {'arm_joints': ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']},
            {'gripper_joints': ['grip_joint']},
            # TF waiting - wait for required transforms before starting navigation
            {'wait_for_tf_timeout': 30.0},
            {'use_gt_tf_for_nav': True},
        ]
    )
    vision_autopilot_delayed = TimerAction(period=25.0, actions=[vision_pick_place_node])

    return LaunchDescription([
        world_arg, map_arg,
        gazebo_nav2_launch,
        static_depth_camera_frame,
        static_depth_camera_frame_legacy,
        static_wrist_camera_frame,
        object_detector_node,
        spawn_delayed,
        spawn_landing_pad_delayed,
        test_block_tf_delayed,       # bridges & relays test_block TF (Fix: _cube_is_lifted)
        vision_autopilot_delayed,
    ])
