#!/usr/bin/env python3
"""
Multi-robot cube-and-sink simulation launch file.

This launch file builds a Gazebo scene that follows the same structure,
assets, robot configuration, and behaviour patterns used in
`scripts/x3plus_examples/vision_autopilot_simple.py`.  The world SDF
(`worlds/multi_robot_scene.sdf`) has the same physics, ground plane,
lighting, and wall as `worlds/office.sdf` (the world vision_autopilot_simple
uses) plus the manipulable objects needed for the dual-robot task.

Layout (world frame, X=forward, Y=left, Z=up):
                              (wall, y=2.5)
                                  |
                                  v

              landing_pad     sink         blue cube
                  (2, 1.2)   (2, 0.0)       (2, -1.2)
                      \\         |          //
                       \\        v         //
                  robot_3              robot_2     <-- sink-grasp robots
                                       (towards -Y handle)
                  robot_1                          <-- cube pick robot
                  (-1.5, 0)

The robots are arranged in a straight line along the world Y axis (Y = -0.7,
0, +0.7) at X = -1.5, all facing +X.  The sink is geometrically centred
between the blue cube (-Y side) and the green landing pad (+Y side) so the
two handle-grasp robots can approach the sink from opposite Y sides.

Launches:
  - Gazebo with the multi_robot_scene world
  - Three X3Plus robots in a straight line (ROBOTS list above)
  - Blue cube at (2.0, -1.2), sink at (2.0, 0.0) between cube and landing,
    green landing pad at (2.0, 1.2), yellow object at (0.0, 1.5)
  - Ground-truth TF relays for every robot and every object
  - The multi-robot state machine node
    (scripts/x3plus_examples/multi_robot_cube_sink_autopilot.py)

Each robot is spawned with a namespaced URDF so that /cmd_vel, arm
commands, and TF frames do not collide.  Topic names are prefixed with
the robot name.

Usage:
  ros2 launch sim_gazebo_bringup multi_robot_cube_sink.launch.py
  ros2 launch sim_gazebo_bringup multi_robot_cube_sink.launch.py gui:=true
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


ROBOT_BRIDGE_DELAY_S = 6.0  # wait for spawned models to exist in Gazebo


# ---------------------------------------------------------------------------
# Robot / object layout (world frame)
# ---------------------------------------------------------------------------
# Three X3Plus robots arranged in a single straight line along the world
# Y axis (perpendicular to the cube/sink line on the X axis), 0.7 m apart
# so the chassis do not collide at startup.  robot_1 is the cube pick
# robot (front-line in the +X direction), robot_2 and robot_3 are the
# sink-handle robots (flanking the sink from -Y and +Y respectively).
ROBOTS = [
    {'name': 'robot_1', 'x': -1.5, 'y':  0.0, 'Y': 0.0},   # cube pick
    {'name': 'robot_2', 'x': -1.5, 'y': -0.7, 'Y': 0.0},   # sink -Y handle
    {'name': 'robot_3', 'x': -1.5, 'y':  0.7, 'Y': 0.0},   # sink +Y handle
]

# Objects included in the world SDF.
#
# Dynamic objects stream their poses from Gazebo via PosePublisher +
# ros_gz_bridge + gazebo_pose_tf_relay.  The cube and sink are picked up
# by the world dynamic_pose/info stream and the per-model pose bridge.
#
# Static objects (`<static>true</static>` in the model SDF) do not get a
# usable pose topic from Gazebo when they are included in the world SDF
# (the PosePublisher's `static_publisher` mode does not align with the
# world pose stream's TF format), so their known pose is published with
# `static_transform_publisher` on /tf_static.  Only the green landing pad
# uses this approach; the sink (which is the drop target) is dynamic.
#
# Note: there is no separate "yellow_object" model in this scene because,
# per the task spec, the sink IS the yellow drop target.
DYNAMIC_OBJECTS = {
    'test_block':   {'x': 2.0, 'y': -1.2, 'z': 0.03,
                     'model': 'test_block'},
    'sink':         {'x': 2.0, 'y':  0.0, 'z': 0.035,
                     'model': 'sink', 'R': 1.57079632679, 'P': 0.0, 'Y': 0.0},
}
STATIC_OBJECTS = {
    'landing_pad':  {'x': 2.0, 'y': 1.2, 'z': 0.001,
                     'model': 'landing_pad'},
}

# Joints and links that must be prefixed with the robot namespace so that
# multiple copies of the same URDF can coexist in one Gazebo world.
NAMESPACED_JOINTS = [
    'base_joint', 'base_imu',
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'back_left_wheel_joint', 'back_right_wheel_joint',
    'laser_joint', 'camera_joint',
    'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5',
    'grip_joint',
    'rlink_joint2', 'rlink_joint3',
    'llink_joint1', 'llink_joint2', 'llink_joint3',
    'mono_joint',
]

NAMESPACED_LINKS = [
    'base_footprint', 'base_link', 'imu_link',
    'front_left_wheel', 'front_right_wheel',
    'back_left_wheel', 'back_right_wheel',
    'laser_link', 'camera_link',
    'arm_link1', 'arm_link2', 'arm_link3', 'arm_link4', 'arm_link5',
    'rlink1', 'rlink2', 'rlink3',
    'llink1', 'llink2', 'llink3',
    'mono_link',
]


def _convert_package_uris_to_absolute_paths(urdf_content):
    """Convert package:// URIs to absolute file:// URIs for Gazebo."""
    def _replace(match):
        uri = match.group(0)
        parts = re.match(r'package://([^/]+)/(.*)', uri)
        if parts:
            pkg = parts.group(1)
            rel = parts.group(2)
            try:
                share = get_package_share_directory(pkg)
                return 'file://' + os.path.join(share, rel)
            except Exception as e:
                print(f"Warning: could not resolve {pkg}: {e}")
                return uri
        return uri

    return re.sub(r'package://[^"\'<\s]+', _replace, urdf_content)


def _make_namespaced_urdf(base_urdf, robot_name):
    """
    Create a robot-specific URDF by prefixing joint/link names and updating
    all Gazebo plugin topic references so the three robots do not share
    controllers, topics, or TF frames.
    """
    # The xacro emits names like "/arm_joint5" when ns is empty.  Strip the
    # leading slash from every XML attribute value so the prefix rewrite below
    # matches bare joint/link tokens.
    urdf = re.sub(r'(?<=name=")/', '', base_urdf)
    urdf = re.sub(r'(?<=parent=")/', '', urdf)
    urdf = re.sub(r'(?<=child=")/', '', urdf)
    urdf = re.sub(r'(?<=link=")/', '', urdf)
    urdf = re.sub(r'(?<=reference=")/', '', urdf)
    urdf = re.sub(r'(?<=mimic joint=")/', '', urdf)
    urdf = re.sub(r'(?<=joint_name>)/', '', urdf)

    prefix = robot_name + '_'

    # Prefix every namespaced joint/link in name=, reference=, parent=, child=,
    # joint_name= and mimic joint= attributes.  Do this BEFORE touching topic
    # strings so that topic names that accidentally contain bare link names
    # are not affected.
    token_pat = r'\b(' + '|'.join(NAMESPACED_JOINTS + NAMESPACED_LINKS) + r')\b'

    def _prefix_token(match):
        return prefix + match.group(1)

    urdf = re.sub(r'name="(' + '|'.join(NAMESPACED_JOINTS + NAMESPACED_LINKS) + r')"',
                  lambda m: f'name="{prefix}{m.group(1)}"', urdf)
    urdf = re.sub(r'reference="(' + '|'.join(NAMESPACED_JOINTS + NAMESPACED_LINKS) + r')"',
                  lambda m: f'reference="{prefix}{m.group(1)}"', urdf)
    urdf = re.sub(r'parent link="(' + '|'.join(NAMESPACED_LINKS) + r')"',
                  lambda m: f'parent link="{prefix}{m.group(1)}"', urdf)
    urdf = re.sub(r'child link="(' + '|'.join(NAMESPACED_LINKS) + r')"',
                  lambda m: f'child link="{prefix}{m.group(1)}"', urdf)
    urdf = re.sub(r'<joint_name>(' + '|'.join(NAMESPACED_JOINTS) + r')</joint_name>',
                  lambda m: f'<joint_name>{prefix}{m.group(1)}</joint_name>', urdf)
    urdf = re.sub(r'<mimic joint="(' + '|'.join(NAMESPACED_JOINTS) + r')"',
                  lambda m: f'<mimic joint="{prefix}{m.group(1)}"', urdf)
    urdf = re.sub(r'<collision>(' + '|'.join(NAMESPACED_LINKS) + r')_collision</collision>',
                  lambda m: f'<collision>{prefix}{m.group(1)}_collision</collision>', urdf)

    # Robot name itself.
    urdf = urdf.replace(
        '<robot name="yahboomcar_X3plus">',
        f'<robot name="{robot_name}_yahboomcar_X3plus">')

    # Gazebo plugin topics: arm/gripper position controllers.
    for j in ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4',
              'arm_joint5']:
        urdf = urdf.replace(
            f'<topic>/{j}_cmd_pos</topic>',
            f'<topic>/{robot_name}/{j}_cmd_pos</topic>')
    urdf = urdf.replace(
        '<topic>/grip_master_target</topic>',
        f'<topic>/{robot_name}/grip_master_target</topic>')
    for j in ['llink_joint1', 'llink_joint2', 'llink_joint3',
              'rlink_joint2', 'rlink_joint3']:
        urdf = urdf.replace(
            f'<topic>/{j}_cmd_pos</topic>',
            f'<topic>/{robot_name}/{j}_cmd_pos</topic>')

    # DiffDrive uses the Gazebo model name in its topics and the wheel
    # joint names must match the namespaced URDF joints.
    urdf = urdf.replace(
        '<topic>/model/x3plus/cmd_vel</topic>',
        f'<topic>/model/{robot_name}/cmd_vel</topic>')
    urdf = urdf.replace(
        '<odom_topic>/model/x3plus/odometry</odom_topic>',
        f'<odom_topic>/model/{robot_name}/odometry</odom_topic>')
    urdf = urdf.replace(
        '<tf_topic>/model/x3plus/tf</tf_topic>',
        f'<tf_topic>/model/{robot_name}/tf</tf_topic>')
    urdf = urdf.replace(
        '<left_joint>front_left_wheel_joint</left_joint>',
        f'<left_joint>{prefix}front_left_wheel_joint</left_joint>')
    urdf = urdf.replace(
        '<right_joint>front_right_wheel_joint</right_joint>',
        f'<right_joint>{prefix}front_right_wheel_joint</right_joint>')
    urdf = urdf.replace(
        '<left_joint>back_left_wheel_joint</left_joint>',
        f'<left_joint>{prefix}back_left_wheel_joint</left_joint>')
    urdf = urdf.replace(
        '<right_joint>back_right_wheel_joint</right_joint>',
        f'<right_joint>{prefix}back_right_wheel_joint</right_joint>')

    # IMU and contact sensor topics.
    urdf = urdf.replace(
        '<topic>/model/x3plus/imu</topic>',
        f'<topic>/model/{robot_name}/imu</topic>')
    # The contact sensor's <topic> element is ignored by Gazebo; the
    # sensor publishes to /world/.../link/.../sensor/.../contact by
    # default. We leave the <topic> in the URDF as a documentation aid
    # and bridge the actual default path (see bridge config below).

    # Camera topics.
    urdf = urdf.replace(
        '<topic>/depth_camera</topic>',
        f'<topic>/{robot_name}/depth_camera</topic>')
    urdf = urdf.replace(
        '<topic>/wrist_mono_camera/image</topic>',
        f'<topic>/{robot_name}/wrist_mono_camera/image</topic>')

    # PosePublisher topic.
    urdf = urdf.replace(
        '<topic>/model/x3plus/pose</topic>',
        f'<topic>/model/{robot_name}/pose</topic>')

    return urdf


def _bridge_args(robot_name, world_name):
    """ros_gz_bridge argument list for one robot namespace."""
    r = robot_name
    w = world_name
    return [
        # Arm / gripper command topics (ROS -> GZ)
        f'/{r}/arm_joint1_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/arm_joint2_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/arm_joint3_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/arm_joint4_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/arm_joint5_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/grip_master_target@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/llink_joint1_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/llink_joint2_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/llink_joint3_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/rlink_joint2_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        f'/{r}/rlink_joint3_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        # Differential drive
        f'/model/{r}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        f'/model/{r}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/model/{r}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        # Contact sensors (gz uses the world-relative sensor path; the
        # <topic> element in the SDF is ignored by the contact sensor plugin,
        # so the sensor publishes to the default path).
        f'/world/{w}/model/{r}/link/{r}_llink2/sensor/llink2_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        f'/world/{w}/model/{r}/link/{r}_rlink2/sensor/rlink2_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        # Joint states
        f'/world/{w}/model/{r}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        # Cameras
        f'/{r}/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'/{r}/depth_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'/{r}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        f'/{r}/wrist_mono_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'/{r}/wrist_mono_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        # Ground-truth model pose (for odom -> base_footprint TF relay).
        f'/model/{r}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    ]


def _bridge_remaps(robot_name, world_name):
    """Remap the ROS-side names of selected topics for one robot."""
    r = robot_name
    w = world_name
    return [
        (f'/model/{r}/cmd_vel', f'/{r}/cmd_vel'),
        (f'/model/{r}/odometry', f'/{r}/odom'),
        (f'/model/{r}/imu', f'/{r}/imu'),
        (f'/world/{w}/model/{r}/joint_state', f'/{r}/joint_states_raw'),
        (f'/{r}/depth_camera/image', f'/{r}/mono_camera/image_raw'),
        (f'/{r}/depth_camera/camera_info', f'/{r}/mono_camera/camera_info'),
        (f'/{r}/wrist_mono_camera/image', f'/{r}/wrist_mono_camera/image_raw'),
        (f'/{r}/wrist_mono_camera/camera_info', f'/{r}/wrist_mono_camera/camera_info'),
        (f'/model/{r}/pose', '/gz_pose_tf'),
    ]


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(sim_gazebo_bringup_dir, 'worlds',
                              'multi_robot_scene.sdf')
    world_name = 'multi_robot_scene'
    map_file = os.path.join(sim_gazebo_bringup_dir, 'maps', 'plain_map.yaml')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start RViz (multi-robot TF tree is crowded; default off)')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show Gazebo GUI')

    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')

    # Generate the base URDF from the in-package xacro (ns empty).
    xacro_file = os.path.join(sim_gazebo_bringup_dir, 'urdf',
                              'yahboomcar_X3plus_multi.urdf.xacro')
    result = subprocess.run(
        ['xacro', xacro_file, 'ns:='],
        capture_output=True, text=True, check=True)
    base_urdf = result.stdout
    base_urdf = _convert_package_uris_to_absolute_paths(base_urdf)

    # Shared Gazebo simulation.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=IfCondition(gui))
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s {world_file}'}.items(),
        condition=UnlessCondition(gui))

    # Environment so Gazebo can resolve model:// URIs in our SDF/meshes.
    # We add the share/ parent so model://sim_gazebo_bringup/... resolves
    # to the files inside this package.
    pkg_share_parent = os.path.dirname(sim_gazebo_bringup_dir)
    set_ign_res_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        pkg_share_parent + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))
    set_gz_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        pkg_share_parent + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))

    launch_entities = [
        use_rviz_arg, gui_arg,
        set_ign_res_path, set_gz_model_path,
        gazebo_launch, gazebo_server_launch,
    ]

    # Static map->odom so RViz has a map frame if enabled.
    launch_entities.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    ))

    # One bridge for the shared world DYNAMIC pose stream and simulation clock.
    # dynamic_pose/info includes runtime-spawned models, unlike pose/info which
    # only covers models present when the world SDF was loaded.
    launch_entities.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[(f'/world/{world_name}/dynamic_pose/info', '/gz_pose_tf')],
        parameters=[{'use_sim_time': True}],
    ))

    # Spawn each robot, its state publisher, bridge, mimic relay, and TF relay.
    # The entire robot section is delayed so that Gazebo has finished loading
    # the world and the ros_gz_bridge world/clock connections are live before
    # any model-specific bridges try to subscribe.
    robot_actions = []
    for robot in ROBOTS:
        rname = robot['name']
        urdf = _make_namespaced_urdf(base_urdf, rname)

        # Validate that namespacing produced the expected arm_link5 frame.
        if f'name="{rname}_arm_link5"' not in urdf:
            raise RuntimeError(
                f"URDF rewrite failed: {rname}_arm_link5 not found in namespaced URDF")

        # Robot state publisher.
        robot_actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{rname}_robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': urdf},
                {'use_sim_time': True},
            ],
            remappings=[('joint_states', f'/{rname}/joint_states')],
        ))

        # Spawn in Gazebo.
        robot_actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{rname}',
            arguments=[
                '-string', urdf,
                '-name', rname,
                '-world', world_name,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', '0.0',
                '-Y', str(robot['Y']),
            ],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))

        # ros_gz bridge for this robot.
        robot_actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{rname}_bridge',
            output='screen',
            arguments=_bridge_args(rname, world_name),
            remappings=_bridge_remaps(rname, world_name),
            parameters=[{'use_sim_time': True}],
        ))

        # Gripper mimic relay.
        robot_actions.append(Node(
            package='sim_gazebo_bringup',
            executable='gripper_mimic_relay',
            name=f'{rname}_gripper_mimic_relay',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'namespace': rname},
            ],
        ))

        # cmd_vel relay: forwards /<ns>/cmd_vel -> /model/<ns>/cmd_vel.
        # The autopilot publishes to /<ns>/cmd_vel but the Gazebo
        # DiffDrive plugin subscribes to /model/<ns>/cmd_vel (see
        # the URDF topic remap above). Without this relay the wheels
        # never spin.
        robot_actions.append(Node(
            package='sim_gazebo_bringup',
            executable='cmd_vel_relay',
            name=f'{rname}_cmd_vel_relay',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'cmd_topic': f'/{rname}/cmd_vel'},
                {'gz_cmd_topic': f'/model/{rname}/cmd_vel'},
            ],
        ))

        # Ground-truth pose relay: odom -> robot_base_footprint.
        # The world dynamic-pose stream (/world/.../dynamic_pose/info) is
        # bridged to /gz_pose_tf; the relay filters the model-name entry.
        robot_actions.append(Node(
            package='sim_gazebo_bringup',
            executable='gazebo_pose_tf_relay',
            name=f'{rname}_tf_relay',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'parent_frame': 'odom'},
                {'child_frame': f'{rname}_base_footprint'},
                {'input_topic': '/gz_pose_tf'},
                {'input_type': 'tf'},
                {'source_child': rname},
            ],
        ))

    launch_entities.append(TimerAction(period=ROBOT_BRIDGE_DELAY_S, actions=robot_actions))

    # Bridge ground-truth poses of the DYNAMIC objects (cube, sink) that are
    # included in the world SDF into the shared /gz_pose_tf topic, and relay
    # them as odom -> <object> TF frames.  Ignition's PosePublisher defaults to
    # /model/<name>/pose (not /tf).
    for obj_name, cfg in DYNAMIC_OBJECTS.items():
        launch_entities.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{obj_name}_pose_bridge',
            output='screen',
            arguments=[
                f'/model/{obj_name}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            remappings=[(f'/model/{obj_name}/pose', '/gz_pose_tf')],
            parameters=[{'use_sim_time': True}],
        ))
        launch_entities.append(Node(
            package='sim_gazebo_bringup',
            executable='gazebo_pose_tf_relay',
            name=f'{obj_name}_tf_relay',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'parent_frame': 'odom'},
                {'child_frame': obj_name},
                {'input_topic': '/gz_pose_tf'},
                {'input_type': 'tf'},
                {'source_child': obj_name},
            ],
        ))

    # Static drop target: publish odom -> yellow_object on /tf_static.
    # We use the new-style named arguments for static_transform_publisher;
    # the old-style positional arguments are deprecated and, in this launch
    # context, caused the node to publish an unusable transform so the
    # autopilot could not resolve the yellow_object frame.
    for obj_name, cfg in STATIC_OBJECTS.items():
        launch_entities.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{obj_name}_static_tf',
            output='screen',
            arguments=[
                '--x', str(cfg['x']),
                '--y', str(cfg['y']),
                '--z', str(cfg['z']),
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'odom',
                '--child-frame-id', obj_name,
            ],
            parameters=[{'use_sim_time': True}],
        ))

    # The cube_platform is now an inline <model> in the world SDF (8x8x4 cm
    # box at (2.0,-1.2,0.02) so its top is at z=0.04 m; the 4 cm cube
    # settles on it with its centre at z=0.06 m, well within the arm's
    # REACH_DOWN reach of z~0.05-0.08 m world after gravity sag).

    # Multi-robot state machine.  use_sim_time is required because the
    # ground-truth TF relays stamp outgoing transforms with the simulation
    # clock; without it the autopilot's wall-clock TF buffer cannot resolve
    # them.
    launch_entities.append(TimerAction(
        period=30.0,
        actions=[
            # The attach/detach service: spawns a fixed joint between
            # the gripper's rlink2 and the test_block cube when the
            # gripper is at REACH_DOWN, removes the joint when the
            # gripper opens at PLACE_DOWN. This sidesteps the broken
            # gripper contact physics (rockers overlapping the pad mesh)
            # by treating the gripper as a rigid "claw" that
            # magically grips the cube.
            Node(
                package='sim_gazebo_bringup',
                executable='cube_attach_detach',
                name='cube_attach_detach',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='sim_gazebo_bringup',
                executable='multi_robot_cube_sink_autopilot',
                name='multi_robot_cube_sink_autopilot',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    ))

    return LaunchDescription(launch_entities)
