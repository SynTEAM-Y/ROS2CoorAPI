#!/usr/bin/env python3
"""
Launch file for starting Gazebo simulation with x3plus robot.

This launch file starts:
1. The Gazebo simulator (ros_gz_sim)
2. Spawns the x3plus robot in Gazebo
3. Publishes robot state using robot_state_publisher
4. Optionally starts RViz for visualization

IMPORTANT: This package requires ros_gz_sim to be installed. If you get a
"package 'ros_gz_sim' not found" error, you need to install it:

    sudo apt-get install ros-humble-ros-gz-sim

Or if that doesn't work due to conflicts, use the RViz-only launch instead:

    ros2 launch sim_gazebo_bringup robot_rviz.launch.py

Usage:
    ros2 launch sim_gazebo_bringup gazebo.launch.py

Optional arguments:
    use_rviz:=false                  - Disable RViz (default: true)
    use_sim_time:=false              - Disable simulated time (default: true)
    world:=office                    - World to load. Options come from the
                                       worlds/ directory of this package.
                                       Defaults to 'empty'. Examples:
                                         world:=empty
                                         world:=office
                                       You may also pass an absolute path to a
                                       custom .sdf file.
"""

import os
import re
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def convert_package_uris_to_absolute_paths(urdf_content):
    """
    Convert package:// URIs in URDF to absolute file paths for Gazebo compatibility.
    
    Gazebo cannot resolve ROS package:// URIs, so we convert them to absolute paths
    that Gazebo can understand.
    
    Example:
        package://yahboomcar_description/meshes/X3plus/visual/base_link.STL
        becomes:
        /path/to/install/yahboomcar_description/share/yahboomcar_description/meshes/X3plus/visual/base_link.STL
    """
    def replace_package_uri(match):
        package_uri = match.group(0)
        # Extract package name and file path
        # Format: package://package_name/relative/path
        match_parts = re.match(r'package://([^/]+)/(.*)', package_uri)
        if match_parts:
            package_name = match_parts.group(1)
            relative_path = match_parts.group(2)
            try:
                package_share_dir = get_package_share_directory(package_name)
                absolute_path = os.path.join(package_share_dir, relative_path)
                # Prefix with file:// so Gazebo treats it as an absolute file URI
                # (otherwise it gets resolved against the <urdf-string> source URI).
                return 'file://' + absolute_path
            except Exception as e:
                print(f"Warning: Could not resolve package {package_name}: {e}")
                return package_uri
        return package_uri
    
    # Replace all package:// URIs with absolute paths
    # This regex matches package://package_name/path/to/file
    modified_content = re.sub(r'package://[^"\'<\s]+', replace_package_uri, urdf_content)
    return modified_content


def _interactive_pick(label, choices, default):
    """Prompt the user to pick one of `choices`. Returns the chosen string.

    - If a value is already supplied via `<label>:=...` on the command line,
      that value is used and no prompt is shown.
    - If stdin/stdout is not a TTY (e.g. launch from another launch file),
      the default is used silently.
    - Empty input -> default. Invalid input -> re-prompt.
    """
    prefix = f'{label}:='
    for a in sys.argv:
        if a.startswith(prefix):
            return a.split(':=', 1)[1]
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        return default
    print()
    print(f'  Select a {label}:')
    for i, name in enumerate(choices, 1):
        marker = '  (default)' if name == default else ''
        print(f'    [{i}] {name}{marker}')
    while True:
        try:
            raw = input(f'  Enter number 1-{len(choices)} or name [default: {default}]: ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return default
        if raw == '':
            return default
        if raw.isdigit() and 1 <= int(raw) <= len(choices):
            return choices[int(raw) - 1]
        if raw in choices:
            return raw
        print(f'  ! Not a valid choice. Try a number 1-{len(choices)} or one of: {", ".join(choices)}')


def generate_launch_description():
    # Arguments
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

    # Discover available worlds in the installed worlds/ directory so the user
    # gets a helpful list if they pick a name that doesn't exist.
    sim_gazebo_bringup_dir_early = get_package_share_directory('sim_gazebo_bringup')
    worlds_dir = os.path.join(sim_gazebo_bringup_dir_early, 'worlds')
    available_worlds = sorted(
        os.path.splitext(f)[0]
        for f in os.listdir(worlds_dir)
        if f.endswith('.sdf')
    )

    # Interactive picker (only if world:= not provided and stdin is a TTY).
    requested_world = _interactive_pick('world', available_worlds, 'empty')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=requested_world,
        description=(
            'World to load (basename without .sdf, or absolute path to a .sdf file). '
            'Available: ' + ', '.join(available_worlds)
        ),
    )

    # Get package shares
    try:
        ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    except Exception as e:
        raise Exception(
            "ros_gz_sim package not found. Please install it with:\n"
            "  sudo apt-get install ros-humble-ros-gz-sim\n"
            "Or use the RViz-only version instead:\n"
            "  ros2 launch sim_gazebo_bringup robot_rviz.launch.py\n"
            f"Error: {e}"
        )
    
    yahboomcar_description_dir = get_package_share_directory('yahboomcar_description')
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    
    # Paths
    xacro_file = os.path.join(yahboomcar_description_dir, 'urdf', 'yahboomcar_X3plus.urdf.xacro')
    rviz_config_file = os.path.join(yahboomcar_description_dir, 'rviz', 'yahboomcar.rviz')

    # Resolve `world` (already chosen interactively above unless world:= was
    # passed; either way `requested_world` holds the basename / abs path).
    if os.path.isabs(requested_world) and os.path.isfile(requested_world):
        world_file = requested_world
    else:
        candidate = os.path.join(sim_gazebo_bringup_dir, 'worlds', requested_world + '.sdf')
        if not os.path.isfile(candidate):
            raise RuntimeError(
                f"World '{requested_world}' not found. "
                f"Available: {', '.join(available_worlds)}. "
                f"Pass world:=<name> or an absolute path to a .sdf file."
            )
        world_file = candidate
    print(f'[sim_gazebo_bringup] Loading world: {world_file}')

    # Get configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Process XACRO file to generate URDF with proper settings
    try:
        # Run xacro to convert .urdf.xacro to .urdf
        # Pass ns argument explicitly - empty string means no namespace prefix
        result = subprocess.run(
            ['xacro', xacro_file, 'ns:='],
            capture_output=True,
            text=True,
            check=True
        )
        robot_description_content = result.stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to process XACRO file with xacro: {e.stderr}")
    except FileNotFoundError:
        raise RuntimeError(
            "xacro command not found. Please install: "
            "sudo apt install ros-humble-xacro"
        )
    
    # Convert package:// URIs to absolute paths so Gazebo can find mesh files
    robot_description_content = convert_package_uris_to_absolute_paths(robot_description_content)
    
    # Fix link/joint names: handle XACRO expansion with empty namespace
    # When ns="", the pattern ${ns}/ becomes just /, so we get "/base_link" instead of "base_link"
    # We need to remove ANY remaining leading slashes from attribute values
    robot_description_content = re.sub(r'name="/', r'name="', robot_description_content)
    robot_description_content = re.sub(r'parent="/', r'parent="', robot_description_content)
    robot_description_content = re.sub(r'child="/', r'child="', robot_description_content)
    robot_description_content = re.sub(r'link="/', r'link="', robot_description_content)
    robot_description_content = re.sub(r'reference="/', r'reference="', robot_description_content)
    robot_description_content = re.sub(r'joint="/', r'joint="', robot_description_content)

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    # NOTE: We intentionally do NOT spawn the static joint_state_publisher here.
    # In simulation, the JointStatePublisher Ignition system inside the URDF publishes
    # the actual physics joint positions to /world/empty/model/x3plus/joint_state, and
    # the ros_gz_bridge below forwards them to /joint_states. Running both publishers
    # would race and overwrite each other.

    # Ignition embeds the world name in the joint_state topic. Derive it from
    # the chosen world file (basename without extension) so switching worlds
    # via world:=<name> doesn't silently break the joint_state bridge.
    world_name = os.path.splitext(os.path.basename(world_file))[0]
    joint_state_topic = f'/world/{world_name}/model/x3plus/joint_state'

    # ros_gz bridge: forward arm/gripper position commands from ROS to Ignition,
    # and joint states + clock from Ignition back to ROS.
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Position commands: ROS std_msgs/Float64 -> Ignition Double
            '/arm_joint1_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/arm_joint2_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/arm_joint3_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/arm_joint4_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/arm_joint5_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            '/grip_joint_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
            # Differential drive: ROS Twist -> Ignition Twist
            # The DiffDrive plugin in the URDF subscribes on the model-prefixed
            # topic (it ignores leading-slash overrides). We bridge that and
            # remap to /cmd_vel below so user nodes can keep using /cmd_vel.
            '/model/x3plus/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Odometry: Ignition -> ROS (also remapped to /odom below)
            '/model/x3plus/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # IMU: Ignition -> ROS. Used by manual_control's closed-loop 90° turn
            # to read real chassis yaw (wheel-odom yaw is wrong when wheels slip).
            '/model/x3plus/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Joint states: Ignition Model -> ROS sensor_msgs/JointState
            f'{joint_state_topic}@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Simulation clock: Ignition -> ROS
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            # User-facing topic names: keep /cmd_vel and /odom on the ROS side
            # while the bridge actually talks to the model-prefixed Ignition
            # topics that the DiffDrive plugin uses.
            ('/model/x3plus/cmd_vel', '/cmd_vel'),
            ('/model/x3plus/odometry', '/odom'),
            ('/model/x3plus/imu', '/imu'),
            # Route raw physics joint states to /joint_states_raw.
            # The gripper_mimic_relay node filters out the frozen mimic joints
            # and republishes to /joint_states so robot_state_publisher can
            # compute finger positions via the URDF <mimic> relationship.
            (joint_state_topic, '/joint_states_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo Sim launch (using ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file]
        }.items()
    )

    # Set environment variables for Gazebo to find models and resources
    set_gazebo_model_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        yahboomcar_description_dir
    )
    
    set_gazebo_model_path2 = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        yahboomcar_description_dir
    )

    # Create/spawn robot in Gazebo Sim
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'x3plus'
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Mimic joint relay: strips passive finger joints from the raw Ignition
    # joint_states so robot_state_publisher computes them via URDF <mimic>.
    gripper_mimic_relay_node = Node(
        package='x3plus_examples',
        executable='gripper_mimic_relay',
        name='gripper_mimic_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Build launch description
    launch_desc = [
        use_sim_time_arg,
        use_rviz_arg,
        world_arg,
        set_gazebo_model_path,
        set_gazebo_model_path2,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        ros_gz_bridge_node,
        gripper_mimic_relay_node,
    ]
    
    # Note: RViz is not included in the default launch to match use_rviz:=false default parameter
    # Users can launch RViz separately if needed via:
    #   ros2 launch sim_gazebo_bringup robot_rviz.launch.py
    # 
    # For Humble compatibility, we don't conditionally include RViz here
    # (IfAction doesn't exist in Humble's launch module)
    
    return LaunchDescription(launch_desc)
