#!/usr/bin/env python3
"""
Fallback launch file: Display x3plus robot in RViz without Gazebo simulation.

This is useful for:
- Testing robot URDF visualization
- Checking robot configuration without simulation
- When Gazebo/ros_gz_sim is not installed

Usage:
    ros2 launch sim_gazebo_bringup robot_rviz.launch.py

Optional arguments:
    use_sim_time:=true               - Use simulated time (default: false)
    map:=<name>                      - Map to load. Options come from the
                                       maps/ directory of this package.
                                       Defaults to 'plain_map'. Examples:
                                         map:=plain_map
                                         map:=turtlebot3_world
                                         map:=willow
                                       You may also pass an absolute path to
                                       a .yaml map file.
"""

import os
import re
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def convert_package_uris_to_file_uris(urdf_content):
    """
    Convert package:// URIs in URDF to file:// URIs for RViz compatibility.
    
    RViz can have issues resolving package:// URIs. Using file:// URIs ensures
    meshes load correctly.
    
    Example:
        package://yahboomcar_description/meshes/X3plus/visual/base_link.STL
        becomes:
        file:///path/to/install/yahboomcar_description/share/yahboomcar_description/meshes/X3plus/visual/base_link.STL
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
                # Convert to file:// URI (must have 3 slashes: file:/// for absolute paths)
                # Ensure path starts with / for proper file:/// format
                if not absolute_path.startswith('/'):
                    absolute_path = '/' + absolute_path
                file_uri = f'file://{absolute_path}'
                return file_uri
            except Exception as e:
                print(f"Warning: Could not resolve package {package_name}: {e}")
                return package_uri
        return package_uri
    
    # Replace all package:// URIs with file:// URIs
    modified_content = re.sub(r'package://[^"\'<\s]+', replace_package_uri, urdf_content)
    return modified_content


def _interactive_pick(label, choices, default):
    """Prompt the user to pick one of `choices`. See gazebo.launch.py for behaviour."""
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

# Fix for snap libc/pthread conflicts on some Ubuntu systems
# When ROS is installed via snap, it may try to load incompatible snap libc libraries
# Preload system libc to override this behavior
def get_env_vars():
    """Get environment variables to fix snap libc conflicts and enable display"""
    env = os.environ.copy()
    
    # Preload system libc libraries to override snap versions
    ld_preload = "/lib/x86_64-linux-gnu/libc.so.6:/lib/x86_64-linux-gnu/libpthread.so.0"
    if 'LD_PRELOAD' in env:
        env['LD_PRELOAD'] = ld_preload + ":" + env['LD_PRELOAD']
    else:
        env['LD_PRELOAD'] = ld_preload
    # Ensure system lib directories are searched before snap libs
    system_libs = "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
    if 'LD_LIBRARY_PATH' in env and env['LD_LIBRARY_PATH']:
        env['LD_LIBRARY_PATH'] = system_libs + ":" + env['LD_LIBRARY_PATH']
    else:
        env['LD_LIBRARY_PATH'] = system_libs
    
    # Ensure DISPLAY and WAYLAND_DISPLAY are passed through for GUI apps
    # This allows RViz and joint_state_publisher_gui to connect to the display server
    if 'DISPLAY' not in env and os.environ.get('DISPLAY'):
        env['DISPLAY'] = os.environ['DISPLAY']
    if 'WAYLAND_DISPLAY' not in env and os.environ.get('WAYLAND_DISPLAY'):
        env['WAYLAND_DISPLAY'] = os.environ['WAYLAND_DISPLAY']
    
    # Add X11 socket and runtime directory for snap compatibility
    # The snap RViz needs access to the host's display socket
    if 'XAUTHORITY' not in env and os.environ.get('XAUTHORITY'):
        env['XAUTHORITY'] = os.environ['XAUTHORITY']
    elif 'XAUTHORITY' not in env:
        env['XAUTHORITY'] = os.path.expanduser('~/.Xauthority')
    
    if 'XDG_RUNTIME_DIR' not in env and os.environ.get('XDG_RUNTIME_DIR'):
        env['XDG_RUNTIME_DIR'] = os.environ['XDG_RUNTIME_DIR']
    
    # Force QT to use X11 platform
    env['QT_QPA_PLATFORM'] = 'xcb'
    
    return env

def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get package shares
    yahboomcar_description_dir = get_package_share_directory('yahboomcar_description')
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')

    # Discover available maps in the installed maps/ directory so the user gets
    # a helpful list if they pick a name that doesn't exist.
    maps_dir = os.path.join(sim_gazebo_bringup_dir, 'maps')
    if os.path.isdir(maps_dir):
        available_maps = sorted(
            os.path.splitext(f)[0]
            for f in os.listdir(maps_dir)
            if f.endswith('.yaml')
        )
    else:
        available_maps = []
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='plain_map',
        description=(
            'Map to load (basename without .yaml, or absolute path to a .yaml file). '
            'Available: ' + (', '.join(available_maps) if available_maps else '(none)')
        ),
    )

    # Interactive picker (only if map:= not provided and stdin is a TTY).
    requested_map = _interactive_pick(
        'map', available_maps if available_maps else ['plain_map'], 'plain_map'
    )

    if os.path.isabs(requested_map) and os.path.isfile(requested_map):
        plain_map_file = requested_map
    else:
        plain_map_file = os.path.join(maps_dir, requested_map + '.yaml')
        if not os.path.isfile(plain_map_file):
            raise RuntimeError(
                f"Map '{requested_map}' not found. "
                f"Available: {', '.join(available_maps)}. "
                f"Pass map:=<name> or an absolute path to a .yaml file."
            )
    print(f'[sim_gazebo_bringup] Loading map: {plain_map_file}')
    
    # Paths
    xacro_file = os.path.join(yahboomcar_description_dir, 'urdf', 'yahboomcar_X3plus.urdf.xacro')
    rviz_config_file = os.path.join(sim_gazebo_bringup_dir, 'rviz', 'gazebo_view.rviz')

    # Get configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get environment variables for snap libc fix
    env_vars = get_env_vars()

    # Export LD_PRELOAD into the launch process environment so child processes inherit it
    os.environ.update(env_vars)

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
    
    # Convert package:// URIs to file:// URIs for RViz mesh loading
    # RViz can have trouble with package:// URIs, file:// works more reliably
    robot_description_content = convert_package_uris_to_file_uris(robot_description_content)
    
    # Fix link/joint names: remove leading slashes from names
    # XACRO with empty ns="/" (/$) generates "/base_link" instead of "base_link"
    # This happens because properties use ${ns}/link_name which becomes //link_name (double slash)
    # after removing, we get just /link_name. We need to remove these leading slashes.
    # NOTE: This only affects TF frame names, not file paths (which were already converted above)
    robot_description_content = re.sub(r' name="/', r' name="', robot_description_content)
    robot_description_content = re.sub(r' parent="/', r' parent="', robot_description_content)
    robot_description_content = re.sub(r' child="/', r' child="', robot_description_content)
    robot_description_content = re.sub(r' link="/', r' link="', robot_description_content)
    robot_description_content = re.sub(r' reference="/', r' reference="', robot_description_content)
    robot_description_content = re.sub(r' joint="/', r' joint="', robot_description_content)

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


    # Joint states are now published by diff_drive_simulator (same timer as TF)
    # to eliminate timestamp jitter between separate publishers

    # RViz process launched via ExecuteProcess with proper environment variables
    rviz_proc = ExecuteProcess(
        cmd=['/opt/ros/humble/lib/rviz2/rviz2', '-d', rviz_config_file],
        name='rviz2',
        output='screen',
        env=env_vars
    )

    # Differential Drive Simulator - Processes cmd_vel and updates robot pose in TF
    # This allows the robot to move in RViz when velocity commands are published
    diff_drive_sim_node = Node(
        package='x3plus_examples',
        executable='diff_drive_simulator',
        name='diff_drive_simulator',
        output='screen'
    )

    # Map Publisher - Loads and publishes map from files
    # Displays the map in RViz so you can see robot movement in context
    # (path resolved above from the `map` launch argument)
    
    map_publisher_node = Node(
        package='x3plus_examples',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        arguments=['--map-path', plain_map_file]
    )

    # Static Transform: map to odom
    # This establishes the connection between the global map frame and the odometry frame
    # The robot starts at the origin of the map
    map_to_odom_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0', '0', '0',           # x, y, z translation
            '0', '0', '0', '1',       # x, y, z, w quaternion (identity = no rotation)
            'map', 'odom'              # parent frame, child frame
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        robot_state_publisher_node,
        rviz_proc,
        diff_drive_sim_node,
        map_publisher_node,
        map_to_odom_broadcaster,
    ])
