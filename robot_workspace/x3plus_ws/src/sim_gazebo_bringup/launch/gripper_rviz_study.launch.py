#!/usr/bin/env python3
"""
Launch file to study the gripper mechanism in RViz.
- Loads the URDF
- Runs robot_state_publisher
- Runs joint_state_publisher_gui (so you can manually move the gripper joints)
- Runs RViz with the gripper view config
"""
import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def convert_package_uris_to_file_uris(urdf_content):
    def replace_package_uri(match):
        package_uri = match.group(0)
        match_parts = re.match(r'package://([^/]+)/(.*)', package_uri)
        if match_parts:
            package_name = match_parts.group(1)
            relative_path = match_parts.group(2)
            try:
                package_share_dir = get_package_share_directory(package_name)
                absolute_path = os.path.join(package_share_dir, relative_path)
                if not absolute_path.startswith('/'):
                    absolute_path = '/' + absolute_path
                return f'file://{absolute_path}'
            except Exception:
                return package_uri
        return package_uri
    return re.sub(r'package://[^"\'<\s]+', replace_package_uri, urdf_content)


def generate_launch_description():
    sim_gazebo_bringup_dir = get_package_share_directory('sim_gazebo_bringup')
    urdf_path = os.path.join(
        sim_gazebo_bringup_dir, 'urdf', 'yahboomcar_X3plus.urdf.xacro')

    # Process the xacro to URDF first (so ${pi/2} and other xacro
    # expressions are resolved). joint_state_publisher_gui and
    # robot_state_publisher do NOT run xacro themselves.
    import subprocess
    result = subprocess.run(
        ['xacro', urdf_path, 'ns:='],
        capture_output=True, text=True, check=True,
    )
    urdf_content = result.stdout
    # The xacro emits names like "/arm_joint5" when ns is empty.  Strip the
    # leading slash from every name attribute so the link/joint tokens
    # line up.  This is the same workaround the multi-robot launch uses.
    urdf_content = re.sub(r'(?<=name=")/', '', urdf_content)
    urdf_content = re.sub(r'(?<=parent=")/', '', urdf_content)
    urdf_content = re.sub(r'(?<=child=")/', '', urdf_content)
    urdf_content = re.sub(r'(?<=link=")/', '', urdf_content)
    urdf_content = re.sub(r'(?<=reference=")/', '', urdf_content)
    urdf_content = re.sub(r'(?<=mimic joint=")/', '', urdf_content)
    urdf_content = convert_package_uris_to_file_uris(urdf_content)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # Use the non-GUI joint_state_publisher so external nodes can drive
    # the joint values via /joint_states messages.  The GUI version
    # locks the values to its slider setpoints.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': False,
            'publish_default_positions': True,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
