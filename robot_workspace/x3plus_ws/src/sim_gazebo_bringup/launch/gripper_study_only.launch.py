#!/usr/bin/env python3
import os
import re
import subprocess
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
    urdf_path = os.path.join(sim_gazebo_bringup_dir, 'urdf', 'yahboomcar_X3plus.urdf.xacro')
    result = subprocess.run(['xacro', urdf_path, 'ns:='], capture_output=True, text=True, check=True)
    urdf_content = result.stdout
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
    return LaunchDescription([robot_state_publisher])
