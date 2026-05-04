#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # ---------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz2 with MoveIt plugin"),
        DeclareLaunchArgument(
            "publish_robot_description", default_value="true",
            description="Whether to publish robot_description param (URDF)"),
        DeclareLaunchArgument(
            "publish_robot_description_semantic", default_value="true",
            description="Whether to publish robot_description_semantic param (SRDF)"),
        DeclareLaunchArgument(
            "load_robot_state_publisher", default_value="true",
            description="Whether to start robot_state_publisher"),
    ]

    # ---------------------------------------------------------------------
    # File paths
    # ---------------------------------------------------------------------
    pkg_share = get_package_share_directory("x3plus_config")

    urdf_file = os.path.join(pkg_share, "config", "x3plus.urdf")
    srdf_file = os.path.join(pkg_share, "config", "x3plus.srdf")
    kinematics_yaml = os.path.join(pkg_share, "config", "kinematics.yaml")
    ompl_yaml = os.path.join(pkg_share, "config", "ompl_planning.yaml")
    controllers_yaml = os.path.join(pkg_share, "config", "moveit_controllers.yaml")

    # ---------------------------------------------------------------------
    # Configurations
    # ---------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    load_robot_state_publisher = LaunchConfiguration("load_robot_state_publisher")

    # ---------------------------------------------------------------------
    # Nodes
    # ---------------------------------------------------------------------
    nodes_to_start = []

    # Robot State Publisher (optional)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": open(urdf_file).read()}
        ],
        condition=IfCondition(load_robot_state_publisher)
    )
    nodes_to_start.append(rsp_node)

    # Move Group (main MoveIt process)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": open(urdf_file).read()},
            {"robot_description_semantic": open(srdf_file).read()},
            ompl_yaml,
            kinematics_yaml,
            controllers_yaml,
        ],
    )
    nodes_to_start.append(move_group_node)

    # RViz2 (optional visualization)
    rviz_config = os.path.join(pkg_share, "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes_to_start.append(rviz_node)

    # ---------------------------------------------------------------------
    # Final LaunchDescription
    # ---------------------------------------------------------------------
    return LaunchDescription(declared_arguments + nodes_to_start)




# --- Old code ---
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# import os

# def generate_launch_description():
#     ns = LaunchConfiguration("ns")
#     declare_ns = DeclareLaunchArgument("ns", default_value="x3plus")

#     # Absolute paths to your model files
#     desc_share = get_package_share_directory("yahboomcar_description")
#     xacro_path = os.path.join(desc_share, "urdf", "yahboomcar_X3plus.urdf.xacro")

#     cfg_share = get_package_share_directory("x3plus_config")
#     # Adjust the SRDF filename if yours differs (check x3plus_config/config/)
#     srdf_path = os.path.join(cfg_share, "config", "yahboomcar_X3plus.srdf")

#     moveit_config = (
#         MoveItConfigsBuilder("yahboomcar_X3plus", package_name="x3plus_config")
#         # Feed the URDF *from xacro* with the required arg(s)
#         .robot_description(file_path=xacro_path, mappings={"ns": ns})
#         # Feed the SRDF explicitly
#         .robot_description_semantic(file_path=srdf_path)
#         # (Optional) load kinematics/OMPL/controllers if your files exist
#         # .trajectory_execution(file_path=os.path.join(cfg_share, "config", "controllers.yaml"))
#         # .planning_pipelines(pipelines=["ompl"])
#         .to_moveit_configs()
#     )

#     return LaunchDescription([declare_ns, generate_demo_launch(moveit_config)])



# --- Old auotgenerated demo code ---
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("yahboomcar_X3plus", package_name="x3plus_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)
