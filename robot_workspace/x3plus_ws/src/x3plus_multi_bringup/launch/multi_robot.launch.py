#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from x3plus_multi_bringup.scripts.rviz_generator import make_rviz_config
from nav2_common.launch import RewrittenYaml

# ---------------- helpers ----------------
def _ns(prefix: str, rid: str) -> str:
    """Return the namespace string, e.g. ('robot','123') -> 'robot123'."""
    return f"{prefix}{rid}"

def _cat(ns_name: str, frame_cfg):
    """Build a single substitution for '<ns>_<frame>' (keeps TF parent/child atomic)."""
    return [TextSubstitution(text=f"{ns_name}_"), frame_cfg]

# --------------- per-robot stack ---------------
def _per_robot_group(rid: str, prefix: str):
    """Create the per-robot namespaced Nav2 stack and helpers."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame   = LaunchConfiguration('base_frame')
    odom_frame   = LaunchConfiguration('odom_frame')

    pkg_share     = FindPackageShare('x3plus_multi_bringup')
    nav2_bt_share = PathJoinSubstitution([FindPackageShare('nav2_bt_navigator'), 'behavior_trees'])
    urdf_xacro    = PathJoinSubstitution([pkg_share, 'urdf', 'yahboomcar_X3plus.urdf.xacro'])
    initial_positions_file = PathJoinSubstitution([pkg_share, 'config', 'initial_positions.yaml'])

    ns_name        = _ns(prefix, rid)
    namespaced_base = _cat(ns_name, base_frame)
    namespaced_odom = _cat(ns_name, odom_frame)

    robot_description = Command(['xacro ', urdf_xacro, 
                                 ' ', TextSubstitution(text=f'prefix:={ns_name}_'),
                                 ' ', 'initial_positions_file:=', initial_positions_file])

    # Upstream Nav2 defaults, then rewrite only what’s frame/time-specific
    nav2_default_params = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml'])

    # Use upstream params; static map is relayed into each namespace by _compose()
    robots_params = RewrittenYaml(
        source_file=nav2_default_params,
        root_key='',
        param_rewrites={
            # --- bt_navigator ---
            'bt_navigator.ros__parameters.global_frame': 'map',
            'bt_navigator.ros__parameters.robot_base_frame': namespaced_base,
            'bt_navigator.ros__parameters.odom_frame': namespaced_odom,
            'bt_navigator.ros__parameters.use_sim_time': use_sim_time,

            # --- controller_server ---
            'controller_server.ros__parameters.odom_frame': namespaced_odom,
            'controller_server.ros__parameters.robot_base_frame': namespaced_base,
            'controller_server.ros__parameters.use_sim_time': use_sim_time,

            # --- planner_server ---
            'planner_server.ros__parameters.global_frame': 'map',
            'planner_server.ros__parameters.robot_base_frame': namespaced_base,
            'planner_server.ros__parameters.odom_frame': namespaced_odom,
            'planner_server.ros__parameters.use_sim_time': use_sim_time,

            # --- behavior_server ---
            'behavior_server.ros__parameters.global_frame': 'map',
            'behavior_server.ros__parameters.robot_base_frame': namespaced_base,
            'behavior_server.ros__parameters.odom_frame': namespaced_odom,
            'behavior_server.ros__parameters.use_sim_time': use_sim_time,

            # --- costmaps ---
            'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
            'global_costmap.global_costmap.ros__parameters.robot_base_frame': namespaced_base,
            'global_costmap.global_costmap.ros__parameters.use_sim_time': use_sim_time,

            'local_costmap.local_costmap.ros__parameters.global_frame': namespaced_odom,
            'local_costmap.local_costmap.ros__parameters.robot_base_frame': namespaced_base,
            'local_costmap.local_costmap.ros__parameters.use_sim_time': use_sim_time,

            # --- other servers ---
            'smoother_server.ros__parameters.use_sim_time': use_sim_time,
            'waypoint_follower.ros__parameters.use_sim_time': use_sim_time,
            'velocity_smoother.ros__parameters.use_sim_time': use_sim_time,

            # --- default BTs---
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                PathJoinSubstitution([nav2_bt_share, 'navigate_to_pose_w_replanning_time.xml']),
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                PathJoinSubstitution([nav2_bt_share, 'navigate_through_poses_w_replanning_time.xml']),
        },
        convert_types=True
    )

    return GroupAction([
        PushRosNamespace(TextSubstitution(text=ns_name)),

        # Namespaced static TF: map -> <ns>_odom published to /<ns>/tf_static
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_odom_identity_{rid}',
            arguments=[
                '--x','0','--y','0','--z','0',
                '--yaw','0','--pitch','0','--roll','0',
                '--frame-id','map',
                '--child-frame-id', _cat(ns_name, LaunchConfiguration('odom_frame'))
            ],
            output='screen',
            remappings=[('/tf','tf'),('/tf_static','tf_static')],
        ),

        # RSP writes namespaced TF. Only joint_states remap to relative topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'publish_frequency': 50.0,
                'ignore_timestamp': True,
                'qos_overrides./joint_states.subscription.reliability': 'reliable',
                'qos_overrides./joint_states.subscription.depth': 50,
            }],
            # Keep TF topics namespaced; joint_states remains namespaced
            remappings=[
                ('/joint_states','joint_states'),
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ],
        ),

        # Choose GUI or non GUI. Here GUI only
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'publish_default_positions': True,
                'rate': 30.0,
                'prefix': TextSubstitution(text=f'{ns_name}_'),
            }],
            remappings=[('/joint_states','joint_states')],
        ),

        # Odom integrator publishes <ns>_odom -> <ns>_<base> into /<ns>/tf
        Node(
            package='x3plus_multi_bringup',
            executable='odom_integrator',
            name='odom_integrator',
            output='screen',
            parameters=[{
                'odom_frame': namespaced_odom,
                'base_frame': namespaced_base,
            }],
            remappings=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ],
        ),

        # Nav2 stack in this namespace
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'namespace': TextSubstitution(text=ns_name),
                'use_sim_time': use_sim_time,
                'params_file': robots_params,
                'autostart': 'true'
            }.items()
        ),
    ])

# ---------------- compose everything ----------------
def _compose(context, *args, **kwargs):
    """Runtime composition: one global map server + per-robot stacks + RViz."""
    robots_id_csv = (LaunchConfiguration('robots_id').perform(context) or '').strip()
    prefix        = (LaunchConfiguration('prefix').perform(context) or 'robot').strip()
    use_rviz      = LaunchConfiguration('rviz')
    map_yaml      = LaunchConfiguration('map')

    robot_ids = [s.strip() for s in robots_id_csv.split(',') if s.strip()]
    actions = []

    # 1) Global map server
    actions += [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': map_yaml,
                'frame_id': 'map',
                'topic_name': 'map'
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
    ]

    # 2) Per robot: map relays, namespaced stack, outward TF relays for RViz
    for rid in robot_ids:
        ns_name = _ns(prefix, rid)

        # Map relays into each namespace with transient local
        actions.append(
            Node(
                package='topic_tools',
                executable='relay',
                name=f'map_relay_{rid}',
                arguments=['/map', f'/{ns_name}/map'],
                parameters=[{
                    'history': 'keep_last',
                    'depth': 1,
                    'reliability': 'reliable',
                    'durability': 'transient_local',
                }],
                output='screen',
            )
        )
        actions.append(
            Node(
                package='topic_tools',
                executable='relay',
                name=f'map_updates_relay_{rid}',
                arguments=['/map_updates', f'/{ns_name}/map_updates'],
                parameters=[{
                    'history': 'keep_last',
                    'depth': 1,
                    'reliability': 'reliable',
                    'durability': 'transient_local',
                    'lazy': True,
                }],
                output='screen',
            )
        )
        # # ---- ROS2 controller spawners ----
        # actions.append(
        #         Node(
        #         package="controller_manager",
        #         executable="spawner",
        #         arguments=["joint_state_broadcaster"],
        #         namespace=ns_name,
        #     )
        # )
        # actions.append(
        #     Node(
        #         package="controller_manager",
        #         executable="spawner",
        #         arguments=[
        #             "arm_group_controller",
        #             "--param-file",
        #             PathJoinSubstitution([
        #                 FindPackageShare("x3plus_multi_bringup"),
        #                 "config",
        #                 "arm_group_controller.yaml",
        #             ]),
        #         ],
        #         namespace=ns_name,
        #     )
        # )
        # actions.append(
        #     Node(
        #         package="controller_manager",
        #         executable="spawner",
        #         arguments=[
        #             "gripper_group_controller",
        #             "--param-file",
        #             PathJoinSubstitution([
        #                 FindPackageShare("x3plus_multi_bringup"),
        #                 "config",
        #                 "gripper_group_controller.yaml",
        #             ]),
        #         ],
        #         namespace=ns_name,
        #     )
        # )

        # Per robot stack once
        actions.append(_per_robot_group(rid, prefix))

        # Outward: namespaced -> global (for RViz)
        actions.append(Node(
            package='topic_tools', executable='relay',
            name=f'tf_{ns_name}_to_global',
            arguments=[f'/{ns_name}/tf', '/tf'],
            output='screen'))

        actions.append(Node(
            package='topic_tools', executable='relay',
            name=f'tf_static_{ns_name}_to_global',
            arguments=[f'/{ns_name}/tf_static', '/tf_static'],
            output='screen'))

    # 3) Single RViz
    rviz_file = make_rviz_config(robot_ids, prefix)
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_multi',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=IfCondition(use_rviz),
        )
    )

    return actions

def generate_launch_description():
    """Top-level multi-robot launcher: one map, N robots, optional RViz."""
    return LaunchDescription([
        # User-facing arguments
        DeclareLaunchArgument('robots_id', description='Comma-separated IDs, e.g. "123,456"'),
        DeclareLaunchArgument('prefix', default_value='robot'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='Absolute path to map.yaml'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),  # simple true/false

        # Build actions at runtime (depends on robots_id list)
        OpaqueFunction(function=_compose),
    ])
