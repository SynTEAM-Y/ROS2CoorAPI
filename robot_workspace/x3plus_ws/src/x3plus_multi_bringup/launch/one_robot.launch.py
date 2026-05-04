#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Launch arguments (set via: ros2 launch ... arg:=value)
    robot_id     = LaunchConfiguration('robot_id')       # Unique numeric/alpha ID for this robot instance
    use_sim_time = LaunchConfiguration('use_sim_time')   # Use /clock (Gazebo/bag) if true
    map_yaml     = LaunchConfiguration('map')            # Absolute path to map.yaml
    base_frame   = LaunchConfiguration('base_frame')     # e.g., base_footprint or base_link
    odom_frame   = LaunchConfiguration('odom_frame')     # e.g., odom
    use_rviz     = LaunchConfiguration('rviz')           # Toggle RViz

    # Package shares / paths
    pkg_share          = FindPackageShare('x3plus_multi_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    nav2_bt_share      = PathJoinSubstitution([FindPackageShare('nav2_bt_navigator'), 'behavior_trees'])

    urdf_xacro  = PathJoinSubstitution([pkg_share, 'urdf', 'yahboomcar_X3plus.urdf.xacro'])
    nav2_launch = PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'sim_view.rviz'])

    # Namespaced frame IDs: robot{ID}_<frame> (e.g., robot12_base_footprint, robot12_odom)
    robot_prefix     = [TextSubstitution(text='robot'), robot_id, TextSubstitution(text='_')]
    namespaced_base  = robot_prefix + [base_frame]
    namespaced_odom  = robot_prefix + [odom_frame]

    # Start from upstream defaults and rewrite frames/time/BT XMLs for this robot namespace
    nav2_default_params = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml'
    ])

    robots_params = RewrittenYaml(
        source_file=nav2_default_params,
        root_key='',
        param_rewrites={
            # bt_navigator
            'bt_navigator.ros__parameters.global_frame': 'map',
            'bt_navigator.ros__parameters.robot_base_frame': namespaced_base,
            'bt_navigator.ros__parameters.odom_topic': 'odom',
            'bt_navigator.ros__parameters.use_sim_time': use_sim_time,

            # controller_server
            'controller_server.ros__parameters.odom_frame': namespaced_odom,
            'controller_server.ros__parameters.robot_base_frame': namespaced_base,
            'controller_server.ros__parameters.use_sim_time': use_sim_time,

            # costmaps
            'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
            'global_costmap.global_costmap.ros__parameters.robot_base_frame': namespaced_base,
            'global_costmap.global_costmap.ros__parameters.use_sim_time': use_sim_time,

            'local_costmap.local_costmap.ros__parameters.global_frame': namespaced_odom,
            'local_costmap.local_costmap.ros__parameters.robot_base_frame': namespaced_base,
            'local_costmap.local_costmap.ros__parameters.use_sim_time': use_sim_time,

            # misc nodes
            'planner_server.ros__parameters.use_sim_time': use_sim_time,
            'smoother_server.ros__parameters.use_sim_time': use_sim_time,
            'behavior_server.ros__parameters.use_sim_time': use_sim_time,
            'waypoint_follower.ros__parameters.use_sim_time': use_sim_time,
            'velocity_smoother.ros__parameters.use_sim_time': use_sim_time,

            # default BTs
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                PathJoinSubstitution([nav2_bt_share, 'navigate_to_pose_w_replanning_time.xml']),
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                PathJoinSubstitution([nav2_bt_share, 'navigate_through_poses_w_replanning_time.xml']),
        },
        convert_types=True
    )

    # Robot description (URDF via xacro) with per-robot prefix "robot{ID}_"
    robot_description = Command([
        'xacro ', urdf_xacro,
        ' ', TextSubstitution(text='prefix:=robot'), robot_id, TextSubstitution(text='_')
    ])

    return LaunchDescription([
        # Public arguments
        DeclareLaunchArgument('robot_id'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='Absolute path to map.yaml'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # Global static TF: map -> robot{ID}_odom (one per robot; not namespaced)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=['0','0','0','0','0','0','map', namespaced_odom],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            output='screen',
        ),

        # Bridge global TF topics into namespaced TF topics (some Nav2 internals expect /robot{ID}/tf*)
        Node(
            package='topic_tools',
            executable='relay',
            name='tf_bridge_global_to_ns',
            arguments=['/tf', [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/tf')]],
            output='screen'
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='tf_static_bridge_global_to_ns',
            arguments=['/tf_static', [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/tf_static')]],
            output='screen'
        ),

        # Per-robot namespaced stack
        PushRosNamespace([TextSubstitution(text='robot'), robot_id]),

        # robot_state_publisher: publishes TF using the prefixed URDF (publishes to global TF topics)
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
                # Robust subscription to joint states
                'qos_overrides./joint_states.subscription.reliability': 'reliable',
                'qos_overrides./joint_states.subscription.depth': 50,
            }],
            remappings=[
                ('tf','/tf'), ('tf_static','/tf_static'),
                ('/joint_states','joint_states'),
            ],
        ),

        # joint_state_publisher_gui: provides joint states in this namespace (optional for sim/demo)
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
                'prefix': robot_prefix,
            }],
            remappings=[('/joint_states','joint_states')],
        ),

        # Odom integrator: publishes robot{ID}_odom -> robot{ID}_{base_frame}
        Node(
            package='x3plus_multi_bringup',
            executable='odom_integrator',
            name='odom_integrator',
            output='screen',
            parameters=[{
                'odom_frame': namespaced_odom,
                'base_frame': namespaced_base,
            }],
            remappings=[('tf','/tf'), ('tf_static','/tf_static')],
        ),

        # TEMP identity TF: odom -> base (remove after verifying odom_integrator TF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_identity',
            arguments=['0','0','0','0','0','0', namespaced_odom, namespaced_base],
            remappings=[('tf','/tf'), ('tf_static','/tf_static')],
            output='screen',
        ),

        # Static map server (namespace-scoped)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'frame_id': 'map',
                'topic_name': 'map'
            }],
        ),

        # Lifecycle manager for map_server only (brings it to active state)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        # Nav2 core stack (planner/controller/BT) under this namespace with rewritten params
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'namespace': [TextSubstitution(text='robot'), robot_id],
                'use_sim_time': use_sim_time,
                'params_file': robots_params,
                'autostart': 'true'
            }.items()
        ),

        # RViz: connect to namespaced topics; TF remains global
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='',
            name=['rviz_', robot_id],
            arguments=['-d', rviz_config],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description},
            ],
            remappings=[
                ('map',               [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/map')]),
                ('scan',              [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/scan')]),
                ('robot_description', [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/robot_description')]),
                ('odom',              [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/odom')]),
                ('cmd_vel',           [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/cmd_vel')]),
                ('/map',              [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/map')]),
                ('/map_updates',      [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/map_updates')]),
                ('/robot_description',[TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/robot_description')]),
                ('/initialpose',      [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/initialpose')]),
                ('/goal_pose',        [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/goal_pose')]),
                ('/clicked_point',    [TextSubstitution(text='/robot'), robot_id, TextSubstitution(text='/clicked_point')]),
            ],
            output='screen',
            condition=IfCondition(use_rviz),
        ),

    ])
