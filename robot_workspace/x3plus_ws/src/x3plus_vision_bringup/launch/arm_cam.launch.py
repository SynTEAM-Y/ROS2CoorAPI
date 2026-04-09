from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Resolve substitutions at runtime
    dev_arg = LaunchConfiguration('device').perform(context).strip()

    params_file = PathJoinSubstitution([
        FindPackageShare('x3plus_vision_bringup'), 'params', 'arm_cam.yaml'
    ])

    # Always load the YAML; only add an override if device was provided
    params = [params_file]
    if dev_arg:
        params.append({'video_device': dev_arg})

    node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='arm_cam',
        namespace='arm_cam',
        output='screen',
        parameters=params,
    )
    return [node]

def generate_launch_description():
    return LaunchDescription([
        # Optional override. If left empty, YAML value is used.
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Optional: override video_device. Leave empty to use YAML default.'
        ),
        OpaqueFunction(function=launch_setup),
    ])
