from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns         = LaunchConfiguration('ns')
    base_frame = LaunchConfiguration('base_frame')

    # Color via v4l2_camera
    rgb_device = LaunchConfiguration('rgb_device')
    rgb_w      = LaunchConfiguration('rgb_w')
    rgb_h      = LaunchConfiguration('rgb_h')
    rgb_fps    = LaunchConfiguration('rgb_fps')

    # Depth/IR via astra_camera
    depth_w   = LaunchConfiguration('depth_w')
    depth_h   = LaunchConfiguration('depth_h')
    depth_fps = LaunchConfiguration('depth_fps')
    reg       = LaunchConfiguration('depth_registration')
    serial    = LaunchConfiguration('serial')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='orbbec'),
        DeclareLaunchArgument('base_frame', default_value='orbbec_link'),
        DeclareLaunchArgument('serial', default_value=''),  # put ACRK94300G5 here if you like

        # RGB (UVC) args
        DeclareLaunchArgument('rgb_device', default_value='/dev/video2'),
        DeclareLaunchArgument('rgb_w', default_value='1280'),
        DeclareLaunchArgument('rgb_h', default_value='720'),
        DeclareLaunchArgument('rgb_fps', default_value='30.0'),

        # Depth args
        DeclareLaunchArgument('depth_w', default_value='640'),
        DeclareLaunchArgument('depth_h', default_value='480'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('depth_registration', default_value='true'),

        # 1) Depth + IR from Astra (disable internal UVC so it won't touch /dev/bus/usb)
        Node(
            package='astra_camera',
            executable='astra_camera_node',  # use the name you saw in the log
            namespace=ns,
            output='screen',
            parameters=[{
                'enable_color': False,            # <— disable internal RGB
                'enable_depth': True,
                'enable_ir': True,
                'use_uvc_camera': False,          # <— important
                'depth_width':  LaunchConfiguration('depth_w'),
                'depth_height': LaunchConfiguration('depth_h'),
                'depth_fps':    LaunchConfiguration('depth_fps'),
                'depth_registration': LaunchConfiguration('depth_registration'),
                'publish_tf': True,
                'base_frame': LaunchConfiguration('base_frame'),
                'serial_number': serial,
            }],
        ),

        # 2) RGB via usb_cam
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='orbbec/color',
            output='screen',
            parameters=[{
                'video_device': rgb_device,
                'image_width': rgb_w,
                'image_height': rgb_h,
                'framerate': rgb_fps,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'orbbec_color',
                'camera_frame_id': 'camera_color_optical_frame',
            }],
        ),

#        # Wait 2s for node to be up, then enable laser
#        TimerAction(
#            period=2.0,
#            actions=[ExecuteProcess(
#                cmd=['ros2', 'service', 'call',
#                     '/orbbec/set_laser_enable',
#                     'astra_camera_msgs/srv/SetInt32',
#                     '{data: 1}'],
#                output='screen'
#            )]
#        ),
#
#        # Wait 2.5s, then disable LDP (prevents auto-dimming)
#        TimerAction(
#            period=2.5,
#            actions=[ExecuteProcess(
#                cmd=['ros2', 'service', 'call',
#                     '/orbbec/set_ldp_enable',
#                     'astra_camera_msgs/srv/SetInt32',
#                     '{data: 0}'],
#                output='screen'
#            )]
#        ),
    ])
