from setuptools import setup

package_name = 'x3plus_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5', 'Pillow', 'PyYAML'],
    zip_safe=True,
    maintainer='Karim',
    maintainer_email='karimnah01@gmail.com',
    description='X3 Plus example nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'min_range = x3plus_examples.min_range:main',
            'closest = x3plus_examples.closest:main',
            'avoid_reflex = x3plus_examples.avoid_reflex:main',
            'lidar_viz_sectors = x3plus_examples.lidar_viz_sectors:main',
            'rosmaster_base_bridge = x3plus_examples.rosmaster_base_bridge:main',
            'navigation = x3plus_examples.navigation:main',
            'rgbd_ir_view = x3plus_examples.rgbd_ir_view:main',
            'rosmaster_rgb_gui = x3plus_examples.rosmaster_rgb_gui:main',
            'display_battery = x3plus_examples.display_battery:main',
            'rosmaster_sequence = x3plus_examples.rosmaster_sequence:main',
            'manual_control = x3plus_examples.manual_control:main',
            'cmd_vel_test = x3plus_examples.cmd_vel_test:main',
            'diff_drive_simulator = x3plus_examples.diff_drive_simulator:main',
            'map_publisher = x3plus_examples.map_publisher:main',
            'arm_controller = x3plus_examples.arm_controller:main',
            'gripper_mimic_relay = x3plus_examples.gripper_mimic_relay:main',
        ],
    },
)
