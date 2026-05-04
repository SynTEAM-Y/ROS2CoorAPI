from setuptools import setup

package_name = 'x3plus_lidar_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup_tg30.launch.py',
        ]),
        ('share/' + package_name + '/params', [
            'params/tg30.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karim',
    maintainer_email='karimnah01@gmail.com',
    description='LiDAR bringup for X3 Plus using ydlidar_ros2_driver',
    license='MIT',
    entry_points={'console_scripts': []},
)
