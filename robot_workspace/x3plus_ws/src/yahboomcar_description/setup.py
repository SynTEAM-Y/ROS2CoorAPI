from setuptools import setup
from glob import glob
import os

package_name = 'yahboomcar_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        # URDF, launch, rviz
        (f'share/{package_name}/urdf',  glob('urdf/*.*')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/rviz',   glob('rviz/*.rviz*')),

        # --- meshes: keep directory structure ---
        # mecanum & Ackermann (top-level)
        # (f'share/{package_name}/meshes/mecanum',   glob('meshes/mecanum/*.*')),
        # (f'share/{package_name}/meshes/Ackermann', glob('meshes/Ackermann/*.*')),

        # X3plus + sensor_X3plus (with visual/collision subfolders)
        (f'share/{package_name}/meshes/X3plus/visual',     glob('meshes/X3plus/visual/*.*')),
        (f'share/{package_name}/meshes/X3plus/collision',  glob('meshes/X3plus/collision/*.*')),
        (f'share/{package_name}/meshes/sensor_X3plus/visual',    glob('meshes/sensor_X3plus/visual/*.*')),
        (f'share/{package_name}/meshes/sensor_X3plus/collision', glob('meshes/sensor_X3plus/collision/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='Robot description (URDF + meshes) for Yahboomcar X3plus',
    license='TODO: License declaration',
    entry_points={'console_scripts': []},
)
