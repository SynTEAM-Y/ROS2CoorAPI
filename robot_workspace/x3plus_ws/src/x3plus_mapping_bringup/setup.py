from setuptools import setup, find_packages
import os

package_name = 'x3plus_mapping_bringup'

def collect_files(dirpath):
    files = []
    if os.path.isdir(dirpath):
        for root, _, names in os.walk(dirpath):
            for n in names:
                files.append(os.path.join(root, n))
    return files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', collect_files('launch')),
        (f'share/{package_name}/rviz',   collect_files('rviz')),
        (f'share/{package_name}/urdf',   collect_files('urdf')),  # ok if empty
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karim',
    maintainer_email='karimnah01@gmail.com',
    description='GMapping + reactive navigation bringup for X3 Plus',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_integrator = x3plus_mapping_bringup.odom_integrator:main',
            'reactive_nav   = x3plus_mapping_bringup.reactive_nav:main',
        ],
    },
)
