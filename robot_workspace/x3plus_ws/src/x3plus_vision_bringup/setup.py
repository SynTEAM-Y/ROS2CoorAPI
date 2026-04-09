from setuptools import setup
from glob import glob

package_name = 'x3plus_vision_bringup'

# Collect files
launch_files = glob('launch/*.launch.py')
param_files  = glob('params/*.yaml')

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', launch_files),
]

if param_files:
    data_files.append(('share/' + package_name + '/params', param_files))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karim',
    maintainer_email='karimnah01@gmail.com',
    description='Bringup launches for X3 Plus cameras',
    license='BSD-3-Clause',
    entry_points={
    },
)
