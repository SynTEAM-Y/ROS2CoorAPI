from setuptools import setup
from glob import glob

package_name = 'scenarios'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karim',
    maintainer_email='karimnah01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ----- Scenario 1: Patrol and rescue
            'patrol_rescue = scenarios.patrol_rescue.patrol_rescue:main',
            'get_coords = scenarios.patrol_rescue.get_coords:main',

            # ----- Scenario 2: Pick and place 
            'pick_and_place = scenarios.pick_and_place.pick_and_place:main',
            'new_pick_and_place = scenarios.pick_and_place.new_pick_and_place:main',
            'pick_front_cube = scenarios.pick_and_place.pick_front_cube:main',

            'pick_place_server = scenarios.scripts.pick_place_server:main',
            'pick_place_service = scenarios.scripts.pick_place_service:main',
            'pick_place_topic_node = scenarios.scripts.pick_place_topic_node:main',
        ],
    },
)