from setuptools import setup, find_packages

package_name = 'projectn'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Let ROS (rosdep/apt) provide Python deps; keep this minimal.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Othman Belal',
    maintainer_email='Othmanobi96@gmail.com',
    description='ProjectN: ROS 2 tree (services upstream, topics downstream) + agent + viz hub.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'server = projectn.main_server:main',
            'agent  = projectn.main_agent:main',
            'projectn-launch-random-tree = projectn.tools.launch_random_tree:main',
            'projectn-agent-hub   = projectn.tools.agent_hub:main',
            "agent_supervisor = projectn.tools.agent_supervisor:main",
            'agent_hub_robot = projectn.tools.agent_hub_robot:main',
            'robot_node = projectn.tools.robot_node:main',
            
        ],
    },
)

