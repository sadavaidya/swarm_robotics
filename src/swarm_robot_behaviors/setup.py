"""Install the behavior nodes and experiment assets."""
from glob import glob
from setuptools import setup

PACKAGE = 'swarm_robot_behaviors'
setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
    ] + [('share/' + PACKAGE + '/' + folder, glob(folder + '/*'))
         for folder in ('launch', 'config', 'worlds', 'rviz')],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sadashiv',
    maintainer_email='svaidya2320@gmail.com',
    description='Simple namespaced leader-following in Gazebo Classic.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'leader_controller = swarm_robot_behaviors.leader_controller:main',
        'follower_controller = swarm_robot_behaviors.follower_controller:main',
    ]},
)
