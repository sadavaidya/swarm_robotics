"""Install the ten-robot formation nodes, launch files and simulation assets."""

from glob import glob
from setuptools import setup

PACKAGE = 'swarm_robot_shape_formation'
setup(
    name=PACKAGE, version='0.1.0', packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+PACKAGE]),
        ('share/'+PACKAGE, ['package.xml', 'README.md']),
        ('share/'+PACKAGE+'/launch', glob('launch/*.launch.py')),
        ('share/'+PACKAGE+'/config', glob('config/*.yaml')+glob('config/*.xml')),
        ('share/'+PACKAGE+'/worlds', glob('worlds/*.world')),
        ('share/'+PACKAGE+'/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='sadashiv', maintainer_email='svaidya2320@gmail.com',
    description='Peer auction and safe planar formation for ten primitive robots.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        name+' = '+PACKAGE+'.'+name+':main'
        for name in ('shape_goal', 'shape_agent', 'formation_monitor', 'velocity_guard')
    ]},
)
