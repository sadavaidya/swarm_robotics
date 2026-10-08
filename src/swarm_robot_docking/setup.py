"""Install identical reusable docking agents and experiment assets."""
from glob import glob
from setuptools import setup

PACKAGE = 'swarm_robot_docking'
setup(
    name=PACKAGE, version='0.1.0', packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
    ] + [('share/' + PACKAGE + '/' + folder, glob(folder + '/*'))
         for folder in ('launch', 'config', 'rviz', 'worlds')],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='sadashiv', maintainer_email='svaidya2320@gmail.com',
    description='Shape-only auctions and logical pair docking.', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        name + ' = ' + PACKAGE + '.' + name + ':main'
        for name in ('swarm_agent', 'swarm_allocator', 'docking_controller', 'environment_reset')
    ]},
)
