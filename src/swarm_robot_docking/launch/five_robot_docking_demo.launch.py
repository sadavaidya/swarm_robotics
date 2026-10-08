"""Reuse the arena and robot, run agents/controllers, and wait for a shape command."""

import json
import os
import random
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

from swarm_robot_docking.spawn_layout import random_layout


def robots(context):
    description = get_package_share_directory('swarm_robot_description')
    with open(LaunchConfiguration('config').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    names = [robot['id'] for robot in config['robots']]
    if len(names) != 5 or len(set(names)) != 5:
        raise ValueError('The demo requires five unique robot IDs')
    random_spawn = LaunchConfiguration('random_spawn').perform(context).lower()
    if random_spawn not in ('true', 'false'):
        raise ValueError('random_spawn must be true or false')
    seed = int(LaunchConfiguration('spawn_seed').perform(context))
    if seed < -1:
        raise ValueError('spawn_seed must be -1 (fresh layout) or a nonnegative integer')
    if seed == -1:
        seed = random.SystemRandom().randrange(2**32)
    if random_spawn == 'true':
        spawn = config.get('spawn', {})
        config['robots'] = random_layout(
            config['robots'], spawn.get('arena_bounds', [-5., 5., -5., 5.]),
            spawn.get('min_separation', .8), spawn.get('wall_clearance', .5), seed=seed)
    layout_log = (f'Random spawn seed={seed}' if random_spawn == 'true' else 'Fixed spawn layout')
    layout_log += '; starting poses: ' + json.dumps(config['robots'])
    # These exact sampled poses are also supplied to reset; reset replays the
    # starting experiment, while relaunching with seed=-1 generates a new one.
    # Derive geometry from the same URDF used to spawn robots: no second set of
    # manually synchronized front-face or drive-axle offsets.
    urdf = ET.fromstring(xacro.process_file(os.path.join(
        description, 'urdf', 'swarm_robot.urdf.xacro')).toxml())
    dock_offset = sum(float(urdf.find("joint[@name='" + name + "']/origin").get('xyz').split()[0])
                      for name in ('front_dock_joint', 'front_dock_frame_joint'))
    axle_offset = float(urdf.find("joint[@name='left_wheel_joint']/origin").get('xyz').split()[0])
    actions = [LogInfo(msg=layout_log), Node(
        package='swarm_robot_docking', executable='swarm_allocator', output='screen',
        parameters=[dict(config['allocator'], use_sim_time=True, dock_offset=dock_offset)])]
    actions.append(Node(
        package='swarm_robot_docking', executable='environment_reset', output='screen',
        parameters=[{'use_sim_time': True, 'robots_json': json.dumps(config['robots'])}]))
    for robot in config['robots']:
        name = robot['id']
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'spawn.launch.py')),
            launch_arguments={'namespace': name, 'frame_prefix': name + '/', 'entity_name': name,
                              'x': str(robot['x']), 'y': str(robot['y']), 'z': '0.01',
                              'yaw': str(robot['yaw'])}.items()))
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', namespace=name,
            name='world_to_odom',
            arguments=['--frame-id', 'world', '--child-frame-id', name+'/odom'],
            parameters=[{'use_sim_time': True}], output='screen'))
        actions.append(Node(
            package='swarm_robot_docking', executable='swarm_agent', namespace=name,
            parameters=[dict(config['agent'], use_sim_time=True, robot_id=name)], output='screen'))
        actions.append(Node(
            package='swarm_robot_docking', executable='docking_controller', namespace=name,
            parameters=[dict(config['controller'], use_sim_time=True,
                             robot_id=name, axle_offset=axle_offset)], output='screen'))
    return actions


def generate_launch_description():
    package = get_package_share_directory('swarm_robot_docking')
    gazebo = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('random_spawn', default_value='true',
                             description='Random starting poses; false uses configured poses'),
        DeclareLaunchArgument('spawn_seed', default_value='-1',
                             description='-1 generates a fresh layout; >=0 reproduces a layout'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            package, 'config', 'docking.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': os.path.join(package, 'worlds', 'docking_arena.world'),
                              'gui': LaunchConfiguration('gui'),
                              'server_required': 'true'}.items()),
        OpaqueFunction(function=robots),
        Node(package='rviz2', executable='rviz2', name='docking_rviz',
             arguments=['-d', os.path.join(package, 'rviz', 'docking.rviz')],
             parameters=[{'use_sim_time': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
