"""Launch ten two-port robots with physical latches, peer agents and watchdogs."""
import json
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from swarm_robot_docking.spawn_layout import random_layout
import yaml


def robots(context):
    package = get_package_share_directory('swarm_robot_description')
    with open(LaunchConfiguration('config').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    names = [r['id'] for r in config['robots']]
    if len(names) != 10 or len(set(names)) != 10:
        raise ValueError('This experiment requires ten unique robot IDs')
    random_spawn = LaunchConfiguration('random_spawn').perform(context).lower()
    seed = int(LaunchConfiguration('spawn_seed').perform(context))
    if seed < -1 or random_spawn not in ('true', 'false'):
        raise ValueError('Use random_spawn true/false and spawn_seed -1 or a nonnegative integer')
    if seed == -1:
        seed = random.SystemRandom().randrange(2**32)
    if random_spawn == 'true':
        settings = config['spawn']
        config['robots'] = random_layout(
            config['robots'], settings['arena_bounds'], settings['min_separation'],
            settings['wall_clearance'], seed)
    actions = [LogInfo(msg=f'Spawn seed={seed}; poses={json.dumps(config["robots"])}'),
               Node(package='swarm_robot_shape_formation', executable='shape_goal',
                    output='screen', parameters=[dict(use_sim_time=False, robot_count=len(names),
                                                      robot_ids=names,
                                                      spacing=config['spacing'])]),
               Node(package='swarm_robot_shape_formation', executable='formation_monitor',
                    output='screen', parameters=[dict(use_sim_time=True, robot_ids=names)]),
               Node(package='swarm_robot_docking', executable='environment_reset',
                    output='screen', parameters=[dict(use_sim_time=True,
                                                      robots_json=json.dumps(config['robots']),
                                                      detach_before_reset=True)])]
    for robot in config['robots']:
        name = robot['id']
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'spawn.launch.py')),
                launch_arguments=dict(namespace=name, frame_prefix=name+'/', entity_name=name,
                                      rear_docking='true', formation_docking='true',
                                      x=str(robot['x']), y=str(robot['y']),
                                      z='0.01', yaw=str(robot['yaw'])).items()),
            Node(package='swarm_robot_shape_formation', executable='shape_agent', namespace=name,
                 parameters=[dict(config['agent'], use_sim_time=True,
                                  robot_id=name, robot_ids=names)],
                 output='screen'),
            Node(package='swarm_robot_shape_formation', executable='velocity_guard',
                 namespace=name,
                 parameters=[dict(use_sim_time=True, command_timeout=.5)], output='screen'),
        ])
    return actions


def generate_launch_description():
    package = get_package_share_directory('swarm_robot_shape_formation')
    gazebo = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('random_spawn', default_value='true'),
        DeclareLaunchArgument('dds_udp', default_value='true'),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(package, 'config', 'fastdds_udp.xml'),
            condition=IfCondition(LaunchConfiguration('dds_udp'))),
        DeclareLaunchArgument('spawn_seed', default_value='-1'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            package, 'config', 'formation.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': os.path.join(package, 'worlds', 'formation_arena.world'),
                              'gui': LaunchConfiguration('gui'),
                              'server_required': 'true'}.items()),
        OpaqueFunction(function=robots),
        Node(package='rviz2', executable='rviz2', name='shape_rviz',
             arguments=['-d', os.path.join(package, 'rviz', 'formation.rviz')],
             parameters=[dict(use_sim_time=True)], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
