"""Launch one arena and five copies of the existing robot plus chain behaviors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def experiment(context):
    """Keep robot poses, target assignments and gains in a separate config."""
    description = get_package_share_directory('swarm_robot_description')
    with open(LaunchConfiguration('config').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    enabled = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    names = [robot['namespace'] for robot in config['robots']]
    if len(names) != 5 or len(set(names)) != 5:
        raise ValueError('This experiment needs five unique robot namespaces')
    actions = []
    for index, robot in enumerate(config['robots']):
        namespace = robot['namespace']
        if index and robot['target'] != names[index - 1]:
            raise ValueError('Each follower must target the preceding robot in the list')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'spawn.launch.py')),
            launch_arguments={
                'namespace': namespace, 'frame_prefix': namespace + '/',
                'entity_name': namespace, 'x': str(robot['x']), 'y': str(robot['y']),
                'z': '0.01', 'yaw': str(robot['yaw']),
            }.items(),
        ))
        # odometry_source=1 reports Gazebo world pose, even for nonzero spawn poses.
        # Thus these are identity transforms, NOT transforms by spawn position.
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='world_to_odom', namespace=namespace,
            arguments=['--frame-id', 'world', '--child-frame-id', namespace + '/odom'],
            parameters=[{'use_sim_time': True}], output='screen'))
        role = 'leader' if index == 0 else 'follower'
        parameters = dict(config['common'], **config[role])
        parameters.update(use_sim_time=True, robot_namespace=namespace, enabled=enabled)
        if index:
            parameters['target_namespace'] = robot['target']
        actions.append(Node(
            package='swarm_robot_behaviors', executable=role + '_controller',
            namespace=namespace, parameters=[parameters], output='screen'))
    return actions


def generate_launch_description():
    package = get_package_share_directory('swarm_robot_behaviors')
    gazebo = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('config', default_value=os.path.join(
            package, 'config', 'experiment.yaml')),
        DeclareLaunchArgument('world', default_value=os.path.join(
            package, 'worlds', 'swarm_arena.world')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui'),
                              'server_required': 'true'}.items()),
        OpaqueFunction(function=experiment),
        Node(package='rviz2', executable='rviz2', name='swarm_rviz',
             arguments=['-d', os.path.join(package, 'rviz', 'five_robots.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
