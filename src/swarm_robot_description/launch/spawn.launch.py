"""Spawn one namespace-ready robot into an already running Gazebo Classic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def spawn_robot(context):
    """Resolve namespace and frame prefix together before expanding Xacro."""
    package = get_package_share_directory('swarm_robot_description')
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = LaunchConfiguration('frame_prefix').perform(context)
    if prefix == 'auto':
        prefix = namespace + '/' if namespace else ''
    entity = LaunchConfiguration('entity_name').perform(context)
    if entity == 'auto':
        entity = namespace.replace('/', '_') if namespace else 'swarm_robot'
    description = xacro.process_file(
        os.path.join(package, 'urdf', 'swarm_robot.urdf.xacro'),
        mappings={'robot_namespace': '/' + namespace, 'frame_prefix': prefix,
                  'formation_docking': LaunchConfiguration('formation_docking').perform(context),
                  'rear_docking': LaunchConfiguration('rear_docking').perform(context)},
    ).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, output='screen', parameters=[{
                 'robot_description': description, 'use_sim_time': True,
             }]),
        # Gazebo services remain global; robot_description is namespace-relative.
        # The plugins already receive their namespace from Xacro.
        Node(package='gazebo_ros', executable='spawn_entity.py',
             namespace=namespace, output='screen',
             arguments=['-entity', entity, '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw'),
                        '-timeout', '60'],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('formation_docking', default_value='false'),
        DeclareLaunchArgument('rear_docking', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot ROS namespace; empty preserves /cmd_vel.'),
        DeclareLaunchArgument('frame_prefix', default_value='auto',
                              description='auto uses namespace/; applied to URDF names and TF.'),
        DeclareLaunchArgument('entity_name', default_value='auto',
                              description='Gazebo model name; auto derives it from namespace.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.01',
                              description='Spawn 1 cm above the ground to settle contacts.'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=spawn_robot),
    ])
