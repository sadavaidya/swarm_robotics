"""Open a robot-specific RViz view for an already running simulation."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def remove_config(context, path):
    """Remove only the temporary configuration created by this launch."""
    if os.path.exists(path):
        os.unlink(path)
    return []


def open_rviz(context):
    """Adapt display frames and topics without modifying the installed template."""
    package = get_package_share_directory('swarm_robot_description')
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = LaunchConfiguration('frame_prefix').perform(context)
    if prefix == 'auto':
        prefix = namespace + '/' if namespace else ''
    with open(os.path.join(package, 'rviz', 'swarm_robot.rviz'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    def adapt(value):
        if isinstance(value, dict):
            return {key: adapt(item) for key, item in value.items()}
        if isinstance(value, list):
            return [adapt(item) for item in value]
        if isinstance(value, str):
            if value in ('odom', 'base_footprint', 'imu_link', 'front_range_link'):
                return prefix + value
            if value in ('/robot_description', '/odom', '/range/front'):
                return '/' + namespace + value if namespace else value
        return value

    descriptor, path = tempfile.mkstemp(prefix='swarm_robot_', suffix='.rviz')
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(adapt(config), stream)
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=remove_config, args=[path]),
        ])),
        Node(package='rviz2', executable='rviz2', namespace=namespace, output='screen',
             arguments=['-d', path], parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value='auto'),
        OpaqueFunction(function=open_rviz),
    ])
