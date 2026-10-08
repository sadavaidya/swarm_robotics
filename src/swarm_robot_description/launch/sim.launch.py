"""Start Gazebo Classic, spawn one robot, and optionally open RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = get_package_share_directory('swarm_robot_description')
    gazebo = get_package_share_directory('gazebo_ros')
    robot_arguments = {
        name: LaunchConfiguration(name)
        for name in ('namespace', 'frame_prefix', 'entity_name', 'x', 'y', 'z', 'yaw')
    }
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo Classic GUI.'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also open the simulation RViz view.'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(package, 'worlds', 'empty.world')),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value='auto'),
        DeclareLaunchArgument('entity_name', default_value='auto'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'spawn.launch.py')),
            launch_arguments=robot_arguments.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
            }.items(),
        ),
    ])
