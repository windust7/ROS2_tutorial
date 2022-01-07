#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        variable_name='param_dir',
        default=os.path.join(
            get_package_share_directory('dwa_turtle'),
            'param',
            'config.yaml'))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir,
                description='Full path of parameter file'),
            Node(
                package='turtlesim',
                node_executable='turtlesim_node',
                node_name='turtlesim_node',
                output='screen'
                ),
            Node(
                package='dwa_turtle',
                node_executable='agent_spawner',
                node_name='agent_spawner',
                output='screen'
                ),
            Node(
                package='dwa_turtle',
                node_executable='dwa_planner',
                node_name='dwa_planner',
                parameters=[param_dir],
                output='screen'
                )
        ]
    )