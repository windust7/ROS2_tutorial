#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(package='my_first_ros2_pkg',
             node_executable='cmd_vel_publisher',
             node_name='cmd_vel_publisher',
             output='screen'),
        Node(package='my_first_ros2_pkg',
             node_executable='cmd_vel_subscriber',
             node_name='cmd_vel_subscriber',
             output='screen'),
    ])