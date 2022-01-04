#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
          Node(package='turtlesim',
               node_executable='turtlesim_node',
               node_name='turtlesim_node',
               output='screen'),
          Node(package='uniform_circle_turtle',
               node_executable='rvd_subscriber',
               node_name='rvd_subscriber',
               output='screen'),
    ])