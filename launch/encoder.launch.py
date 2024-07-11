#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_rpi_as5048a',
            executable='encoder_node',
            name='as5048a_encoder',
            output='screen',
            parameters=[{
                'encoder_resolution': 16383.0
            }]
        ),
    ])
