#!/usr/bin/env python3

import os

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node

def generate_launch_description():
    my_package_dir = get_package_share_directory('tuto_vision')
    return launch.LaunchDescription([
        Node(
            package='tuto_vision',
            executable='vision_1',
            name="vision"
            ),
        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', os.path.join('/home/bot/ros2_ws/tuto_vision/tuto_vision', 'rviz2.config.challenge1.rviz')])
    ])

