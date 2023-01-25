#!/usr/bin/env python3

import os

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node

def generate_launch_description():
    my_package_dir = get_package_share_directory('tuto_vision')
    return launch.LaunchDescription([
        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', '/home/bot/ros2_ws/Group-8/tuto_vision/tuto_vision/rviz2.config.challenge2.rviz'])
    ])

