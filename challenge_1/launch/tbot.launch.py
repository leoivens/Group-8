#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    tbot_sim_path = get_package_share_directory('tbot_start')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/minimal.launch.py'])),
        Node(
            package='tbot_pytools',
            executable='multiplexer',
            name="multi"
            ),
        Node(
            package='tuto_move',
            executable='reactive_move',
            name="reactive_move"
            )
        ])
           
         

