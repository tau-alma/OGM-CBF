#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ogm_gridmap',
            executable='ogm_laserscan',
            name='map_builder',
            output='screen',
            remappings=[
                ('map','/scan/in_mir'),
                ('scan','/scan'),
                ],
        ),
        Node(
            package='ogm_tools',
            executable='map_crop_memory',
            name='map_transformer',
            output='screen',
            remappings=[
                ('map','/scan/in_mir'),
                ('map_im','/map/in_mir'),
                ],
        ),
    ])
