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
            executable='pose_2_odom',
            name='pose_2_odom',
            output='log',
            remappings=[
                ('pose','/robot_pose'),
                ('odom','/robot_odom'),
                ],
        ),
        Node(
            package='ogm_gridmap',
            executable='ogm_laserscan',
            name='map_builder',
            output='log',
            remappings=[
                ('map','/scan/in_mir'),
                ('scan','/scan'),
                ],
            parameters=[
                {"map_frame" : "odom"},
                {"target_frame" : "base_footprint"},
                ],
        ),
        Node(
            package='ogm_tools',
            executable='map_crop_memory',
            name='map_transformer',
            output='log',
            remappings=[
                ('map','/scan/in_mir'),
                ('map_im','/map/in_mir'),
                ],
        ),
    ])
