#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('ogm_cbf_kinematic_sim')
    map_params = os.path.join(pkg_share, 'config', 'map_params.yaml')
    

    return LaunchDescription([
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='map_publisher_node',
            name='map_publisher',
            output='screen',
            parameters=[map_params],

        ),
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='kinematic_diff_drive_sim_node',
            name='sim',
            output='screen',
            
        ),
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='map_viz_node',
            name='map_viz',
            output='screen',
            
        ),
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='start_pose_selector_node',
            name='start_pose_selector',
            output='screen',
            
        ),
    ])
