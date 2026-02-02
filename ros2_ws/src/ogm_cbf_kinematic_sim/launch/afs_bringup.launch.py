#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ogm_cbf_kinematic_sim')
    hyperparams_file = os.path.join(pkg_share, 'hyperparams.yaml')
    # 1) load the file & extract the ros__parameters dict
    with open(hyperparams_file, 'r') as f:
        y = yaml.safe_load(f)
    params = y.get('ros__parameters', {})

    return LaunchDescription([
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='map_publisher_node',
            name='map_publisher',
            output='screen',
            parameters=[ params ],   # <<< inline dict
        ),
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='kinematic_afs_sim_node',
            name='sim',
            output='screen',
            parameters=[ params ],
        ),
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='map_viz_node',
            name='map_viz',
            output='screen',
            parameters=[ params ],
        ),
        Node(
            package='ogm_cbf_kinematic_sim',
            executable='start_pose_selector_node',
            name='start_pose_selector',
            output='screen',
            parameters=[ params ],
        ),
    ])
