#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    _use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use_sim_time'
    )

    _ns = DeclareLaunchArgument(
        'ns',
        default_value='ogm',
        description='tf handler namespace'
    )


    velarray_pcd_2_map_pcd = Node(
            package='ogm_gridmap',
            executable='pcd_transformer',
            name='pcd_velarray2odom',
            output={'both': 'log'},
            namespace=LaunchConfiguration('ns'),
            remappings=[
                ('pcd_in','/M1600_driver/points'),
                ('pcd_out','points/in_map'),
                ('odom','/odom_in_map'),
                ],
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                {"odom_frame" : "map"},
                {"link_frame" : "base_link"},
                {"target_frame" : "velarray_sensor"},
                {"sync_slack" : 0.0125},
                {"sync_window" : 1.0},
                ],
        )

    gridmap = Node(
            package='ogm_gridmap',
            executable='elevgrid',
            name='elevation_gridmap',
            output={'both': 'screen'},
            namespace=LaunchConfiguration('ns'),
            remappings=[
                ('pcd','points/in_map'),
                ('clearance_odom','/odom_in_map'),
                ],
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                {"map_frame" : 'map'},
                {"do_pub_occgrid" : True},
                {"do_pub_occimg" : True},
                {"occgrid_vis_z" : -20.0},
                {"do_pub_elevgrid_real" : False},
                {"do_pub_elevgrid_vis" : True},
                {"cellsize" : .25},
                {"height" : 80.},
                {"width" : 100.},
                {"clearance_thr" : -1.0},
                {"traversable_slope" : 0.78},
                ],
        )

    return LaunchDescription([
        _use_sim_time,
        _ns,
        velarray_pcd_2_map_pcd,
        gridmap,
    ])
