#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns="ogm"
    use_sim_time=False

    front_scan_2_front_pcd =  Node(
            package='ogm_gridmap',
            executable='scan2pcd',
            name='scan2pcd_front',
            output='log',
            namespace=ns,
            remappings=[
                ('scan','/scan'),
                ('pcd','points/in_front_laser_link'),
                ],
            parameters=[
                {"use_sim_time" : use_sim_time},
                {"target_frame" : "front_laser_link"},
                ],
        )

    rear_scan_2_rear_pcd = Node(
            package='ogm_gridmap',
            executable='scan2pcd',
            name='scan2pcd_back',
            output='log',
            namespace=ns,
            remappings=[
                ('scan','/scan'),
                ('pcd','points/in_back_laser_link'),
                ],
            parameters=[
                {"use_sim_time" : use_sim_time},
                {"target_frame" : "back_laser_link"},
                ],
        )

    rear_pcd_2_odom_pcd = Node(
            package='ogm_gridmap',
            executable='pcd_transformer',
            name='pcd_front2odom',
            output='screen',
            namespace=ns,
            remappings=[
                ('pcd_in','points/in_front_laser_link'),
                ('pcd_out','points/in_odom'),
                ('odom','/odom_in_map'),
                ],
            parameters=[
                {"use_sim_time" : use_sim_time},
                {"odom_frame" : "map"},
                {"link_frame" : "base_link"},
                {"target_frame" : "front_laser_link"},
                ],
        )

    gridmap = Node(
            package='ogm_gridmap',
            executable='occgrid',
            name='gridmap',
            output='screen',
            namespace=ns,
            remappings=[
                ('pcd','points/in_odom'),
                ('odom','/odom'),
                ],
            parameters=[
                {"use_sim_time" : use_sim_time},
                {"do_pub_grid" : True},
                {"do_pub_img" : True},
                {"map_frame" : "map"},
                {"cell_size" : 0.05},
                {"height" : 30.},
                {"width" : 50.},
                {"s_target" : 0.95},
                ],
        )


    return LaunchDescription([
        front_scan_2_front_pcd,
        rear_scan_2_rear_pcd,
        rear_pcd_2_odom_pcd,
        gridmap
    ])
