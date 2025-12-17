#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns="ogm"

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
                {"target_frame" : "back_laser_link"},
                ],
        )

    pose_2_odom = Node(
            package='ogm_gridmap',
            executable='pose_2_odom',
            name='pose_2_odom',
            output='log',
            namespace=ns,
            remappings=[
                ('pose','/robot_pose'),
                ('odom','robot_odom'),
                ],
            parameters=[
                {"map_frame" : "odom"},
                {"target_frame" : "base_link"},
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
                ('odom','robot_odom'),
                ],
            parameters=[
                {"odom_frame" : "odom"},
                {"link_frame" : "base_link"},
                {"target_frame" : "front_laser_link"},
                ],
        )

    odom_2_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="fixposition_to_map" ,
            output={'both': 'log'} ,
            namespace=ns,
            parameters=[
                {'odom_topic' : 'robot_odom'},
                ],
            ) 

    base_link_2_front_lidar = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_base_link_2_front_liser_link",
            arguments=["0.4253", "0.2345", "0.",
                       "0.", "0.", "0.3826834", "0.9238795",
                       "base_link", "front_laser_link"],
            ) 


    return LaunchDescription([
        pose_2_odom,
        odom_2_tf,
        base_link_2_front_lidar,
        front_scan_2_front_pcd,
        rear_scan_2_rear_pcd,
        rear_pcd_2_odom_pcd,
    ])
