#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns="tf_handler"
    use_sim_time=True

    odom_2_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="odom_2_tf" ,
            output={'both': 'log'} ,
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                {'odom_topic' : '/odom'},
                ],
            ) 

    sbr_base_footprint_2_front_lidar = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sbr_base_footprint_2_front_laser_link",
            arguments=["0.4253", "0.2345", "0.",
                       "0.", "0.", "0.3826834", "0.9238795",
                       "base_footprint", "front_laser_link"],
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                ],
            ) 


    return LaunchDescription([
        odom_2_tf,
        sbr_base_footprint_2_front_lidar,
    ])
