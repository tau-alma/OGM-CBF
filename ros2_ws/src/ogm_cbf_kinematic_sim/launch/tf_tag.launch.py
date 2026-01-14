#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns="tf_handler"
    use_sim_time=False

    camera_br = Node(
            package="ogm_tools",
            executable="camera_broadcaster",
            name="l500_broadcaster" ,
            output={'both': 'screen'} ,
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                {"map_frame" : "map"},
                {"tag_link_frame" : "april_link"},
                {"tag_frame" : "tagStandard41h12:0"},
                {"camera_frame" : "l500_color_optical_frame"},
                {"tag_init_frame" : "april_init_link"},
                ],
            ) 

    return LaunchDescription([
        camera_br,
    ])
