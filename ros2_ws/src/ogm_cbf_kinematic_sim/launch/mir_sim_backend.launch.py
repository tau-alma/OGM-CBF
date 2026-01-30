#!/usr/bin/env python3
import os
import yaml

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
        default_value='mir_sim_backend',
        description='mir sim namespace'
    )

    cmd_delay = Node(
            package="ogm_gridmap",
            executable="cmd_delay",
            name="cmd_delay" ,
            output='screen' ,
            namespace=LaunchConfiguration('ns'),
            remappings=[
                ('in', '/cmd_vel/raw'),
                ('out', '/cmd_vel'),
                ],
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                {'delay' : 2.},
                ],
            ) 


    return LaunchDescription([
        _use_sim_time,
        _ns,
        cmd_delay,
    ])
