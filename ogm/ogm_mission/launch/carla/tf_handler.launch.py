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
        default_value='carla_tf_handler',
        description='tf handler namespace'
    )


    offset_odom = Node(
            package="ogm_gridmap",
            executable="offset_odom",
            name="odom_2_map" ,
            output='screen' ,
            namespace=LaunchConfiguration('ns'),
            remappings=[
                ('odom_in' , '/carla/ego_vehicle/odometry
 '),
                ('odom_out' , '/odom_in_map'),
                ],
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                {'map_frame' : 'map'},
                {'link_frame' : 'base_link'},
                {'offset_to_link' : False},
                {'from_pose' : False},
                {'from_params' : True},
                {'fixed_offset_x' : -25.},
                {'fixed_offset_y' : 25.},
                ],
            ) 

    odom_2_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="odom_2_tf" ,
            output={'both': 'log'} ,
            namespace=LaunchConfiguration('ns'),
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                {'odom_topic' : '/odom_in_map'},
                ],
            ) 

    sbr_base_link_2_velarray = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sbr_base_link_2_front_laser_link",
            arguments=["2.0", "0.0", "0.3",
                       "0.", "0.", "0.0",
                       "base_link", "velarray"],
            namespace=LaunchConfiguration('ns'),
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                ],
            ) 


    return LaunchDescription([
        _use_sim_time,
        _ns,
        offset_odom,
        odom_2_tf,
        sbr_base_link_2_velarray,
    ])
