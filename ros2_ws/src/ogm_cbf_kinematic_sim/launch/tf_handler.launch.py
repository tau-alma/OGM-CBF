#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns="tf_handler"
    use_sim_time=False


    offset_odom = Node(
            package="ogm_gridmap",
            executable="offset_odom",
            name="odom_2_map" ,
            output='screen' ,
            namespace=ns,
            remappings=[
                ('odom_in' , '/odom'),
                ('odom_out' , '/odom_in_map'),
                ('offset_pose' , '/robot_pose'),
                ],
            parameters=[
                {"use_sim_time" : use_sim_time},
                {'map_frame' : 'map'},
                {'link_frame' : 'base_link'},
                {'offset_to_link' : False},
                {'from_pose' : False},
                {'from_params' : True},
                {'fixed_offset_x' : 10.},
                {'fixed_offset_y' : 10.},
                ],
            ) 

    odom_2_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="odom_2_tf" ,
            output={'both': 'log'} ,
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                {'odom_topic' : '/odom_in_map'},
                ],
            ) 

    sbr_base_link_2_front_lidar = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sbr_base_link_2_front_laser_link",
            arguments=["0.4253", "0.2345", "0.",
                       "0.", "0.", "0.3826834", "0.9238795",
                       "base_link", "front_laser_link"],
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                ],
            ) 

    sbr_base_link_2_april_link = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sbr_base_link_2_april_link",
            arguments=["-0.163", "-0.033", "0.",
                       "0.", "0.", "1.", "0.",
                       "base_link", "april_link"],
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                ],
            ) 

    camera_br = Node(
            package="ogm_tools",
            executable="camera_broadcaster",
            name="l500_broadcaster" ,
            output={'both': 'screen'} ,
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                ],
            ) 

    return LaunchDescription([
        offset_odom,
        odom_2_tf,
        sbr_base_link_2_front_lidar,
        sbr_base_link_2_april_link,
        camera_br,
    ])
