#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns="tf_handler"
    use_sim_time=False

    sbr_base_link_2_t265_parent_link = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sbr_t265_parent_link_2_base_link",
            arguments=["0.4", "0.", "0.",
                       "1.", "0.", "0.", "0.",
                       "base_link", "t265_pose_frame"],
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                ],
            ) 

    offset_odom = Node(
            package="ogm_gridmap",
            executable="offset_odom",
            name="odom_2_map" ,
            output='screen' ,
            namespace=ns,
            remappings=[
                ('odom_in' , '/t265/pose/sample'),
                ('odom_out' , '/odom_t265'),
                ('offset_pose' , '/robot_pose'),
                ],
            parameters=[
                {"use_sim_time" : use_sim_time},
                {'map_frame' : 'map'},
                {'link_frame' : 't265_parent_link'},
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

    odom_2_base_link = Node(
            package="ogm_tools",
            executable="odom_transform",
            name="odom_to_base_link" ,
            #output={'both': 'log'} ,
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                {'parent_frame' : 't265_pose_frame'},
                {'target_frame' : 'base_link'},
                ],
            remappings=[
                ('odom_out' , '/odom_in_map'),
                ('odom_in' , '/odom_t265'),
                ('tf' , '/tf'),
                ('tf_static' , '/tf_static'),
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
                       "0.", "0.", "0.7071", "0.7071",
                       "base_link", "april_link"],
            namespace=ns,
            parameters=[
                {"use_sim_time" : use_sim_time},
                ],
            ) 

    return LaunchDescription([
        sbr_base_link_2_t265_parent_link,
        offset_odom,
        odom_2_tf,
        odom_2_base_link,
        sbr_base_link_2_front_lidar,
        sbr_base_link_2_april_link,
    ])
