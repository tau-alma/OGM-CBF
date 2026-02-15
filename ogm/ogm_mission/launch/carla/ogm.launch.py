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

    rectify = ComposableNodeContainer(
            name='imgproc',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    namespace=namespace,
                    parameters=[{'queue_size': 10}],
                    remappings=[
                        ('image_rect', '/carla/ego_vehicle/rgb_depth/image_rect_raw'),
                        ('image', '/carla/ego_vehicle/rgb_depth/image_raw'),
                    ]
                ),
            ],
    )


    depth_2_pcd = ComposableNodeContainer(
            name='pcl_remote',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz',
                    namespace=namespace,
                    parameters=[{'queue_size': 10}],
                    remappings=[
                        ('image_rect', '/carla/ego_vehicle/rgb_depth/image_rect_raw'),
                        ('camera_info', '/carla/ego_vehicle/rgb_depth/camera_info'),
                        ('points', '/carla/ego_vehicle/rgb_depth/points')
                    ]
                ),
            ],
            output='screen',
    )

    velarray_pcd_2_map_pcd = Node(
            package='ogm_gridmap',
            executable='pcd_transformer',
            name='pcd_velarray2odom',
            output={'both': 'screen'},
            namespace=LaunchConfiguration('ns'),
            remappings=[
                ('pcd_in','/carla/ego_vehicle/rgb_depth/lidar'),
                ('pcd_out','points/in_map'),
                ('odom','/odom_in_map'),
                ],
            parameters=[
                {"use_sim_time" : LaunchConfiguration('use_sim_time')},
                {"odom_frame" : "map"},
                {"link_frame" : "ego_vehicle"},
                {"target_frame" : "ego_vehicle/lidar"},
                {"sync_slack" : 0.0125},
                {"sync_window" : 1.0},
                ],
        )

    gridmap = Node(
            package='ogm_gridmap',
            executable='elevgrid',
            name='elevation_gridmap',
            output={'both': 'log'},
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
                {"occgrid_vis_z" : 0.0},
                {"do_pub_elevgrid_real" : False},
                {"do_pub_elevgrid_vis" : False},
                {"cellsize" : .2},
                {"height" : 150.},
                {"width" : 40.},
                {"clearance_thr" : -1.0},
                {"traversable_slope" : 0.78},
                ],
        )

    return LaunchDescription([
        _use_sim_time,
        _ns,
        rectify,
        depth_to_pcd,
        velarray_pcd_2_map_pcd,
        gridmap,
    ])
