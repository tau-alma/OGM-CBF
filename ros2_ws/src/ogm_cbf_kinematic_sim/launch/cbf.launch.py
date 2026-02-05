from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = LaunchConfiguration('pkg')
    exe = LaunchConfiguration('exe')
    params_file = LaunchConfiguration('params_file')

    default_pkg = 'ogm_cbf_kinematic_sim'  # change if different
    default_exe = 'ogm_cbf_clf_node_mir_pyr'  # change to your installed console_script name
    default_params = os.path.join(get_package_share_directory(default_pkg), 'config', 'cbf_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('pkg', default_value=default_pkg),
        DeclareLaunchArgument('exe', default_value=default_exe),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package=pkg,
            executable=exe,
            name='CBF_Controller_Node',
            output='screen',
            parameters=[params_file],
            # optional remaps if needed later:
            # remappings=[('odom_2','/odom'), ('map_image','/map_image')],
        )
    ])
