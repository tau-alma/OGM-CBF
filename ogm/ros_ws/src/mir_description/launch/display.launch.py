from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    mir_gazebo_path = FindPackageShare('mir_gazebo')
    default_model_path = 'urdf/mir.urdf.xacro'

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                        description='Path to robot urdf file relative to mir_gazebo package'))

    #ld.add_action(IncludeLaunchDescription(
    #    PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
    #    launch_arguments={
    #        'urdf_package': 'mir_gazebo',
    #        'urdf_package_path': LaunchConfiguration('model'),
    #        'jsp_gui': LaunchConfiguration('gui')}.items()
    #))
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'mir_gazebo',
            'urdf_package_path': LaunchConfiguration('model'),
            }.items()
    ))

    return ld
