from setuptools import find_packages, setup

package_name = 'ogm_cbf_kinematic_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # this installs launch files under share/<pkg>/launch
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
        ]),
        # this installs hyperparams.yaml under share/<pkg>
        ('share/' + package_name, [
            'hyperparams.yaml',
        ]),
    ],
    install_requires=['setuptools','rclpy','opencv-python','wandb'],
    zip_safe=True,
    maintainer='golnaz',
    maintainer_email='golnaz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ogm_cbf_clf_node_mir = ogm_cbf_kinematic_sim.ogm_cbf_clf_node_mir:main',
            'kinematic_diff_drive_sim_node = ogm_cbf_kinematic_sim.kinematic_diff_drive_sim_node:main',
            'map_viz_node = ogm_cbf_kinematic_sim.map_viz_node:main',
            'wandb_logger_node = ogm_cbf_kinematic_sim.wandb_logger_node:main',
            'map_publisher_node = ogm_cbf_kinematic_sim.map.map_publisher_node:main',
            'start_pose_selector_node = ogm_cbf_kinematic_sim.start_pose_selector_node:main',
        ],
    },
)
