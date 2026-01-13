from setuptools import setup

package_name = 'ogm_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='trung.l.nguyen@tuni.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_crop_memory = ogm_tools.map_crop_memory:main',
            'camera_broadcaster = ogm_tools.camera_broadcaster:main',
        ],
    },
)
