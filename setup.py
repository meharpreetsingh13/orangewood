from setuptools import setup
import os
from glob import glob

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS2 package discovery
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # URDF + Xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf') + glob('urdf/*.xacro')),

        # Mesh files
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl') + glob('meshes/*.dae')),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # World files (both .world and .sdf)
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world') + glob('worlds/*.sdf')),

        # Model files
        (os.path.join('share', package_name, 'models'),
            glob('models/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@email.com',
    description='4WD Scouting Rover — Gazebo Fortress',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_hold_node = rover_description.teleop_hold_node:main',
        ],
    },
)
