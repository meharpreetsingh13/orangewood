import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_path   = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'rover.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'earthquake_world.sdf')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    mesh_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'install', 'rover_description', 'share'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([

        # Mesh path for Fortress
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=mesh_path
        ),

        # 1. Gazebo Fortress
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': world_file + ' -r',
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': True}
            ]
        ),

        # 3. Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 4. Spawn rover — 8s delay
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_rover',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-name',  'scouting_rover',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.10',
                    ]
                ),
            ]
        ),

        # 5. Bridge + TF fixes — 10s delay
        TimerAction(
            period=10.0,
            actions=[

                # ROS ↔ Gazebo bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge',
                    output='screen',
                    arguments=[
                        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                        '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    ],
                    parameters=[{
                        'qos_overrides./tf.publisher.durability': 'transient_local',
                        'qos_overrides./tf_static.publisher.durability': 'transient_local',
                        'use_sim_time': True,
                    }]
                ),

                # Camera bridge
                Node(
                    package='ros_gz_image',
                    executable='image_bridge',
                    name='camera_bridge',
                    output='screen',
                    arguments=['/camera/image_raw'],
                ),

                # ── TF FIX 1: Fortress scoped lidar frame → ROS frame ──
                # Fortress names sensor frames as:
                # "scouting_rover/base_link/lidar_sensor"
                # slam_toolbox needs: "lidar_link"
                # This static TF bridges the gap
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='lidar_tf_fix',
                    output='screen',
                    arguments=[
                        '0.070', '0', '0.037',
                        '0', '0', '0', '1',
                        'base_link',
                        'scouting_rover/base_link/lidar_sensor'
                    ],
                    parameters=[{'use_sim_time': True}]
                ),

                # ── TF FIX 2: Publish odom→base_link TF ──
                # Fortress diff_drive publishes TF as:
                # "odom" → "scouting_rover/base_link"
                # ROS expects: "odom" → "base_link"
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_tf_fix',
                    output='screen',
                    arguments=[
                        '0', '0', '0',
                        '0', '0', '0', '1',
                        'base_link',
                        'scouting_rover/base_link'
                    ],
                    parameters=[{'use_sim_time': True}]
                ),

            ]
        ),

    ])