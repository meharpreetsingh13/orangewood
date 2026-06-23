#!/usr/bin/env python3
"""
slam.launch.py
══════════════════════════════════════════════════════════════
Extends gazebo.launch.py to add slam_toolbox mapping.

Launch order (delays matched to gazebo.launch.py timers):
  0 s  — Gazebo Fortress + RSP + JSP        (via gazebo.launch.py)
  8 s  — Spawn rover                         (via gazebo.launch.py)
 10 s  — ROS↔Gz bridge (cmd_vel/odom/scan…) (via gazebo.launch.py)
 12 s  — slam_toolbox async mapper           (NEW — waits for bridge)
 12 s  — teleop_hold_node                    (NEW)
 13 s  — RViz2  (optional, pass rviz:=true)  (NEW)

Usage:
  ros2 launch rover_description slam.launch.py
  ros2 launch rover_description slam.launch.py rviz:=true
══════════════════════════════════════════════════════════════
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share   = get_package_share_directory('rover_description')
    params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_cfg    = os.path.join(pkg_share, 'config', 'slam_rviz.rviz')

    # ── Launch arguments ──────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz with a SLAM-ready config'
    )
    rviz = LaunchConfiguration('rviz')

    # ── 1. Include the existing Gazebo launch ─────────────
    #    Brings up: Gazebo Fortress, RSP, JSP, spawn, gz_bridge,
    #    camera_bridge — all with their original timing.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    # ── 2. slam_toolbox async online mapper ───────────────
    #    Starts 12 s after launch (gz_bridge at 10 s needs 2 s
    #    to stabilise before /scan + /odom flow reliably).
    slam_node = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[slam.launch] Starting slam_toolbox ...'),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    params_file,
                    # Sync with Gazebo /clock — must be True for sim
                    {'use_sim_time': True},
                ],
                # Topic routing — these match what gz_bridge publishes:
                #   /scan         LaserScan  from gpu_lidar on lidar_link
                #   /odom         Odometry   from front diff_drive plugin
                #   /tf           TF         odom→base_link from diff_drive
                # If your LiDAR topic is different, add a remapping:
                # remappings=[('/scan', '/your_scan_topic')],
            ),
        ]
    )

    # ── 3. Teleop node (PID + SLAM status) ───────────────
    #    Opened in xterm so keyboard input works alongside Gazebo.
    #    Falls back to inline launch if no DISPLAY (headless).
    use_xterm = bool(os.environ.get('DISPLAY'))
    teleop_node = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[slam.launch] Starting teleop_hold_node ...'),
            Node(
                package='rover_description',
                executable='teleop_hold_node',
                name='teleop_hold_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                prefix='xterm -e' if use_xterm else '',
            ),
        ]
    )

    # ── 4. RViz2 (optional) ───────────────────────────────
    #    Displays: /map, /scan, /odom path, robot model.
    #    Config file is loaded from config/slam_rviz.rviz if it
    #    exists; otherwise RViz opens with an empty layout.
    rviz_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg] if os.path.isfile(rviz_cfg)
                           else [],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(rviz),
            )
        ]
    )

    return LaunchDescription([
        rviz_arg,
        gazebo_launch,
        slam_node,
        teleop_node,
        rviz_node,
    ])
