#!/usr/bin/env python3
"""
Ball Robot 仿真 — 一键全栈启动
用法:
  ros2 launch robot_bringup sim.launch.py
  ros2 launch robot_bringup sim.launch.py headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_bringup, 'worlds', 'ball_robot.sdf')
    bridge_config = os.path.join(pkg_bringup, 'config', 'gz_bridge.yaml')
    fastlio_params = os.path.join(pkg_bringup, 'config', 'fastlio_params.yaml')
    rviz_config = os.path.join(pkg_fast_lio, 'rviz', 'fastlio.rviz')

    # 1. Gazebo 仿真
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    # 2. ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # 3. 静态 TF: camera_init → robot/odom (world origin → odom frame)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'robot/odom'],
    )

    # 4. FAST-LIO2 建图
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[fastlio_params, {'use_sim_time': True}],
        output='screen',
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        gz_sim,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=3.5, actions=[static_tf]),
        TimerAction(period=6.0, actions=[fast_lio]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
