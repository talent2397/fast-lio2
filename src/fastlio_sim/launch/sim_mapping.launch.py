#!/usr/bin/env python3
"""
FAST-LIO2 Gazebo 仿真建图 — 一键启动
用法:
  ros2 launch fastlio_sim sim_mapping.launch.py
  ros2 launch fastlio_sim sim_mapping.launch.py headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory('fastlio_sim')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    headless = LaunchConfiguration('headless')
    world_file = os.path.join(pkg_sim, 'worlds', 'ball_robot.sdf')
    bridge_config = os.path.join(pkg_sim, 'config', 'gz_bridge.yaml')
    velodyne_config = os.path.join(pkg_sim, 'config', 'velodyne.yaml')

    # 1. Gazebo
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
        name='fastlio_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # 2.5 静态 TF: camera_init → diff_car/odom (world-to-odom, 模拟从原点开始)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'diff_car/odom'],
    )

    # 2.6 静态 TF: diff_car/chassis → body (别名, RViz/FAST-LIO 用 body 作为基座坐标系)
    static_tf_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_chassis_to_body',
        arguments=['0', '0', '0', '0', '0', '0', 'diff_car/chassis', 'body'],
    )

    # 3. FAST-LIO2 mapping
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[velodyne_config, {'use_sim_time': True}],
        output='screen',
    )

    # 4. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_fast_lio, 'rviz', 'fastlio.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        gz_sim,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=3.5, actions=[static_tf_odom, static_tf_body]),
        TimerAction(period=6.0, actions=[fast_lio]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
