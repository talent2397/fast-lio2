#!/usr/bin/env python3
"""
2D 地图与导航启动 — pointcloud_to_laserscan + slam_toolbox

地图管线:
  FAST-LIO /cloud_registered → pointcloud_to_laserscan → /scan
  /scan + /robot/odom → slam_toolbox (online_async) → /map

用法:
  ros2 launch robot_navigation mapping.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 3D 点云 → 2D 激光扫描
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'robot/base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.05,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043633,  # ~0.25 deg, 1440 rays
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 2. slam_toolbox — 2D 占据栅格建图 (online_async)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            '/home/c/fastlio_ws/src/robot_navigation/config/slam_toolbox.yaml',
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([pointcloud_to_laserscan, slam_toolbox])
