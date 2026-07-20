#!/usr/bin/env python3
"""
LiDAR-Camera 3D fusion launch — starts camera_info_publisher + lidar_camera_fusion.

Usage:
  ros2 launch robot_vision fusion.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_info = Node(
        package='robot_vision',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        parameters=[
            {'camera_width': 640},
            {'camera_height': 480},
            {'camera_fx': 554.38},
            {'camera_fy': 554.38},
            {'camera_cx': 320.0},
            {'camera_cy': 240.0},
            {'camera_frame': 'robot/camera_optical'},
            {'publish_rate': 30.0},
        ],
        output='screen',
    )

    fusion = Node(
        package='robot_vision',
        executable='lidar_camera_fusion',
        name='lidar_camera_fusion',
        parameters=[
            {'min_points_in_bbox': 5},
            {'sync_slop': 0.1},
            {'output_frame': 'robot/odom'},
            {'enable_debug_img': True},
        ],
        output='screen',
    )

    return LaunchDescription([camera_info, fusion])
