#!/usr/bin/env python3
"""YOLO detector launch file"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo = Node(
        package='robot_vision',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[
            {'model': 'yolov8n.pt'},
            {'inference_every': 3},  # run on every 3rd frame (~10 fps from 30fps)
        ],
        output='screen',
    )
    return LaunchDescription([yolo])
