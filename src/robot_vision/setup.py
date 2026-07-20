#!/usr/bin/env python3
"""robot_vision — YOLO detection + LiDAR-camera 3D localization"""
import os
from setuptools import setup
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='talent2397',
    maintainer_email='talent2397@gmail.com',
    description='Vision-based object detection for robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_detector = robot_vision.yolo_detector:main',
            'lidar_camera_fusion = robot_vision.lidar_camera_fusion:main',
            'camera_info_publisher = robot_vision.camera_info_publisher:main',
            'cmd_vel_relay = robot_vision.cmd_vel_relay:main',
        ],
    },
)
