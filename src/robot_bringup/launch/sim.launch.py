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
    pkg_navigation = get_package_share_directory('robot_navigation')

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

    # 3.0 静态 TF: map → robot/odom (identity, Nav2 激活需要)
    # slam_toolbox 启动后会动态更新此变换
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot/odom'],
    )

    # 3.1 静态 TF: robot/base_link → robot/lidar_link (SDF: 0, 0, 0.40)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.40', '0', '0', '0', 'robot/base_link', 'robot/lidar_link'],
    )

    # 3.2 静态 TF: robot/base_link → robot/camera_link (SDF: 0.42, 0, 0.30)
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.42', '0', '0.30', '0', '0', '0', 'robot/base_link', 'robot/camera_link'],
    )

    # 3.3 静态 TF: link → sensor (identity)
    static_tf_lidar_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_to_sensor',
        arguments=['0', '0', '0', '0', '0', '0', 'robot/lidar_link', 'robot/lidar_link/lidar_sensor'],
    )
    static_tf_camera_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_sensor',
        arguments=['0', '0', '0', '0', '0', '0', 'robot/camera_link', 'robot/camera_link/camera_sensor'],
    )

    # 3.4 静态 TF: camera sensor (ROS frame) → camera optical (OpenCV frame)
    # ROS: x-forward, y-left, z-up  →  OpenCV: x-right, y-down, z-forward
    # Rotation: yaw=PI/2 pitch=-PI/2  →  quaternion (qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)
    static_tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_optical',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5',
                   'robot/camera_link/camera_sensor', 'robot/camera_optical'],
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

    # 6. Vision pipeline: camera_info → YOLO detector → LiDAR-Camera fusion
    camera_info = Node(
        package='robot_vision',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        parameters=[
            {'camera_frame': 'robot/camera_optical'},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    yolo_detector = Node(
        package='robot_vision',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[
            {'inference_every': 1},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    lidar_fusion = Node(
        package='robot_vision',
        executable='lidar_camera_fusion',
        name='lidar_camera_fusion',
        parameters=[
            {'min_points_in_bbox': 5},
            {'sync_slop': 0.2},
            {'output_frame': 'robot/odom'},
            {'enable_debug_img': True},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # 7. 2D 建图: pointcloud_to_laserscan + slam_toolbox
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'mapping.launch.py')
        ),
    )

    # 8. 全自动探索导航: 边建图边探索 + A* 路径 + 目标追踪
    auto_explorer = Node(
        package='robot_navigation',
        executable='auto_explorer',
        name='auto_explorer',
        parameters=[{
            'use_sim_time': True,
            'max_v': 1.5,
            'max_w': 1.5,
            'goal_dist': 2.0,
            'obs_stop': 0.8,
            'spin_speed': 0.8,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        gz_sim,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=3.5, actions=[static_tf, static_tf_map_odom, static_tf_lidar, static_tf_camera, static_tf_lidar_sensor, static_tf_camera_sensor, static_tf_camera_optical]),
        TimerAction(period=5.0, actions=[camera_info]),
        TimerAction(period=5.5, actions=[yolo_detector]),
        TimerAction(period=6.0, actions=[lidar_fusion]),
        TimerAction(period=6.0, actions=[fast_lio]),
        TimerAction(period=8.0, actions=[mapping]),      # 等 FAST-LIO 出云
        TimerAction(period=10.0, actions=[auto_explorer]),
        TimerAction(period=14.0, actions=[rviz]),
    ])
