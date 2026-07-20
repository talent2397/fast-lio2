#!/usr/bin/env python3
"""
2D 地图启动 — pointcloud_to_laserscan + slam_toolbox

地图管线:
  FAST-LIO /cloud_registered → pointcloud_to_laserscan → /scan
  /scan + /robot/odom → slam_toolbox (online_async) → /map

slam_toolbox 是 Lifecycle 节点，必须发送 CONFIGURE→ACTIVATE 事件。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')
    slam_params = os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml')

    # 1. 3D 点云 → 2D 激光扫描
    pcl_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'robot/base_link',
            'transform_tolerance': 0.5,
            'min_height': 0.05,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043633,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 2. slam_toolbox — Lifecycle 节点 (namespace 必须指定)
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    # CONFIGURE 事件
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    # ACTIVATE 事件（CONFIGURE 完成后自动触发）
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam_toolbox] 激活中...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    return LaunchDescription([
        pcl_node,
        slam_node,
        configure_event,
        activate_event,
    ])
