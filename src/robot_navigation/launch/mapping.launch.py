#!/usr/bin/env python3
"""
2D 地图启动 — cloud_to_scan (无TF) + slam_toolbox

地图管线:
  FAST-LIO /cloud_registered → cloud_to_scan (odom姿态) → /scan
  /scan + /robot/odom → slam_toolbox (online_async) → /map

优势: cloud_to_scan 直接用 odom 姿态做坐标变换，不走 TF message_filter，
      彻底消除 "Message Filter dropping message" 问题。
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

    # 1. 3D 点云 → 2D 激光扫描 (自定义节点, 无 TF 依赖)
    pcl_node = Node(
        package='robot_navigation',
        executable='cloud_to_scan.py',
        name='cloud_to_scan',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 2. slam_toolbox — Lifecycle 节点
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

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
