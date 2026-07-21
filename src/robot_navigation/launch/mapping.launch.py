#!/usr/bin/env python3
"""
2D 地图 — 自研 cloud_to_scan (odom姿态变换) + slam_toolbox

  为什么不用 pointcloud_to_laserscan:
    target_frame=camera_init → scan ranges 相对于世界原点, 机器人移动后即错误
    target_frame=robot/base_link → TF message_filter 丢帧
  自研方案: 用 /robot/odom 姿态手动变换, 发布 robot/base_link 帧的 scan
    → slam_toolbox 不需要 TF 变换 scan → 0 丢帧

管线:
  FAST-LIO /cloud_registered (camera_init) + /robot/odom
    → cloud_to_scan.py (numpy向量化, odom→base_link变换)
    → /scan (robot/base_link 帧)
    → slam_toolbox (scan已在base_link, 无需TF) + auto_explorer
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

    # 1. 自研 3D→2D: 用 odom 姿态手动变换, 发布 robot/base_link 帧
    pcl_node = Node(
        package='robot_navigation',
        executable='cloud_to_scan.py',
        name='cloud_to_scan',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 2. slam_toolbox
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

    return LaunchDescription([pcl_node, slam_node, configure_event, activate_event])
