#!/usr/bin/env python3
"""
2D 地图 — cloud_to_scan + slam_toolbox (LifecycleNode, 延迟激活)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, TimerAction
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')
    slam_params = os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml')

    pcl_node = Node(
        package='robot_navigation',
        executable='cloud_to_scan.py',
        name='cloud_to_scan',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    # t=5s: CONFIGURE → inactive
    configure_action = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[slam_toolbox] CONFIGURE...'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )),
        ],
    )

    # t=30s: ACTIVATE → active (CONFIGURE完成后再激活)
    activate_action = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg='[slam_toolbox] ACTIVATE...'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    )

    return LaunchDescription([pcl_node, slam_node, configure_action, activate_action])
