#!/usr/bin/env python3
"""Nav2 导航 — 手动 lifecycle（跳过 lifecycle_manager 和 collision_monitor）"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')
    params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    script = os.path.join(pkg_nav, 'scripts', 'activate_nav2.sh')

    nodes = [
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', parameters=[params], output='screen'),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=[params], output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', parameters=[params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', parameters=[params], output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[params],
             remappings=[('cmd_vel', '/robot/cmd_vel')], output='screen'),
    ]

    activate = TimerAction(period=6.0, actions=[
        ExecuteProcess(cmd=['bash', script], output='screen', name='nav2_activate'),
    ])

    return LaunchDescription([*nodes, activate])
