#!/usr/bin/env python3
"""Nav2 测试 — 直接发 NavigateToPose action goal"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import time


class Nav2Tester(Node):
    def __init__(self):
        super().__init__('nav2_tester')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(3.0, self._try_send)

    def _try_send(self):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待 navigate_to_pose action server...')
            return
        self.timer.cancel()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position = Point(x=5.0, y=0.0, z=0.0)
        goal.pose.pose.orientation = Quaternion(w=1.0)
        self.get_logger().info('发送 goal (5,0)...')
        self.client.send_goal_async(goal).add_done_callback(self._on_response)

    def _on_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal 被拒绝! Nav2 未就绪或地图无此位置')
            raise SystemExit
        self.get_logger().info('Goal 已接受，导航中...')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        if status == 4:
            self.get_logger().info('导航成功!')
        else:
            self.get_logger().error(f'导航失败: status={status}')
        raise SystemExit


def main():
    rclpy.init()
    node = Nav2Tester()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()
