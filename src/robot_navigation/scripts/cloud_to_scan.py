#!/usr/bin/env python3
"""3D点云 → 2D 激光扫描 (无需 TF, 直接用odom姿态变换)"""
import math, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.pose = None  # (x, y, yaw) in camera_init frame
        self._last_scan_t = self.get_clock().now()

        self.create_subscription(Odometry, '/robot/odom', self._odom, 10)
        self.create_subscription(
            PointCloud2, '/cloud_registered', self._cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # 参数
        self.range_min = 0.3
        self.range_max = 30.0
        self.angle_inc = 2.0 * math.pi / 1440  # 0.25°
        self.height_min = 0.05
        self.height_max = 2.0
        self.get_logger().info('cloud_to_scan 就绪 (无TF)')

    def _odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)

    def _cloud(self, m):
        if self.pose is None: return
        now = self.get_clock().now()
        if (now - self._last_scan_t).nanoseconds < 9e7:  # 节流 10Hz
            return
        self._last_scan_t = now

        px, py, yaw = self.pose
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)

        scan = LaserScan()
        scan.header.stamp = m.header.stamp
        scan.header.frame_id = 'robot/base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        n_rays = 1440
        ranges = [float('inf')] * n_rays

        # 迭代点云: 只取30m内的点
        for pt in pc2.read_points(m, field_names=('x','y','z'), skip_nans=True):
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
            dx = x - px; dy = y - py
            lx = dx * cos_y - dy * sin_y
            ly = dx * sin_y + dy * cos_y

            if not (self.height_min < abs(z) < self.height_max):
                continue

            dist = math.hypot(lx, ly)
            if dist < self.range_min or dist > self.range_max:
                continue

            angle = math.atan2(ly, lx)
            if math.isnan(angle):
                continue
            idx = int((angle + math.pi) / self.angle_inc)
            idx = max(0, min(n_rays - 1, idx))
            if dist < ranges[idx]:
                ranges[idx] = dist

        scan.ranges = ranges
        scan.scan_time = 0.1
        self.pub.publish(scan)

def main(): rclpy.init(); rclpy.spin(CloudToScan())
if __name__ == '__main__': main()
