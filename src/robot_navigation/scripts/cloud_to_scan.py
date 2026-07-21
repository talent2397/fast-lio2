#!/usr/bin/env python3
"""Raw LiDAR → LaserScan: 直接用 Gazebo 原始 LiDAR, 非 FAST-LIO 累积点云"""
import math, numpy as np, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry

def _make_dtype(point_step):
    pad = max(0, point_step - 12)
    return np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32),
                     ('_pad', f'V{pad}')])

class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self._last_t = 0.0; self._n = 0
        self.n_rays = 720
        self.angle_inc = 2.0 * math.pi / self.n_rays

        # 订阅 Gazebo 原始 LiDAR (robot/lidar_link 帧)
        self.create_subscription(
            PointCloud2, '/robot/lidar', self._cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('cloud_to_scan v3 (raw LiDAR) 就绪')

    def _cloud(self, m):
        now = time.time()
        if now - self._last_t < 0.09: return  # 10Hz
        self._last_t = now

        t0 = time.perf_counter()
        try:
            dt = _make_dtype(m.point_step)
            arr = np.frombuffer(m.data, dtype=dt, count=m.width)
        except Exception:
            return

        x = arr['x'].astype(np.float64)
        y = arr['y'].astype(np.float64)

        # 高度过滤 (LiDAR在z=0.40, 忽略地面和天花板)
        z = arr['z'].astype(np.float64)
        h_mask = (np.abs(z) > 0.05) & (np.abs(z) < 3.0)
        if np.count_nonzero(h_mask) < 10: return
        x, y = x[h_mask], y[h_mask]

        # 距离
        dists = np.sqrt(x*x + y*y)
        d_mask = (dists > 0.3) & (dists < 30.0)
        if np.count_nonzero(d_mask) < 5: return
        x, y, dists = x[d_mask], y[d_mask], dists[d_mask]

        # 角度分桶
        angles = np.arctan2(y, x)
        idx = np.clip(((angles + math.pi) / self.angle_inc).astype(np.int32),
                       0, self.n_rays - 1)

        ranges = np.full(self.n_rays, np.inf, dtype=np.float32)
        order = np.argsort(idx)
        i_sorted, d_sorted = idx[order], dists[order]
        i = 0
        while i < len(i_sorted):
            bid = i_sorted[i]; j = i + 1
            while j < len(i_sorted) and i_sorted[j] == bid: j += 1
            ranges[bid] = np.min(d_sorted[i:j]); i = j

        scan = LaserScan()
        scan.header.stamp = m.header.stamp
        scan.header.frame_id = 'robot/base_link'  # LiDAR offset negligible for 2D
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.3; scan.range_max = 30.0
        scan.ranges = ranges.tolist()
        scan.scan_time = 0.1
        self.pub.publish(scan)

        self._n += 1
        if self._n % 50 == 0:
            n_valid = np.count_nonzero(~np.isinf(ranges))
            self.get_logger().info(f'scan: {m.width}pts→{n_valid}/{self.n_rays}rays '
                                   f'in {(time.perf_counter()-t0)*1000:.1f}ms',
                                   throttle_duration_sec=5.0)

def main(): rclpy.init(); rclpy.spin(CloudToScan())
if __name__ == '__main__': main()
