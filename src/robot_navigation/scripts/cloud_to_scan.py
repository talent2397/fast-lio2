#!/usr/bin/env python3
"""Raw LiDAR → LaserScan: 点云在lidar_link局部帧, z≈0为传感器高度"""
import math, numpy as np, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan

def _make_dtype(point_step):
    pad = max(0, point_step - 12)
    return np.dtype([('x',np.float32),('y',np.float32),('z',np.float32),('_pad',f'V{pad}')])

class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self._last_t = 0.0; self._n = 0
        self.n_rays = 720; self.angle_inc = 2.0*math.pi/self.n_rays
        self.create_subscription(PointCloud2, '/robot/lidar', self._cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('cloud_to_scan v5 (local frame, correct z filter) 就绪')

    def _cloud(self, m):
        now = time.time()
        if now - self._last_t < 0.09: return
        self._last_t = now; t0 = time.perf_counter()
        try:
            dt = _make_dtype(m.point_step)
            arr = np.frombuffer(m.data, dtype=dt, count=m.width)
        except: return

        x = arr['x'].astype(np.float64); y = arr['y'].astype(np.float64)
        z = arr['z'].astype(np.float64)

        # 高度: 地面z≈-0.58, z>-0.55保留障碍; z<2.5过滤天花板
        hm = (z > -0.55) & (z < 2.5)
        if np.count_nonzero(hm) < 10: return
        x, y = x[hm], y[hm]

        dists = np.sqrt(x*x + y*y)
        dm = (dists > 0.5) & (dists < 30.0)
        if np.count_nonzero(dm) < 5: return
        x, y, dists = x[dm], y[dm], dists[dm]

        angles = np.arctan2(y, x)
        idx = np.clip(((angles+math.pi)/self.angle_inc).astype(np.int32), 0, self.n_rays-1)
        ranges = np.full(self.n_rays, np.inf, dtype=np.float32)
        order = np.argsort(idx); i_s, d_s = idx[order], dists[order]
        i=0
        while i < len(i_s):
            bid = i_s[i]; j = i+1
            while j < len(i_s) and i_s[j]==bid: j+=1
            ranges[bid] = np.min(d_s[i:j]); i=j

        n_valid = int(np.count_nonzero(~np.isinf(ranges)))
        scan = LaserScan()
        scan.header.stamp = m.header.stamp; scan.header.frame_id = 'robot/base_link'
        scan.angle_min = -math.pi; scan.angle_max = math.pi
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.5; scan.range_max = 30.0
        scan.ranges = ranges.tolist(); scan.scan_time = 0.1
        self.pub.publish(scan)

        self._n += 1
        if self._n % 50 == 0:
            self.get_logger().info(f'scan: {m.width}pt→{n_valid}/{self.n_rays}rays '
                                   f'in {(time.perf_counter()-t0)*1000:.1f}ms',
                                   throttle_duration_sec=5.0)

def main(): rclpy.init(); rclpy.spin(CloudToScan())
if __name__=='__main__': main()
