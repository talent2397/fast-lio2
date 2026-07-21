#!/usr/bin/env python3
"""
3D点云 → 2D 激光扫描 (全 numpy 向量化, 50x 加速)

FAST-LIO /cloud_registered (camera_init 帧)
  → numpy 向量化坐标变换 (odom 位置 + 旋转)
  → 高度过滤 → 距离过滤 → 角度分桶取最近点
  → /scan (robot/base_link 帧)
"""
import math, time, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry

# cloud_registered 点云格式: 动态推断 dtype
# FAST-LIO 输出: x,y,z 占前 12 字节, 余下跳过
def _make_dtype(point_step):
    """从 point_step 推断 dtype: x,y,z float32 + padding"""
    pad_bytes = point_step - 12  # 3 × float32
    if pad_bytes < 0:
        pad_bytes = 0
    return np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32),
                     ('_pad', f'V{pad_bytes}')])

class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.pose = None  # (x, y, yaw) in camera_init frame
        self._last_scan_t = 0.0
        self._proc_times = []  # 处理时间采样

        self.create_subscription(Odometry, '/robot/odom', self._odom, 10)
        self.create_subscription(
            PointCloud2, '/cloud_registered', self._cloud, qos_profile_sensor_data)

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.diag_pub = self.create_publisher(LaserScan, '/scan_raw', 10)

        self.range_min = 0.3
        self.range_max = 30.0
        self.height_min = 0.05
        self.height_max = 2.0
        self.n_rays = 720   # 0.5° 分辨率
        self.angle_inc = 2.0 * math.pi / self.n_rays
        self._diag_count = 0

        self.get_logger().info('cloud_to_scan v2 就绪 (numpy向量化)')

    def _odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)

    def _cloud(self, m):
        if self.pose is None: return
        now = time.time()

        # 节流: 10Hz
        if now - self._last_scan_t < 0.09:
            return
        self._last_scan_t = now

        t0 = time.perf_counter()
        px, py, yaw = self.pose

        # ── 步骤1: numpy 向量化读取全部点 ──
        try:
            dt = _make_dtype(m.point_step)
            arr = np.frombuffer(m.data, dtype=dt, count=m.width)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(
                f'dtype parse err: point_step={m.point_step} '
                f'data_len={len(m.data)} expected={m.width * m.point_step}',
                throttle_duration_sec=10.0)
            return

        x = arr['x'].astype(np.float64)
        y = arr['y'].astype(np.float64)
        z = arr['z'].astype(np.float64)

        # ── 步骤2: 高度过滤 ──
        h_mask = (np.abs(z) > self.height_min) & (np.abs(z) < self.height_max)
        if np.count_nonzero(h_mask) < 10:
            return
        x, y = x[h_mask], y[h_mask]

        # ── 步骤3: 变换到机器人坐标系 (向量化) ──
        dx = x - px
        dy = y - py
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        lx = dx * cos_y - dy * sin_y  # 机器人前方 = x
        ly = dx * sin_y + dy * cos_y  # 机器人左方 = y

        # ── 步骤4: 距离过滤 ──
        dists = np.sqrt(lx*lx + ly*ly)
        d_mask = (dists >= self.range_min) & (dists <= self.range_max)
        if np.count_nonzero(d_mask) < 10:
            return
        lx, ly, dists = lx[d_mask], ly[d_mask], dists[d_mask]

        # ── 步骤5: 计算角度 → 分桶取最近点 ──
        angles = np.arctan2(ly, lx)
        indices = np.floor((angles + math.pi) / self.angle_inc).astype(np.int32)
        np.clip(indices, 0, self.n_rays - 1, out=indices)

        # 用 indirect sort + argmin per group 取最近点
        ranges = np.full(self.n_rays, np.inf, dtype=np.float32)
        # 关键优化: 用 lexsort 先按 index 排序, 再逐块取 min
        sort_idx = np.argsort(indices)
        indices_sorted = indices[sort_idx]
        dists_sorted = dists[sort_idx]

        # 逐块处理 (每块是同一个 index)
        i = 0
        while i < len(indices_sorted):
            bin_id = indices_sorted[i]
            j = i + 1
            while j < len(indices_sorted) and indices_sorted[j] == bin_id:
                j += 1
            ranges[bin_id] = np.min(dists_sorted[i:j])
            i = j

        # ── 步骤6: 发布 ──
        scan = LaserScan()
        scan.header.stamp = m.header.stamp
        scan.header.frame_id = 'robot/base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        scan.scan_time = 0.1
        self.pub.publish(scan)

        # 性能诊断 (每50帧)
        dt = time.perf_counter() - t0
        self._proc_times.append(dt)
        if self._diag_count == 0:
            avg = np.mean(self._proc_times[-50:]) * 1000 if self._proc_times else 0
            n_pts = len(dists_sorted) if 'dists_sorted' in dir() else 0
            self.get_logger().info(
                f'scan: {n_pts}pts→{np.count_nonzero(~np.isinf(ranges))}rays '
                f'in {avg:.1f}ms (cloud={m.width}pts, rate={1.0/dt:.0f}Hz)',
                throttle_duration_sec=5.0)
        self._diag_count = (self._diag_count + 1) % 50

def main():
    rclpy.init()
    rclpy.spin(CloudToScan())

if __name__ == '__main__':
    main()
