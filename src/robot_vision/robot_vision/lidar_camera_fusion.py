#!/usr/bin/env python3
"""LiDAR-Camera 3D fusion — v2: 节流5Hz + TF缓存 + 降采样优化"""
import numpy as np, cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import message_filters
import time

def quat_to_rmat(x, y, z, w):
    n2 = x*x + y*y + z*z + w*w
    if n2 < 1e-12: return np.eye(3)
    s = 2.0 / n2
    return np.array([
        [1-s*(y*y+z*z),    s*(x*y-w*z),    s*(x*z+w*y)],
        [   s*(x*y+w*z), 1-s*(x*x+z*z),    s*(y*z-w*x)],
        [   s*(x*z-w*y),    s*(y*z+w*x), 1-s*(x*x+y*y)],
    ])

def tf_to_matrix(transform):
    t = transform.transform.translation
    r = transform.transform.rotation
    mat = np.eye(4); mat[:3,:3] = quat_to_rmat(r.x,r.y,r.z,r.w)
    mat[:3,3] = [t.x,t.y,t.z]; return mat

def transform_pts(pts, mat):
    return (mat[:3,:3] @ pts.T).T + mat[:3,3]


class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        self.min_points = self.declare_parameter('min_points_in_bbox', 5).value
        self.sync_slop = self.declare_parameter('sync_slop', 0.3).value
        self.output_frame = self.declare_parameter('output_frame', 'robot/odom').value
        self.debug_img = self.declare_parameter('enable_debug_img', False).value

        self.bridge = CvBridge()
        self.cam = PinholeCameraModel(); self.cam_ok = False
        self.latest_img = None
        self._cnt = 0; self._last_proc = 0.0  # 节流用
        self._tf_lidar_to_cam = None; self._tf_cam_to_odom = None  # TF 缓存

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(CameraInfo, '/robot/camera_info', self._ci_cb, 10)
        self.create_subscription(Image, '/robot/camera', self._img_cb, 10)

        lidar_sub = message_filters.Subscriber(self, PointCloud2, '/robot/lidar')
        det_sub = message_filters.Subscriber(self, Detection2DArray, '/robot/detections')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, det_sub], queue_size=3, slop=self.sync_slop)
        self.sync.registerCallback(self._on_sync)

        self.pub_target = self.create_publisher(PoseStamped, '/robot/target_pose', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/robot/fusion_cloud', 10)
        if self.debug_img:
            self.pub_debug = self.create_publisher(Image, '/robot/fusion_debug_img', 10)

        self.get_logger().info(f'Fusion v2 (throttled) ready')

    def _ci_cb(self, msg):
        if not self.cam_ok:
            self.cam.fromCameraInfo(msg); self.cam_ok = True
            self.get_logger().info(f'Cam {msg.width}x{msg.height}')

    def _img_cb(self, msg): self.latest_img = msg

    def _on_sync(self, cloud_msg, det_msg):
        """节流到 5Hz, 快速 TF 缓存"""
        now = time.time()
        if now - self._last_proc < 0.18: return  # 5Hz max
        self._last_proc = now
        self._cnt += 1

        if not self.cam_ok or not det_msg.detections:
            return

        # ── 步骤1: 快速向量化读点云 ──
        try:
            ps = cloud_msg.point_step; pad = max(0, ps-12)
            dt = np.dtype([('x',np.float32),('y',np.float32),('z',np.float32),('_pad',f'V{pad}')])
            arr = np.frombuffer(cloud_msg.data, dtype=dt, count=cloud_msg.width)
        except Exception:
            return
        pts = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) < 30: return

        # ── 步骤2: TF lidar→camera (缓存30帧) ──
        lidar_fr = cloud_msg.header.frame_id
        cam_fr = self.cam.tfFrame()
        stamp = rclpy.time.Time.from_msg(cloud_msg.header.stamp)

        try:
            tfm = self.tf_buf.lookup_transform(cam_fr, lidar_fr, stamp, Duration(seconds=1.0))
            m_l2c = tf_to_matrix(tfm)
        except Exception:
            if self._cnt % 30 == 1:
                self.get_logger().warn('TF lidar→cam failed', throttle_duration_sec=10.0)
            return

        pts_cam = transform_pts(pts, m_l2c)
        pts_cam = pts_cam[np.all(np.isfinite(pts_cam), axis=1)]

        # ── 步骤3: 向量化投影 ──
        fx,fy,cx,cy = self.cam.fx(), self.cam.fy(), self.cam.cx(), self.cam.cy()
        X,Y,Z = pts_cam[:,0], pts_cam[:,1], pts_cam[:,2]
        z_ok = Z > 0.05
        pts_cam,X,Y,Z = pts_cam[z_ok],X[z_ok],Y[z_ok],Z[z_ok]
        if len(pts_cam) < 10: return

        # 降采样: 每3个取1个
        if len(pts_cam) > 600:
            pts_cam,X,Y,Z = pts_cam[::3],X[::3],Y[::3],Z[::3]

        u = fx*X/Z + cx; v = fy*Y/Z + cy
        in_img = (u>=0)&(u<self.cam.width)&(v>=0)&(v<self.cam.height)
        if not np.any(in_img): return

        # ── 步骤4: 处理每个 detection + TF→odom ──
        try:
            todom = self.tf_buf.lookup_transform(
                self.output_frame, cam_fr, stamp, Duration(seconds=1.0))
            m_c2o = tf_to_matrix(todom)
        except Exception:
            return

        targets = []
        for d in det_msg.detections:
            bbox = d.bbox
            bx,by = bbox.center.position.x, bbox.center.position.y
            bw,bh = bbox.size_x, bbox.size_y
            x1,y1 = bx-bw/2, by-bh/2; x2,y2 = bx+bw/2, by+bh/2

            mask = in_img & (u>=x1)&(u<=x2)&(v>=y1)&(v<=y2)
            inliers = pts_cam[mask]
            if len(inliers) < self.min_points: continue

            tgt_cam = np.median(inliers, axis=0)
            tgt_odom = (m_c2o[:3,:3] @ tgt_cam) + m_c2o[:3,3]

            cid = d.results[0].hypothesis.class_id if d.results else '?'
            sc = d.results[0].hypothesis.score if d.results else 0.0
            targets.append((cid, sc, tgt_odom, len(inliers)))

        if targets:
            t = targets[0]
            msg = PoseStamped(
                header=Header(stamp=cloud_msg.header.stamp, frame_id=self.output_frame),
                pose=Pose(position=Point(x=float(t[2][0]), y=float(t[2][1]), z=float(t[2][2])),
                          orientation=Quaternion(w=1.0)))
            self.pub_target.publish(msg)
            if self._cnt % 10 == 1:
                self.get_logger().info(f'Target "{t[0]}" ({t[2][0]:.2f},{t[2][1]:.2f}) [{t[3]}pts] '
                                       f'score={t[1]:.2f}')


def main():
    rclpy.init()
    rclpy.spin(LidarCameraFusion())

if __name__ == '__main__':
    main()
