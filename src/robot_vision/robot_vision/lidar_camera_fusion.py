"""LiDAR-Camera 3D fusion — v6: 深度窗口 + 地面平面三角测量 双策略（借鉴 human_tracking）"""
import numpy as np, cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import message_filters
import time

def quat_to_rmat(x, y, z, w):
    n2 = x*x + y*y + z*z + w*w
    if n2 < 1e-12: return np.eye(3)
    s = 2.0 / n2
    return np.array([[1-s*(y*y+z*z), s*(x*y-w*z), s*(x*z+w*y)],[s*(x*y+w*z), 1-s*(x*x+z*z), s*(y*z-w*x)],[s*(x*z-w*y), s*(y*z+w*x), 1-s*(x*x+y*y)]])

def tf_to_matrix(t):
    r = t.transform.rotation; tr = t.transform.translation
    m = np.eye(4); m[:3,:3] = quat_to_rmat(r.x,r.y,r.z,r.w); m[:3,3] = [tr.x,tr.y,tr.z]; return m

def transform_pts(pts, mat):
    return (mat[:3,:3] @ pts.T).T + mat[:3,3]


class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        self.min_points = 2
        self.depth_window = 0.8
        self.output_frame = 'robot/odom'
        self.cam = PinholeCameraModel(); self.cam_ok = False
        self.latest_img = None; self._cnt = 0
        self._ema_pos = None; self._ema_alpha = 0.3

        import tf2_ros
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(CameraInfo, '/robot/camera_info', self._ci, 10)
        self.create_subscription(Image, '/robot/camera', self._img, 10)

        from rclpy.qos import qos_profile_sensor_data
        ls = message_filters.Subscriber(self, PointCloud2, '/robot/lidar', qos_profile=qos_profile_sensor_data)
        ds = message_filters.Subscriber(self, Detection2DArray, '/robot/detections', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([ls, ds], queue_size=5, slop=0.5)
        self.sync.registerCallback(self._on_sync)

        self.pub_tgt = self.create_publisher(PoseStamped, '/robot/target_pose', 10)
        self.pub_dbg = self.create_publisher(Image, '/robot/fusion_debug_img', 10)
        self.get_logger().info('Fusion v6 (depth_window + ground_triangulation) ready')

    def _ci(self, m):
        if not self.cam_ok:
            self.cam.fromCameraInfo(m); self.cam_ok = True
            self.get_logger().info(f'Cam {m.width}x{m.height} frame={self.cam.tfFrame()}')

    def _img(self, m): self.latest_img = m

    def _on_sync(self, cloud_msg, det_msg):
        self._cnt += 1
        if not self.cam_ok or not det_msg.detections: return

        # ── 读点云 ──
        ps = cloud_msg.point_step; pad = max(0, ps-12)
        dt = np.dtype([('x',np.float32),('y',np.float32),('z',np.float32),('_pad',f'V{pad}')])
        arr = np.frombuffer(cloud_msg.data, dtype=dt, count=cloud_msg.width)
        pts_lidar = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        pts_lidar = pts_lidar[np.all(np.isfinite(pts_lidar), axis=1)]
        if len(pts_lidar) < 5: return

        # ── TF lidar→camera ──
        lidar_fr, cam_fr = cloud_msg.header.frame_id, self.cam.tfFrame()
        stamp = rclpy.time.Time.from_msg(cloud_msg.header.stamp)
        m_l2c = None
        for ts in [stamp, rclpy.time.Time()]:
            try:
                m_l2c = tf_to_matrix(self.tf_buf.lookup_transform(cam_fr, lidar_fr, ts, Duration(seconds=1)))
                break
            except Exception: continue
        if m_l2c is None: return

        pts_cam = transform_pts(pts_lidar, m_l2c)
        pts_cam = pts_cam[np.all(np.isfinite(pts_cam), axis=1)]

        # ── 投影 ──
        fx,fy,cx,cy = self.cam.fx(), self.cam.fy(), self.cam.cx(), self.cam.cy()
        X,Y,Z = pts_cam[:,0], pts_cam[:,1], pts_cam[:,2]
        ok = Z > 0.05
        pts_cam,X,Y,Z,pts_lidar = pts_cam[ok],X[ok],Y[ok],Z[ok],pts_lidar[ok]
        if len(pts_cam) < 3: return
        if len(pts_cam) > 800:  # 降采样
            idx = np.linspace(0, len(pts_cam)-1, 800, dtype=int)
            pts_cam,X,Y,Z,pts_lidar = pts_cam[idx],X[idx],Y[idx],Z[idx],pts_lidar[idx]
        u = fx*X/Z + cx; v = fy*Y/Z + cy
        in_img = (u>=0) & (u<self.cam.width) & (v>=0) & (v<self.cam.height)

        # ── Debug图像 ──
        if self.latest_img and self._cnt % 2 == 0:
            try:
                dbg = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
                step_p = max(1, len(u)//600)
                for j in range(0, len(u), step_p):
                    if in_img[j]:
                        clr = (0,0,255) if Z[j]<0.5 else ((0,255,255) if Z[j]<2 else (0,255,0))
                        cv2.circle(dbg, (int(u[j]), int(v[j])), 1, clr, -1)
                for d in det_msg.detections:
                    bb = d.bbox; bx,by = int(bb.center.position.x), int(bb.center.position.y)
                    bw,bh = int(bb.size_x), int(bb.size_y)
                    cv2.rectangle(dbg, (bx-bw//2, by-bh//2), (bx+bw//2, by+bh//2), (255,0,0), 2)
                    cid = d.results[0].hypothesis.class_id if d.results else '?'
                    cv2.putText(dbg, str(cid), (bx-bw//2, by-bh//2-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1)
                self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))
            except Exception: pass

        # ── TF camera→odom ──
        m_c2o = None
        for ts in [stamp, rclpy.time.Time()]:
            try:
                m_c2o = tf_to_matrix(self.tf_buf.lookup_transform(self.output_frame, cam_fr, ts, Duration(seconds=1)))
                break
            except Exception: continue
        if m_c2o is None: return

        # ── 地面平面三角测量 (v6: 当LiDAR打不到球时, 用bbox底边+地面平面解算3D) ──
        targets = []
        for d in det_msg.detections:
            bb = d.bbox; bx,by = bb.center.position.x, bb.center.position.y
            bw,bh = bb.size_x, bb.size_y
            cid = d.results[0].hypothesis.class_id if d.results else '?'
            sc = d.results[0].hypothesis.score if d.results else 0.0

            # 策略1: LiDAR深度窗口
            x1, y1 = bx-bw/2-30, by-bh/2-30  # 扩大30px
            x2, y2 = bx+bw/2+30, by+bh/2+30
            in_bbox = in_img & (u>=x1) & (u<=x2) & (v>=y1) & (v<=y2)
            n_in = int(np.sum(in_bbox))

            if n_in >= self.min_points:
                depths = Z[in_bbox]
                d_min = float(depths.min())
                fg = depths <= d_min + self.depth_window
                if np.sum(fg) >= self.min_points:
                    tgt_cam = np.median(pts_cam[in_bbox][fg], axis=0)
                    tgt_odom = transform_pts(np.atleast_2d(tgt_cam), m_c2o)[0]
                    targets.append((cid, sc, tgt_odom[:2], tgt_cam[2], int(np.sum(fg)), 'lidar'))
                    continue

            # 策略2: 地面平面三角测量
            # 相机光学帧: x右 y下 z前. 地面在y=cam_height(摄像头离地≈0.50m)
            foot_px_y = int(min(by + bh/2, self.cam.height - 1))
            foot_px_x = int(min(max(bx, 0), self.cam.width - 1))
            ray = np.array(self.cam.projectPixelTo3dRay((foot_px_x, foot_px_y)))
            cam_height = 0.50  # camera离地高度
            if ray[1] > 0.01:  # 射线指向下方(地面)
                t = cam_height / ray[1]
                tgt_cam = ray * t  # 相机光学帧3D坐标
                tgt_odom = transform_pts(np.atleast_2d(tgt_cam), m_c2o)[0]
                targets.append((cid, sc, tgt_odom[:2], tgt_cam[2], 0, 'ground'))
                if self._cnt % 20 == 1:
                    self.get_logger().info(f'Ground-tri: foot=({foot_px_x},{foot_px_y}) '
                                           f'ray_y={ray[1]:.2f} t={t:.1f}m '
                                           f'tgt=({tgt_odom[0]:.1f},{tgt_odom[1]:.1f})')
                continue

            # 策略3: 回退取最近LiDAR点深度
            if np.any(in_img):
                d_idx = np.argmin(np.sqrt((u[in_img]-bx)**2 + (v[in_img]-by)**2))
                z_est = Z[in_img][d_idx]
                z_est = max(z_est, 0.5)
                tgt_cam = np.array([(bx-cx)*z_est/fx, (by-cy)*z_est/fy, z_est])
                tgt_odom = transform_pts(np.atleast_2d(tgt_cam), m_c2o)[0]
                targets.append((cid, sc, tgt_odom[:2], z_est, 0, 'fallback'))
                continue

        # ── 发布 ──
        if targets:
            t = targets[0]
            raw = t[2]
            if self._ema_pos is None: self._ema_pos = raw
            else: self._ema_pos = 0.3*raw + 0.7*self._ema_pos
            msg = PoseStamped(header=Header(stamp=cloud_msg.header.stamp, frame_id=self.output_frame),
                              pose=Pose(position=Point(x=float(self._ema_pos[0]), y=float(self._ema_pos[1])),
                                        orientation=Quaternion(w=1.0)))
            self.pub_tgt.publish(msg)
            if self._cnt % 5 == 1:
                self.get_logger().info(f'Target "{t[0]}" ema=({self._ema_pos[0]:.2f},{self._ema_pos[1]:.2f}) '
                                       f'Z={t[3]:.2f}m src={t[5]}')

        if self._cnt % 50 == 1:
            src_counts = {}
            for x in targets: src_counts[x[5]] = src_counts.get(x[5], 0) + 1
            self.get_logger().info(f'Fusion v6: sync={self._cnt} det={len(det_msg.detections)} '
                                   f'in_img={int(np.sum(in_img))} targets={len(targets)} src={src_counts}')


def main(): rclpy.init(); rclpy.spin(LidarCameraFusion())
if __name__ == '__main__': main()
