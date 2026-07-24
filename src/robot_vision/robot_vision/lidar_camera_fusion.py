"""LiDAR-Camera 3D fusion — v7: 球心三角测量 + 异常值过滤 + EMA限速 + bbox边沿拒绝 + Gazebo odom坐标"""
import math
import numpy as np, cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import Odometry
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
        self._ema_pos = None; self._ema_alpha = 0.25  # v7: 0.5→0.25 防单次坏测量拉偏50%
        self._ema_raw_count = 0      # v7: 累计有效测量次数(前几次用更高alpha快速初始化)
        self._last_raw = None        # v7: 上一次原始测量(异常值检测用)

        # v7: 改用Gazebo odometry计算camera→world变换(独立于FAST-LIO2 TF)
        self.odom_pos = None; self.odom_yaw = 0.0  # Gazebo ground-truth pose
        self.create_subscription(Odometry, '/robot/odom', self._odom_cb, 10)

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
        self.get_logger().info('Fusion v7 (ball_center_triangulation + outlier_reject + EMA limit) ready')

    def _ci(self, m):
        if not self.cam_ok:
            self.cam.fromCameraInfo(m); self.cam_ok = True
            self.get_logger().info(f'Cam {m.width}x{m.height} frame={self.cam.tfFrame()}')

    def _img(self, m): self.latest_img = m

    def _odom_cb(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.odom_pos = np.array([p.x, p.y, p.z])
        self.odom_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def _cam_to_odom(self, pt_cam):
        """v7: 用Gazebo odometry将点从camera_optical变换到world/odom
        camera_optical: X=right, Y=down, Z=forward
        camera_sensor(=robot convention): X=forward, Y=left, Z=up
        TF optical→sensor: R*(x_o,y_o,z_o) = (z_o, -x_o, -y_o)_sensor
        sensor→base_link: + (0.42, 0, 0.30)
        base_link→odom: Gazebo odometry pose
        """
        x_c, y_c, z_c = pt_cam
        # Step 1: camera_optical → base_link
        # optical→sensor R: (z_o, -x_o, -y_o) + translation (0.42, 0, 0.30)
        x_b = z_c + 0.42    # optical forward → base X
        y_b = -x_c           # optical right → base -Y (right in left-handed robot frame)
        z_b = -y_c + 0.30    # optical down → base -Z
        # Step 2: base_link → odom (Gazebo odometry)
        cy, sy = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        x_o = self.odom_pos[0] + cy*x_b - sy*y_b
        y_o = self.odom_pos[1] + sy*x_b + cy*y_b
        return np.array([x_o, y_o])

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

        # ── v7: 用Gazebo odometry代替FAST-LIO2 TF做camera→odom变换 ──
        if self.odom_pos is None: return  # odom未就绪时跳过

        # ── 球心三角测量 (用球心像素+球高度平面求交) ──
        targets = []
        BALL_HEIGHT = 0.35   # v7: 球心离地高度(仿真已知), 替换地面Z=0
        CAM_HEIGHT = 0.50    # 相机离地高度
        TARGET_DEPTH = CAM_HEIGHT - BALL_HEIGHT  # 0.15m — 球心在相机光轴下方距离

        for d in det_msg.detections:
            bb = d.bbox; bx,by = bb.center.position.x, bb.center.position.y
            bw,bh = bb.size_x, bb.size_y
            cid = d.results[0].hypothesis.class_id if d.results else '?'
            sc = d.results[0].hypothesis.score if d.results else 0.0

            # v7: bbox边沿拒绝 — 如果球在图像边缘(clipped), 跳过(bbox不准确)
            bbox_edge_margin = 15
            if (bx - bw/2 < bbox_edge_margin or bx + bw/2 > self.cam.width - bbox_edge_margin or
                by - bh/2 < bbox_edge_margin or by + bh/2 > self.cam.height - bbox_edge_margin):
                continue  # 裁剪的bbox → 跳过

            # 策略1: LiDAR深度窗口
            x1, y1 = bx-bw/2-30, by-bh/2-30
            x2, y2 = bx+bw/2+30, by+bh/2+30
            in_bbox = in_img & (u>=x1) & (u<=x2) & (v>=y1) & (v<=y2)
            n_in = int(np.sum(in_bbox))

            if n_in >= self.min_points:
                depths = Z[in_bbox]
                d_min = float(depths.min())
                fg = depths <= d_min + self.depth_window
                if np.sum(fg) >= self.min_points:
                    tgt_cam = np.median(pts_cam[in_bbox][fg], axis=0)
                    tgt_odom = self._cam_to_odom(tgt_cam)
                    targets.append((cid, sc, tgt_odom[:2], tgt_cam[2], int(np.sum(fg)), 'lidar'))
                    continue

            # 策略2: v7.1 地面三角测量 — 用bbox底边+地面平面 (比球心更稳定)
            # bbox底边 ≈ 球触地点, 交地面得球正下方的XY (球就在正上方0.35m的Z处)
            foot_px_y = int(min(by + bh*0.5, self.cam.height - 1))
            foot_px_x = int(min(max(bx, 0), self.cam.width - 1))
            ray = np.array(self.cam.projectPixelTo3dRay((foot_px_x, foot_px_y)))
            if ray[1] > 0.005:  # 射线指向下方
                t = CAM_HEIGHT / ray[1]  # 交地面(Y=CAM_HEIGHT=0.50)
                if 0.3 < t < 30.0:
                    tgt_cam = ray * t  # 地面点在相机帧的3D坐标
                    tgt_odom = self._cam_to_odom(tgt_cam)
                    targets.append((cid, sc, tgt_odom, tgt_cam[2], 0, 'ground'))
                    if self._cnt % 20 == 1:
                        self.get_logger().info(f'Ground-tri: foot=({foot_px_x},{foot_px_y}) '
                                               f'ray_y={ray[1]:.3f} t={t:.1f}m '
                                               f'tgt=({tgt_odom[0]:.1f},{tgt_odom[1]:.1f})')
                    continue

            # 策略2b: 回退球心三角测量 (bbox底边不可用时)
            center_px = (int(min(max(bx, 0), self.cam.width-1)),
                        int(min(max(by, 0), self.cam.height-1)))
            ray = np.array(self.cam.projectPixelTo3dRay(center_px))
            if ray[1] > 0.005:
                t = TARGET_DEPTH / ray[1]
                if 0.3 < t < 50.0:
                    tgt_cam = ray * t
                    tgt_odom = self._cam_to_odom(tgt_cam)
                    targets.append((cid, sc, tgt_odom, tgt_cam[2], 0, 'center'))
                    continue

            # 策略3: 回退取最近LiDAR点深度
            if np.any(in_img):
                d_idx = np.argmin(np.sqrt((u[in_img]-bx)**2 + (v[in_img]-by)**2))
                z_est = Z[in_img][d_idx]
                z_est = max(z_est, 0.5)
                tgt_cam = np.array([(bx-cx)*z_est/fx, (by-cy)*z_est/fy, z_est])
                tgt_odom = self._cam_to_odom(tgt_cam)
                targets.append((cid, sc, tgt_odom[:2], z_est, 0, 'fallback'))
                continue

        # ── v7: 发布 (异常值过滤 + EMA限速 + 快速初始化) ──
        if targets:
            t = targets[0]
            raw = t[2]

            # v7: 异常值过滤 — 如果与EMA偏差>5m且已有>5次有效测量, 跳过
            if self._ema_pos is not None and self._ema_raw_count > 5:
                dev = math.hypot(raw[0]-self._ema_pos[0], raw[1]-self._ema_pos[1])
                if dev > 5.0:
                    self.get_logger().warn(f'Outlier rejected: raw=({raw[0]:.1f},{raw[1]:.1f}) '
                                          f'dev={dev:.1f}m from ema=({self._ema_pos[0]:.1f},{self._ema_pos[1]:.1f})',
                                          throttle_duration_sec=3.0)
                    return  # 不发布, 不更新EMA

            # v7: EMA更新 — 前5次用更高alpha快速初始化(0.6→0.25)
            if self._ema_pos is None:
                self._ema_pos = raw
                self._ema_raw_count = 1
            else:
                effective_alpha = 0.6 if self._ema_raw_count < 5 else self._ema_alpha
                self._ema_pos = effective_alpha*raw + (1-effective_alpha)*self._ema_pos
                self._ema_raw_count += 1

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
