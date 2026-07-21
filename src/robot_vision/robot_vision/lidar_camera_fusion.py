#!/usr/bin/env python3
"""LiDAR-Camera 3D fusion — v2.1: debug图像 + 诊断日志 + sync_slop=0.5"""
import numpy as np, cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped
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
        self.min_points = self.declare_parameter('min_points_in_bbox', 2).value  # 小球只需2点
        self.sync_slop = self.declare_parameter('sync_slop', 0.5).value
        self.output_frame = self.declare_parameter('output_frame', 'robot/odom').value
        self.debug_img = self.declare_parameter('enable_debug_img', True).value

        self.bridge = CvBridge()
        self.cam = PinholeCameraModel(); self.cam_ok = False
        self.latest_img = None
        self._cnt = 0; self._last_proc = 0.0
        self._diag_t = 0.0

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(CameraInfo, '/robot/camera_info', self._ci_cb, 10)
        self.create_subscription(Image, '/robot/camera', self._img_cb, 10)

        lidar_sub = message_filters.Subscriber(
            self, PointCloud2, '/robot/lidar', qos_profile=qos_profile_sensor_data)
        det_sub = message_filters.Subscriber(
            self, Detection2DArray, '/robot/detections', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, det_sub], queue_size=5, slop=self.sync_slop)
        self.sync.registerCallback(self._on_sync)

        self.pub_target = self.create_publisher(PoseStamped, '/robot/target_pose', 10)
        self.pub_debug = self.create_publisher(Image, '/robot/fusion_debug_img', 10)

        self.get_logger().info(f'Fusion v3 (pixel back-projection) ready (slop={self.sync_slop}s)')

    def _ci_cb(self, msg):
        if not self.cam_ok:
            self.cam.fromCameraInfo(msg); self.cam_ok = True
            self.get_logger().info(f'Cam {msg.width}x{msg.height} frame={self.cam.tfFrame()}')

    def _img_cb(self, msg): self.latest_img = msg

    def _on_sync(self, cloud_msg, det_msg):
        self._cnt += 1

        if not self.cam_ok:
            if self._cnt == 1:
                self.get_logger().warn('Fusion: waiting for CameraInfo...')
            return

        # ── 步骤1: 读点云 ──
        try:
            ps = cloud_msg.point_step; pad = max(0, ps-12)
            dt = np.dtype([('x',np.float32),('y',np.float32),('z',np.float32),('_pad',f'V{pad}')])
            arr = np.frombuffer(cloud_msg.data, dtype=dt, count=cloud_msg.width)
        except Exception as e:
            if self._cnt % 30 == 1:
                self.get_logger().warn(f'Fusion parse error: {e}')
            return
        pts = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) < 30: return

        # ── 步骤2: TF lidar→camera (static TF, 任何时间戳都行) ──
        lidar_fr = cloud_msg.header.frame_id
        cam_fr = self.cam.tfFrame()
        stamp = rclpy.time.Time.from_msg(cloud_msg.header.stamp)

        try:
            tfm = self.tf_buf.lookup_transform(cam_fr, lidar_fr, stamp, Duration(seconds=1.0))
            m_l2c = tf_to_matrix(tfm)
        except Exception:
            try:
                tfm = self.tf_buf.lookup_transform(cam_fr, lidar_fr, rclpy.time.Time())
                m_l2c = tf_to_matrix(tfm)
            except Exception:
                return

        pts_cam = transform_pts(pts, m_l2c)
        pts_cam = pts_cam[np.all(np.isfinite(pts_cam), axis=1)]

        # ── 步骤3: 投影 ──
        fx,fy,cx,cy = self.cam.fx(), self.cam.fy(), self.cam.cx(), self.cam.cy()
        X,Y,Z = pts_cam[:,0], pts_cam[:,1], pts_cam[:,2]
        z_ok = Z > 0.05
        pts_cam,X,Y,Z = pts_cam[z_ok],X[z_ok],Y[z_ok],Z[z_ok]
        if len(pts_cam) < 10: return

        if len(pts_cam) > 600:
            pts_cam,X,Y,Z = pts_cam[::3],X[::3],Y[::3],Z[::3]

        u = fx*X/Z + cx; v = fy*Y/Z + cy
        in_img = (u>=0)&(u<self.cam.width)&(v>=0)&(v<self.cam.height)

        # ── 步骤4: 发布debug图像 ──
        if self.latest_img is not None and self._cnt % 2 == 0:
            try:
                dbg = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
                n_show = min(len(u), 800)
                step_pt = max(1, len(u)//n_show)
                for j in range(0, len(u), step_pt):
                    if in_img[j]:
                        px, py = int(u[j]), int(v[j])
                        z = Z[j]
                        if z < 0.5: color = (0,0,255)
                        elif z < 2.0: color = (0,255,255)
                        else: color = (0,255,0)
                        cv2.circle(dbg, (px, py), 1, color, -1)
                for d in det_msg.detections:
                    bb = d.bbox
                    bx,by = int(bb.center.position.x), int(bb.center.position.y)
                    bw,bh = int(bb.size_x), int(bb.size_y)
                    cx1,cy1 = bx-bw//2, by-bh//2
                    cx2,cy2 = bx+bw//2, by+bh//2
                    cid = d.results[0].hypothesis.class_id if d.results else '?'
                    cv2.rectangle(dbg, (cx1,cy1), (cx2,cy2), (255,0,0), 2)
                    cv2.putText(dbg, str(cid), (cx1,cy1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1)
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))
            except Exception:
                pass

        if not det_msg.detections:
            return

        if not np.any(in_img):
            return

        # ── 步骤5: TF camera→odom (回退时间戳找缓存, Duration(0)不阻塞) ──
        m_c2o = None
        t_cloud = stamp.nanoseconds * 1e-9
        for offset in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0]:
            t_query = rclpy.time.Time(seconds=max(0.0, t_cloud - offset))
            try:
                todom = self.tf_buf.lookup_transform(
                    self.output_frame, cam_fr, t_query, Duration(nanoseconds=0))
                m_c2o = tf_to_matrix(todom)
                break
            except Exception:
                continue
        if m_c2o is None:
            if self._cnt % 30 == 1:
                self.get_logger().warn(f'Fusion TF {cam_fr}→{self.output_frame} all stamps fail')
            return

        targets = []
        for d in det_msg.detections:
            bbox = d.bbox
            bx,by = bbox.center.position.x, bbox.center.position.y
            bw,bh = bbox.size_x, bbox.size_y
            cid = d.results[0].hypothesis.class_id if d.results else '?'
            sc = d.results[0].hypothesis.score if d.results else 0.0

            # 策略1: bbox内LiDAR点 (扩大80px捕获)
            margin = 80
            x1,y1 = bx-bw/2-margin, by-bh/2-margin
            x2,y2 = bx+bw/2+margin, by+bh/2+margin
            mask1 = in_img & (u>=x1)&(u<=x2)&(v>=y1)&(v<=y2)
            if np.sum(mask1) >= self.min_points:
                tgt_cam = np.median(pts_cam[mask1], axis=0)
                tgt_odom = (m_c2o[:3,:3] @ tgt_cam) + m_c2o[:3,3]
                targets.append((cid, sc, tgt_odom, int(np.sum(mask1))))
                continue

            # 策略2: 取bbox中心像素的深度 (反投影)
            # 在bbox周围200px找LiDAR点, 用深度中值推算球的3D位置
            img_pts = pts_cam[in_img]; img_u = u[in_img]; img_v = v[in_img]
            if len(img_pts) > 0:
                dists = np.sqrt((img_u - bx)**2 + (img_v - by)**2)
                # 取最近的50个点 (或全部, 取少者)
                n_nearby = min(50, len(dists))
                nearby_idx = np.argpartition(dists, n_nearby)[:n_nearby]
                # 深度中值 (稳健估计)
                z_est = np.median(img_pts[nearby_idx, 2])
                # 像素→相机3D: px = fx*X/Z + cx, py = fy*Y/Z + cy
                X_est = (bx - cx) * z_est / fx
                Y_est = (by - cy) * z_est / fy
                tgt_cam = np.array([X_est, Y_est, z_est])
                tgt_odom = (m_c2o[:3,:3] @ tgt_cam) + m_c2o[:3,3]
                targets.append((cid, sc, tgt_odom, n_nearby))
                if self._cnt % 20 == 1:
                    self.get_logger().info(
                        f'Fusion back-project: bbox=({bx:.0f},{by:.0f}) '
                        f'Z_est={z_est:.2f}m from {n_nearby} nearby pts')
                continue

        # v3: 像素反投影 — 不再依赖稀疏LiDAR命中
        if self._cnt % 50 == 1:
            self.get_logger().info(
                f'Fusion v3: sync={self._cnt} det={len(det_msg.detections)} '
                f'in_img_pts={int(np.sum(in_img))} targets={len(targets)}')

        if targets:
            t = targets[0]
            msg = PoseStamped(
                header=Header(stamp=cloud_msg.header.stamp, frame_id=self.output_frame),
                pose=Pose(position=Point(x=float(t[2][0]), y=float(t[2][1]), z=float(t[2][2])),
                          orientation=Quaternion(w=1.0)))
            self.pub_target.publish(msg)
            if self._cnt % 5 == 1:
                self.get_logger().info(f'Target "{t[0]}" ({t[2][0]:.2f},{t[2][1]:.2f}) '
                                       f'[{t[3]}pts] sc={t[1]:.2f}')


def main():
    rclpy.init()
    rclpy.spin(LidarCameraFusion())

if __name__ == '__main__':
    main()
