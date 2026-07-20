#!/usr/bin/env python3
"""LiDAR-Camera 3D fusion — 向量化投影 + 中值定位 → /robot/target_pose"""
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


def quat_to_rmat(x, y, z, w):
    """Quaternion (xyzw) → 3x3 rotation matrix (pure numpy, no scipy)."""
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
    mat = np.eye(4)
    mat[:3, :3] = quat_to_rmat(r.x, r.y, r.z, r.w)
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def transform_pts(pts, mat):
    """Apply 4x4 homogeneous transform to (N,3) points."""
    return (mat[:3, :3] @ pts.T).T + mat[:3, 3]


class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        self.min_points = self.declare_parameter('min_points_in_bbox', 5).value
        self.sync_slop = self.declare_parameter('sync_slop', 0.3).value
        self.output_frame = self.declare_parameter('output_frame', 'robot/odom').value
        self.debug_img = self.declare_parameter('enable_debug_img', False).value  # 默认关 debug 省性能

        self.bridge = CvBridge()
        self.cam = PinholeCameraModel()
        self.cam_ok = False
        self.latest_img = None
        self.call_cnt = 0

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(CameraInfo, '/robot/camera_info', self._ci_cb, 10)
        self.create_subscription(Image, '/robot/camera', self._img_cb, 10)

        lidar_sub = message_filters.Subscriber(self, PointCloud2, '/robot/lidar')
        det_sub = message_filters.Subscriber(self, Detection2DArray, '/robot/detections')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, det_sub], queue_size=5, slop=self.sync_slop)
        self.sync.registerCallback(self._on_sync)

        self.pub_target = self.create_publisher(PoseStamped, '/robot/target_pose', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/robot/fusion_cloud', 10)
        if self.debug_img:
            self.pub_debug = self.create_publisher(Image, '/robot/fusion_debug_img', 10)

        self.get_logger().info(f'Fusion ready (slop={self.sync_slop}s, debug={self.debug_img})')

    def _ci_cb(self, msg):
        if not self.cam_ok:
            self.cam.fromCameraInfo(msg)
            self.cam_ok = True
            self.get_logger().info(f'Camera: {msg.width}x{msg.height} fx={self.cam.fx():.1f}')

    def _img_cb(self, msg):
        self.latest_img = msg

    def _on_sync(self, cloud_msg, det_msg):
        self.call_cnt += 1
        if not self.cam_ok or not det_msg.detections:
            return

        # 1. 提取 XYZ (lidar frame)
        pts = []
        for pt in point_cloud2.read_points(cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True):
            pts.append([pt[0], pt[1], pt[2]])
        if not pts: return
        pts = np.array(pts, dtype=np.float64)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) == 0: return

        lidar_fr = cloud_msg.header.frame_id
        cam_fr = self.cam.tfFrame()
        stamp = rclpy.time.Time.from_msg(cloud_msg.header.stamp)

        # 2. TF: lidar → camera optical
        try:
            tfm = self.tf_buf.lookup_transform(cam_fr, lidar_fr, stamp, Duration(seconds=0.2))
        except Exception:
            return
        pts_cam = transform_pts(pts, tf_to_matrix(tfm))
        pts_cam = pts_cam[np.all(np.isfinite(pts_cam), axis=1)]

        # 3. 向量化投影: u = fx*X/Z+cx, v = fy*Y/Z+cy
        fx, fy, cx, cy = self.cam.fx(), self.cam.fy(), self.cam.cx(), self.cam.cy()
        X, Y, Z = pts_cam[:, 0], pts_cam[:, 1], pts_cam[:, 2]
        z_ok = Z > 0.05
        pts_cam = pts_cam[z_ok]; X, Y, Z = X[z_ok], Y[z_ok], Z[z_ok]
        if len(pts_cam) == 0: return

        # 降采样到 2000 点
        if len(pts_cam) > 2000:
            ix = np.random.choice(len(pts_cam), 2000, replace=False)
            pts_cam = pts_cam[ix]; X, Y, Z = X[ix], Y[ix], Z[ix]

        u = fx * X / Z + cx
        v = fy * Y / Z + cy
        in_img = (u >= 0) & (u < self.cam.width) & (v >= 0) & (v < self.cam.height)
        if not np.any(in_img): return

        # 4. 对每个 detection: bbox 过滤 + 中值 3D
        all_inlier = []
        targets = []
        for d in det_msg.detections:
            bbox = d.bbox
            cid = d.results[0].hypothesis.class_id if d.results else '?'
            sc = d.results[0].hypothesis.score if d.results else 0.0
            bx, by = bbox.center.position.x, bbox.center.position.y
            bw, bh = bbox.size_x, bbox.size_y
            x1, y1 = bx - bw/2.0, by - bh/2.0
            x2, y2 = bx + bw/2.0, by + bh/2.0

            in_bbox = in_img & (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
            inliers = pts_cam[in_bbox]
            if len(inliers) < self.min_points: continue

            tgt_cam = np.median(inliers, axis=0)
            # TF → odom
            try:
                ps = PointStamped()
                ps.header.frame_id = cam_fr
                ps.point = Point(x=float(tgt_cam[0]), y=float(tgt_cam[1]), z=float(tgt_cam[2]))
                todom = self.tf_buf.lookup_transform(self.output_frame, cam_fr, stamp,
                                                     Duration(seconds=0.2))
                ps_out = do_transform_point(ps, todom)
                tw = np.array([ps_out.point.x, ps_out.point.y, ps_out.point.z])
            except Exception:
                continue

            targets.append((cid, sc, tw))
            all_inlier.append(inliers)

            if self.call_cnt % 10 == 1:
                self.get_logger().info(
                    f'Target "{cid}" ({tw[0]:.2f}, {tw[1]:.2f}, {tw[2]:.2f}) [{len(inliers)}pts]')

        # 5. Publish
        if targets:
            msg = PoseStamped(header=Header(stamp=cloud_msg.header.stamp,
                               frame_id=self.output_frame),
                               pose=Pose(position=Point(x=float(targets[0][2][0]),
                                       y=float(targets[0][2][1]),
                                       z=float(targets[0][2][2])),
                                        orientation=Quaternion(w=1.0)))
            self.pub_target.publish(msg)

        if all_inlier and self.pub_cloud.get_subscription_count() > 0:
            all_pts = np.vstack(all_inlier)
            hdr = Header(stamp=cloud_msg.header.stamp, frame_id=cam_fr)
            self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(
                hdr, [(float(p[0]), float(p[1]), float(p[2])) for p in all_pts]))

        if self.debug_img and self.latest_img:
            try:
                img = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
                for ui, vi in zip(u[in_img][:500], v[in_img][:500]):
                    ui, vi = int(ui), int(vi)
                    if 0 <= ui < self.cam.width and 0 <= vi < self.cam.height:
                        cv2.circle(img, (ui, vi), 1, (0, 255, 0), -1)
                for d in det_msg.detections:
                    bbox = d.bbox
                    x1 = int(bbox.center.position.x - bbox.size_x/2)
                    y1 = int(bbox.center.position.y - bbox.size_y/2)
                    x2 = int(bbox.center.position.x + bbox.size_x/2)
                    y2 = int(bbox.center.position.y + bbox.size_y/2)
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                dbg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                dbg.header.stamp = cloud_msg.header.stamp
                dbg.header.frame_id = cam_fr
                self.pub_debug.publish(dbg)
            except Exception:
                pass


def main():
    rclpy.init()
    n = LidarCameraFusion()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
