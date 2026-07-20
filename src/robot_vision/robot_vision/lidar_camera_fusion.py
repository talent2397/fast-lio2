#!/usr/bin/env python3
"""
LiDAR-Camera 3D fusion node — projects LiDAR points onto camera plane for 3D target localization.

Pipeline:
  1. Sync /robot/lidar (PointCloud2) + /robot/detections (Detection2DArray)
  2. TF2: transform LiDAR points from lidar_frame → camera_frame
  3. Project 3D points to 2D image plane using /robot/camera_info intrinsics
  4. Filter points inside each detection bbox
  5. Compute median 3D position → publish as /robot/target_pose

Subscriptions:
  /robot/lidar        (PointCloud2)
  /robot/detections   (Detection2DArray)
  /robot/camera_info  (CameraInfo)        — latest stored
  /robot/camera       (Image)             — latest stored for debug viz

Publications:
  /robot/target_pose      (PoseStamped)      — 3D world coordinate of detected target
  /robot/fusion_debug_img (Image)            — LiDAR projections overlaid on camera image
  /robot/fusion_cloud     (PointCloud2)      — inlier points within bbox (for RViz)
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PointStamped
from std_msgs.msg import Header

import tf2_ros
from tf2_geometry_msgs import do_transform_point

import message_filters


def tf_to_matrix(transform) -> np.ndarray:
    """Convert a geometry_msgs/TransformStamped to a 4x4 homogeneous matrix."""
    t = transform.transform.translation
    r = transform.transform.rotation
    # Quaternion to rotation matrix (xyzw → matrix, pure numpy, no scipy)
    x, y, z, w = r.x, r.y, r.z, r.w
    # Handle degenerate quaternion (all zeros → identity)
    norm2 = x*x + y*y + z*z + w*w
    if norm2 < 1e-12:
        rot = np.eye(3)
    else:
        s = 2.0 / norm2
        rot = np.array([
            [1 - s*(y*y + z*z),     s*(x*y - w*z),     s*(x*z + w*y)],
            [    s*(x*y + w*z), 1 - s*(x*x + z*z),     s*(y*z - w*x)],
            [    s*(x*z - w*y),     s*(y*z + w*x), 1 - s*(x*x + y*y)],
        ])
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to (N,3) points. Returns (N,3)."""
    pts_h = np.hstack([points, np.ones((len(points), 1))])
    pts_tf = (matrix @ pts_h.T).T
    return pts_tf[:, :3]


class LidarCameraFusion(Node):
    """3D localization via LiDAR point cloud projection onto camera plane."""

    def __init__(self):
        super().__init__('lidar_camera_fusion')

        # --- Parameters ---
        self.min_points = self.declare_parameter('min_points_in_bbox', 5).value
        self.sync_slop = self.declare_parameter('sync_slop', 0.1).value
        self.output_frame = self.declare_parameter('output_frame', 'robot/odom').value
        self.enable_debug_img = self.declare_parameter('enable_debug_img', True).value

        # --- State ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_model_ready = False
        self.latest_camera_img = None

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscriptions ---
        self.create_subscription(CameraInfo, '/robot/camera_info', self._camera_info_cb, 10)
        self.create_subscription(Image, '/robot/camera', self._camera_cb, 10)

        lidar_sub = message_filters.Subscriber(self, PointCloud2, '/robot/lidar')
        det_sub = message_filters.Subscriber(self, Detection2DArray, '/robot/detections')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, det_sub], queue_size=10, slop=self.sync_slop,
        )
        self.sync.registerCallback(self._sync_callback)

        # --- Publishers ---
        self.target_pub = self.create_publisher(PoseStamped, '/robot/target_pose', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/robot/fusion_cloud', 10)
        if self.enable_debug_img:
            self.debug_pub = self.create_publisher(Image, '/robot/fusion_debug_img', 10)

        self.get_logger().info(
            f'LiDAR-Camera fusion ready (sync_slop={self.sync_slop}s, '
            f'min_points={self.min_points}, output_frame={self.output_frame})'
        )

    # -----------------------------------------------------------------
    #  Callbacks
    # -----------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        if not self.camera_model_ready:
            self.camera_model.fromCameraInfo(msg)
            self.camera_model_ready = True
            self.get_logger().info(
                f'Camera model: {msg.width}x{msg.height} '
                f'fx={self.camera_model.fx():.1f} fy={self.camera_model.fy():.1f} '
                f'frame={self.camera_model.tfFrame()}'
            )

    def _camera_cb(self, msg: Image):
        self.latest_camera_img = msg

    # -----------------------------------------------------------------
    #  Main sync
    # -----------------------------------------------------------------

    def _sync_callback(self, cloud_msg: PointCloud2, det_msg: Detection2DArray):
        if not hasattr(self, '_sync_cnt'):
            self._sync_cnt = 0
        self._sync_cnt += 1
        if self._sync_cnt % 30 == 1:
            self.get_logger().info(f'Sync #{self._sync_cnt}: lidar={cloud_msg.header.stamp.sec}.{cloud_msg.header.stamp.nanosec//1000000:03d} det={det_msg.header.stamp.sec}.{det_msg.header.stamp.nanosec//1000000:03d} ndet={len(det_msg.detections)}')

        if not self.camera_model_ready:
            self.get_logger().warn('Camera model not ready', throttle_duration_sec=5.0)
            return
        if not det_msg.detections:
            return

        # --- 1. Extract XYZ from LiDAR (original frame) ---
        points_lidar = self._extract_xyz(cloud_msg)
        if points_lidar is None:
            return
        # Filter NaN/Inf before transform
        finite = np.all(np.isfinite(points_lidar), axis=1)
        points_lidar = points_lidar[finite]
        if len(points_lidar) == 0:
            return

        lidar_frame = cloud_msg.header.frame_id
        camera_frame = self.camera_model.tfFrame()

        # --- 2. TF lookup: lidar → camera optical frame ---
        tf_stamp = rclpy.time.Time.from_msg(cloud_msg.header.stamp)
        actual_cam_frame = camera_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame, lidar_frame, tf_stamp,
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF {lidar_frame}→{camera_frame}: {type(e).__name__}',
                throttle_duration_sec=5.0,
            )
            return

        # --- 3. Manual transform (avoid do_transform_cloud field mismatch) ---
        t = transform.transform.translation
        r = transform.transform.rotation
        if self._sync_cnt % 30 == 1:
            self.get_logger().info(
                f'TF matrix: t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) q=({r.x:.3f},{r.y:.3f},{r.z:.3f},{r.w:.3f})'
            )
        tf_mat = tf_to_matrix(transform)
        if np.any(~np.isfinite(tf_mat)):
            self.get_logger().error(f'TF matrix has NaN/Inf! t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) q=({r.x:.3f},{r.y:.3f},{r.z:.3f},{r.w:.3f})')
            return
        points_cam = transform_points(points_lidar, tf_mat)
        # Filter NaN (some LiDAR points may have infinite range values)
        n_total = len(points_cam)
        finite = np.all(np.isfinite(points_cam), axis=1)
        n_finite = np.count_nonzero(finite)
        points_cam = points_cam[finite]
        if self._sync_cnt % 30 == 1:
            self.get_logger().info(f'Points: {n_total} total → {n_finite} finite after transform')
        if len(points_cam) == 0:
            return

        # --- 4. Project 3D → 2D ---
        valid_mask, pixels = self._project_points(points_cam)
        n_valid = np.count_nonzero(valid_mask)
        if self._sync_cnt % 10 == 1:
            self.get_logger().info(f'  Proj: {n_valid}/{len(points_cam)} pts in image')
        if n_valid == 0:
            return

        points_cam_valid = points_cam[valid_mask]
        pixels_valid = pixels[valid_mask]

        # --- 5. Per-detection: bbox filter + median 3D ---
        all_inlier_pts = []
        target_poses = []
        n_proj = np.count_nonzero(valid_mask)

        for detection in det_msg.detections:
            bbox = detection.bbox
            class_id = detection.results[0].hypothesis.class_id if detection.results else 'unknown'
            score = detection.results[0].hypothesis.score if detection.results else 0.0

            cx, cy = bbox.center.position.x, bbox.center.position.y
            bw, bh = bbox.size_x, bbox.size_y
            x1, y1 = cx - bw / 2.0, cy - bh / 2.0
            x2, y2 = cx + bw / 2.0, cy + bh / 2.0

            mask = (pixels_valid[:, 0] >= x1) & (pixels_valid[:, 0] <= x2) & \
                   (pixels_valid[:, 1] >= y1) & (pixels_valid[:, 1] <= y2)
            mask &= points_cam_valid[:, 2] > 0.1  # in front of camera

            inliers = points_cam_valid[mask]
            if self._sync_cnt % 30 == 1:
                self.get_logger().info(
                    f'  Det "{class_id}": bbox=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}) '
                    f'inliers={len(inliers)} (need {self.min_points})'
                )
            if len(inliers) < self.min_points:
                continue

            # Median (robust to outliers)
            target_cam = np.median(inliers, axis=0)

            # Transform camera→odom
            target_world = self._tf_point(target_cam, actual_cam_frame, self.output_frame, cloud_msg.header.stamp)
            if target_world is None:
                continue

            target_poses.append((class_id, score, target_world))
            all_inlier_pts.append(inliers)

            self.get_logger().info(
                f'Target "{class_id}" 3D: ({target_world[0]:.2f}, {target_world[1]:.2f}, {target_world[2]:.2f}) '
                f'[{len(inliers)} pts, score={score:.2f}]'
            )

        # --- 6. Publish ---
        if target_poses:
            self._pub_pose(target_poses[0], cloud_msg.header.stamp)
        if all_inlier_pts:
            self._pub_cloud(np.vstack(all_inlier_pts), actual_cam_frame, cloud_msg.header.stamp)
        if self.enable_debug_img:
            self._pub_debug(pixels_valid, det_msg, cloud_msg.header.stamp)

    # -----------------------------------------------------------------
    #  Helpers
    # -----------------------------------------------------------------

    def _extract_xyz(self, cloud_msg: PointCloud2) -> np.ndarray | None:
        pts = []
        for pt in point_cloud2.read_points(cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True):
            pts.append([pt[0], pt[1], pt[2]])
        if not pts:
            return None
        return np.array(pts, dtype=np.float64)

    def _project_points(self, pts_3d: np.ndarray):
        """Project (N,3) camera-frame points to (N,2) pixel coords. Returns (mask, pixels)."""
        n = len(pts_3d)
        pixels = np.zeros((n, 2), dtype=np.float32)
        for i in range(n):
            uv = self.camera_model.project3dToPixel(pts_3d[i])
            pixels[i] = uv
        valid = (pixels[:, 0] >= 0) & (pixels[:, 0] < self.camera_model.width) & \
                (pixels[:, 1] >= 0) & (pixels[:, 1] < self.camera_model.height)
        return valid, pixels

    def _tf_point(self, pt_cam, from_frame, to_frame, stamp) -> np.ndarray | None:
        """Transform a single point between frames via TF2."""
        try:
            ps = PointStamped()
            ps.header.frame_id = from_frame
            ps.point = Point(x=float(pt_cam[0]), y=float(pt_cam[1]), z=float(pt_cam[2]))
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
            out = do_transform_point(ps, t)
            return np.array([out.point.x, out.point.y, out.point.z])
        except Exception as e:
            self.get_logger().warn(
                f'Point TF {from_frame}→{to_frame}: {type(e).__name__}',
                throttle_duration_sec=5.0,
            )
            return None

    def _pub_pose(self, target, stamp):
        class_id, score, xyz = target
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame
        msg.pose = Pose(
            position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.target_pub.publish(msg)

    def _pub_cloud(self, pts: np.ndarray, frame_id, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header,
            [(float(p[0]), float(p[1]), float(p[2])) for p in pts],
        )
        self.cloud_pub.publish(cloud_msg)

    def _pub_debug(self, pixels, det_msg, stamp):
        if self.latest_camera_img is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(self.latest_camera_img, 'bgr8')
        except Exception:
            return
        h, w = img.shape[:2]

        # Green dots = LiDAR projections
        for u, v in pixels:
            ui, vi = int(u), int(v)
            if 0 <= ui < w and 0 <= vi < h:
                cv2.circle(img, (ui, vi), 1, (0, 255, 0), -1)

        # Bboxes
        for detection in det_msg.detections:
            bbox = detection.bbox
            class_id = detection.results[0].hypothesis.class_id if detection.results else '?'
            cx, cy = bbox.center.position.x, bbox.center.position.y
            bw, bh = bbox.size_x, bbox.size_y
            x1 = int(cx - bw / 2.0)
            y1 = int(cy - bh / 2.0)
            x2 = int(cx + bw / 2.0)
            y2 = int(cy + bh / 2.0)
            color = (255, 0, 0) if 'red' in class_id.lower() else (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, class_id, (x1, max(y1 - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.putText(img, f'LiDAR pts: {len(pixels)}', (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        debug_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        debug_msg.header.stamp = stamp
        debug_msg.header.frame_id = self.camera_model.tfFrame()
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
