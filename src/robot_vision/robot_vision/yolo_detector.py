#!/usr/bin/env python3
"""
YOLOv8 + HSV color detector — dual-mode target detection.

Mode 1 (YOLO):  ultralytics YOLOv8 with COCO weights, if torch is installed.
Mode 2 (HSV):   OpenCV HSV color segmentation for red ball, always works as fallback.

Topics:
  Sub: /robot/camera (Image)
  Pub: /robot/detections (Detection2DArray)
  Pub: /robot/detection_img (Image) — annotated image with bboxes+crosshairs
"""

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D, Pose2D

# Try loading YOLO; fall back to HSV-only if torch not installed
YOLO_ENABLED = False
YOLO = None
try:
    from ultralytics import YOLO as _YOLO
    YOLO_ENABLED = True
except ImportError:
    pass

# Target definitions
TARGET_CLASSES_YOLO = {32: 'sports ball', 56: 'chair', 0: 'person'}

# HSV ranges for RED ball (双阈值融合, 红色在 H 两端: 0-10 & 160-180)
# S_min=100 V_min=70: 过滤低饱和度/低亮度像素, 大幅减少误检
RED_LOWER_1 = np.array([0, 100, 70])
RED_UPPER_1 = np.array([10, 255, 255])
RED_LOWER_2 = np.array([160, 100, 70])
RED_UPPER_2 = np.array([180, 255, 255])
# 形状验证: 最小面积 300, 球体接近正方形 (宽高比 0.6-1.6)
MIN_BALL_AREA = 300
BALL_ASPECT_RATIO = (0.5, 2.0)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # --- YOLO (if available) ---
        if YOLO_ENABLED:
            model_path = self.declare_parameter('yolo_model', 'yolov8n.pt').value
            self.get_logger().info(f'Loading YOLOv8: {model_path}...')
            self.yolo = _YOLO(model_path)
            self.get_logger().info('YOLOv8 loaded')
        else:
            self.yolo = None
            self.get_logger().warn('YOLOv8 not available (torch missing). Using HSV red-ball detection only.')
            self.get_logger().warn('To enable YOLO: pip3 install --break-system-packages ultralytics')

        self.bridge = CvBridge()

        # Sub
        self.create_subscription(Image, '/robot/camera', self.image_callback, 10)

        # Pub
        self.det_pub = self.create_publisher(Detection2DArray, '/robot/detections', 10)
        self.vis_pub = self.create_publisher(Image, '/robot/detection_img', 10)

        # Throttle
        self.frame_count = 0
        self.inference_every = self.declare_parameter('inference_every', 2).value

        self.get_logger().info(f'Detector ready (YOLO={"ON" if YOLO_ENABLED else "OFF"}, HSV=always)')

    def _detect_red_ball(self, hsv_img):
        """HSV color segmentation for red ball. Returns list of (cx,cy,w,h) tuples."""
        mask1 = cv2.inRange(hsv_img, RED_LOWER_1, RED_UPPER_1)
        mask2 = cv2.inRange(hsv_img, RED_LOWER_2, RED_UPPER_2)
        mask = mask1 | mask2

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_BALL_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            # 形状验证: 过滤长条状噪声 (宽高比检查)
            aspect = w / max(h, 1)
            if aspect < BALL_ASPECT_RATIO[0] or aspect > BALL_ASPECT_RATIO[1]:
                continue
            # 外接矩形填充率检查 (>30% 才算有效)
            fill_ratio = area / (w * h)
            if fill_ratio < 0.3:
                continue
            cx = x + w / 2.0
            cy = y + h / 2.0
            results.append((cx, cy, float(w), float(h)))

        return results, mask

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.inference_every != 0:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        h, w = rgb.shape[:2]
        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = rgb.copy()

        # --- HSV RED BALL DETECTION (always run) ---
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        red_dets, red_mask = self._detect_red_ball(hsv)

        for (cx, cy, bw, bh) in red_dets:
            x1, y1 = int(cx - bw/2), int(cy - bh/2)
            x2, y2 = int(cx + bw/2), int(cy + bh/2)

            # Draw on annotated image
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated, 'RED BALL', (x1, max(y1-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.drawMarker(annotated, (int(cx), int(cy)), (0, 255, 255),
                           cv2.MARKER_CROSS, 20, 2)

            # Build detection msg
            det2d = Detection2D()
            det2d.header = msg.header
            obj = ObjectHypothesisWithPose()
            obj.hypothesis.class_id = 'red_ball'
            obj.hypothesis.score = 0.9   # HSV isn't probabilistic
            det2d.results.append(obj)
            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.size_x = bw
            bbox.size_y = bh
            det2d.bbox = bbox
            det_array.detections.append(det2d)

        # --- YOLO (if available) ---
        if self.yolo is not None:
            try:
                results = self.yolo(rgb, verbose=False)
                yolo_dets = results[0]
                yolo_ann = yolo_dets.plot()

                for box in yolo_dets.boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    if cls_id not in TARGET_CLASSES_YOLO:
                        continue
                    x1b, y1b, x2b, y2b = box.xyxy[0].tolist()
                    cxb = (x1b + x2b) / 2.0
                    cyb = (y1b + y2b) / 2.0
                    bw2 = x2b - x1b
                    bh2 = y2b - y1b

                    det2d = Detection2D()
                    det2d.header = msg.header
                    obj = ObjectHypothesisWithPose()
                    obj.hypothesis.class_id = TARGET_CLASSES_YOLO[cls_id]
                    obj.hypothesis.score = conf
                    det2d.results.append(obj)
                    bbox = BoundingBox2D()
                    bbox.center = Pose2D()
                    bbox.center.position.x = cxb
                    bbox.center.position.y = cyb
                    bbox.size_x = bw2
                    bbox.size_y = bh2
                    det2d.bbox = bbox
                    det_array.detections.append(det2d)

                # Overlay YOLO annotations on our image
                annotated = cv2.addWeighted(annotated, 0.6,
                                            cv2.cvtColor(yolo_ann, cv2.COLOR_BGR2RGB), 0.4, 0)
            except Exception as e:
                self.get_logger().warn(f'YOLO inference error: {e}', throttle_duration_sec=5.0)

        if det_array.detections:
            self.det_pub.publish(det_array)
            names = [d.results[0].hypothesis.class_id for d in det_array.detections]
            self.get_logger().info(f'Detected: {names}', throttle_duration_sec=2.0)

        vis_msg = self.bridge.cv2_to_imgmsg(annotated, 'rgb8')
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
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
