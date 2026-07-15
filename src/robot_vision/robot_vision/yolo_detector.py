#!/usr/bin/env python3
"""YOLOv8 object detector node — /robot/camera → YOLOv8 → detections + annotated image

Uses COCO pretrained weights. Target classes: ball (sports ball), chair, person, etc.
Publishes:
  - /robot/detections     (vision_msgs/Detection2DArray)
  - /robot/detection_img  (sensor_msgs/Image) annotated with bboxes
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

# Target class IDs for COCO (0-indexed)
TARGET_CLASSES = {
    32: 'sports ball',   # 0-indexed: 32
    56: 'chair',          # 0-indexed: 56
    0: 'person',          # for debugging
}
# COCO class names (80 classes) for reference
COCO_CLASSES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat',
    'traffic light','fire hydrant','stop sign','parking meter','bench','bird',
    'cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe',
    'backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard',
    'sports ball','kite','baseball bat','baseball glove','skateboard','surfboard',
    'tennis racket','bottle','wine glass','cup','fork','knife','spoon','bowl',
    'banana','apple','sandwich','orange','broccoli','carrot','hot dog','pizza',
    'donut','cake','chair','couch','potted plant','bed','dining table','toilet',
    'tv','laptop','mouse','remote','keyboard','cell phone','microwave','oven',
    'toaster','sink','refrigerator','book','clock','vase','scissors',
    'teddy bear','hair drier','toothbrush',
]


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Load YOLOv8 nano (fastest, good enough for sim)
        model_path = self.declare_parameter('model', 'yolov8n.pt').value
        self.get_logger().info(f'Loading YOLOv8 model: {model_path}...')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLOv8 model loaded OK')

        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(Image, '/robot/camera', self.image_callback, 10)

        # Publishers
        self.det_pub = self.create_publisher(Detection2DArray, '/robot/detections', 10)
        self.vis_pub = self.create_publisher(Image, '/robot/detection_img', 10)

        # Throttle: run inference every N frames
        self.frame_count = 0
        self.inference_every = self.declare_parameter('inference_every', 3).value

        self.get_logger().info(f'YOLO detector ready. Target classes: ')
        for cid, cname in TARGET_CLASSES.items():
            self.get_logger().info(f'  - {cname} (class_id={cid})')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.inference_every != 0:
            return  # skip frames to save CPU

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # YOLO inference
        results = self.model(cv_image, verbose=False)
        detections = results[0]

        # Build vision_msgs
        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = detections.plot()  # BGR numpy array

        for box in detections.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())

            if cls_id not in TARGET_CLASSES:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1

            det2d = Detection2D()
            det2d.header = msg.header

            # Results
            obj = ObjectHypothesisWithPose()
            obj.hypothesis.class_id = str(cls_id)
            obj.hypothesis.score = conf
            det2d.results.append(obj)

            # Bbox
            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.x = cx
            bbox.center.y = cy
            bbox.size_x = w
            bbox.size_y = h
            det2d.bbox = bbox

            det_array.detections.append(det2d)
            self.get_logger().info(
                f'Found {TARGET_CLASSES[cls_id]} score={conf:.2f} at ({cx:.0f},{cy:.0f})',
                throttle_duration_sec=2.0,
            )

        if det_array.detections:
            self.det_pub.publish(det_array)

        # Publish annotated image (BGR → RGB for RViz)
        vis_msg = self.bridge.cv2_to_imgmsg(
            cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB), 'rgb8')
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
