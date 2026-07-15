#!/usr/bin/env python3
"""YOLOv8 object detector node — subscribes to /robot/camera, publishes detections."""
# Phase 2.2: 完整实现（仿真验证后补充）

import rclpy
from rclpy.node import Node


class YoloDetector(Node):
    """YOLOv8 real-time object detection."""

    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info('YOLO detector node started (skeleton - Phase 2.2)')
        # TODO Phase 2.2:
        # - Subscribe to /robot/camera (sensor_msgs/Image)
        # - Load YOLOv8 model (COCO pretrained)
        # - Run inference on each frame
        # - Publish Detection2DArray or custom Detection3DArray


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
