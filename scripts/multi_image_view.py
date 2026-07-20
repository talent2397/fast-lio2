#!/usr/bin/env python3
"""
三合一图像查看器 — 将 Camera / Detections / Fusion 三路图像水平拼接显示。

订阅:
  /robot/camera          (原始图像)
  /robot/detection_img   (YOLO+HSV 检测框)
  /robot/fusion_debug_img (LiDAR 投影点 + bbox)

用法:
  python3 scripts/multi_image_view.py
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MultiImageViewer(Node):
    def __init__(self):
        super().__init__('multi_image_viewer')
        self.bridge = CvBridge()
        self.images = {
            'camera': None,
            'detection': None,
            'fusion': None,
        }

        self.create_subscription(Image, '/robot/camera', lambda m: self._cb(m, 'camera'), 10)
        self.create_subscription(Image, '/robot/detection_img', lambda m: self._cb(m, 'detection'), 10)
        self.create_subscription(Image, '/robot/fusion_debug_img', lambda m: self._cb(m, 'fusion'), 10)

        # Create OpenCV window ONCE
        self.win_name = 'Robot Vision — F=全屏  ESC=退出'
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_name, 1200, 480)

        self.timer = self.create_timer(0.05, self._render)
        self.get_logger().info('Multi-Image Viewer ready')
        self.get_logger().info('  F = 全屏切换  |  ESC = 退出')

    def _cb(self, msg: Image, key: str):
        try:
            self.images[key] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _render(self):
        H = 400
        panels = []

        for key, label in [
            ('camera', 'Camera (Raw)'),
            ('detection', 'Detections (YOLO+HSV)'),
            ('fusion', 'Fusion (LiDAR Projection)'),
        ]:
            img = self.images.get(key)
            if img is None:
                img = np.full((H, 320, 3), (40, 40, 40), dtype=np.uint8)
                cv2.putText(img, 'Waiting...', (60, H // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            else:
                h, w = img.shape[:2]
                new_w = int(w * H / h)
                img = cv2.resize(img, (new_w, H))

            bar_h = 28
            canvas = np.zeros((H + bar_h, img.shape[1], 3), dtype=np.uint8)
            canvas[bar_h:, :] = img
            cv2.putText(canvas, label, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            panels.append(canvas)

        stitched = np.hstack(panels)
        cv2.imshow(self.win_name, stitched)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('f') or key == ord('F'):
            is_fs = cv2.getWindowProperty(self.win_name, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(
                self.win_name, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if not is_fs else cv2.WINDOW_NORMAL,
            )

        if key == 27:
            self.get_logger().info('ESC — shutting down')
            cv2.destroyAllWindows()
            raise SystemExit


def main():
    rclpy.init()
    node = MultiImageViewer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
