#!/usr/bin/env python3
"""
CameraInfo publisher — broadcasts static camera intrinsics.

Since ros_gz_bridge does NOT bridge CameraInfo from Gazebo Harmonic,
this node publishes the calibration parameters from ROS parameters.

Topics:
  Pub: /robot/camera_info (CameraInfo) @ 30Hz

Parameters (with defaults matching ball_robot.sdf camera sensor):
  camera_width:  640
  camera_height: 480
  camera_fx:     554.38
  camera_fy:     554.38
  camera_cx:     320.0
  camera_cy:     240.0
  camera_frame:  camera_link
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    """Publish static CameraInfo at fixed rate."""

    def __init__(self):
        super().__init__('camera_info_publisher')

        # --- Read intrinsics from parameters ---
        self.width = self.declare_parameter('camera_width', 640).value
        self.height = self.declare_parameter('camera_height', 480).value
        self.fx = self.declare_parameter('camera_fx', 554.38).value
        self.fy = self.declare_parameter('camera_fy', 554.38).value
        self.cx = self.declare_parameter('camera_cx', 320.0).value
        self.cy = self.declare_parameter('camera_cy', 240.0).value
        self.frame_id = self.declare_parameter('camera_frame', 'robot/camera_optical').value
        self.rate = self.declare_parameter('publish_rate', 30.0).value

        # --- Publisher ---
        self.pub = self.create_publisher(CameraInfo, '/robot/camera_info', 10)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            f'CameraInfo publisher ready — {self.width}x{self.height} '
            f'fx={self.fx:.2f} fy={self.fy:.2f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f} '
            f'frame={self.frame_id} @ {self.rate}Hz'
        )

    def _publish(self):
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'

        # Intrinsic matrix K (row-major)
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]

        # Rectification matrix R (identity)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix P = K [I|0]
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
