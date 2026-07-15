#!/usr/bin/env python3
"""LiDAR-Camera fusion node — projects LiDAR points onto camera image for 3D target localization."""
# Phase 2.5: 完整实现（YOLO检测结果 + 同步LiDAR点云 → 3D世界坐标）

import rclpy
from rclpy.node import Node


class LidarCameraFusion(Node):
    """3D localization via LiDAR point cloud projection onto camera plane."""

    def __init__(self):
        super().__init__('lidar_camera_fusion')
        self.get_logger().info('LiDAR-Camera fusion node started (skeleton - Phase 2.5)')
        # TODO Phase 2.5:
        # - Subscribe to /robot/lidar (PointCloud2) and YOLO detections
        # - Synchronize LiDAR scan with camera frame (approx time sync)
        # - Project LiDAR points to image plane (camera_info + extrinsics)
        # - For each YOLO bbox, compute mean 3D position of projected points
        # - Publish target 3D coordinate (geometry_msgs/PoseStamped)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
