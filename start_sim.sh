#!/bin/bash
# ============================================================
# Ball Robot 全栈仿真 — 直接在当前终端启动所有节点
# 用法: bash ~/fastlio_ws/start_sim.sh
#
# 另开终端: bash ~/fastlio_ws/scripts/drive_control.sh
# ============================================================
set -e

WS="/home/c/fastlio_ws"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null || true

echo "清理旧进程..."
pkill -f "gz sim|ros_gz_bridge|fastlio_mapping|rviz2|static_transform|camera_info_publisher|yolo_detector|lidar_camera_fusion" 2>/dev/null || true
sleep 2

echo ""
echo "========================================"
echo "  Ball Robot — 全栈仿真启动"
echo "  Gazebo + Bridge + TF + YOLO + Fusion + FAST-LIO + Mapping + RViz"
echo "========================================"
echo ""

ros2 launch robot_bringup sim.launch.py
