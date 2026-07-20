#!/bin/bash
# ============================================================
# Ball Robot 仿真 — 2 窗口一键启动
# 用法: bash ~/fastlio_ws/start_sim.sh
# ============================================================
set -e

WS="/home/c/fastlio_ws"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null || true

echo "清理旧进程..."
pkill -f "gz sim|ros_gz_bridge|fastlio_mapping|rviz2|static_transform|drive_control|camera_info_publisher|yolo_detector|lidar_camera_fusion|multi_image_view" 2>/dev/null || true
sleep 2

# ================================================================
# 窗口 1: 全栈仿真 + RViz 图像 (Gazebo + Bridge + TF + Vision + SLAM + Mapping + RViz)
# ================================================================
gnome-terminal --title="Main Stack" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null
echo '========================================'
echo '  Ball Robot — 全栈仿真启动'
echo '  Gazebo + Bridge + TF + YOLO + Fusion + FAST-LIO + Mapping + RViz'
echo '========================================'
ros2 launch robot_bringup sim.launch.py
" &

sleep 1

# ================================================================
# 窗口 2: WASD 键盘控制
# ================================================================
gnome-terminal --title="WASD Drive" -- bash -c "
source /opt/ros/jazzy/setup.bash 2>/dev/null
source $WS/install/setup.bash 2>/dev/null
echo '========================================'
echo '  WASD 键盘控制'
echo '  W = 前进   S = 后退'
echo '  A = 右转    D = 左转'
echo '  Q = 退出'
echo '========================================'
sleep 8
python3 $WS/scripts/drive_control.py
" &

echo ""
echo "========================================"
echo "  2 窗口启动中，请等待 ~15s 加载"
echo ""
echo "  窗口1: Main Stack  — Gazebo + Vision + SLAM + Mapping + RViz"
echo "          RViz 内含: 点云地图 + Map + 三路图像 (Camera|Detections|Fusion)"
echo "  窗口2: WASD Drive  — 键盘控制"
echo ""
echo "  统一话题:"
echo "    /robot/lidar       /robot/imu         /robot/camera"
echo "    /robot/cmd_vel     /robot/odom        /robot/camera_info"
echo "    /robot/detections  /robot/detection_img"
echo "    /robot/target_pose /robot/fusion_debug_img"
echo "    /map  /scan  /cloud_registered"
echo "========================================"
