#!/bin/bash
# ============================================================
# Ball Robot 仿真 — 3 窗口一键启动
# 用法: bash ~/fastlio_ws/start_sim.sh
# ============================================================
set -e

WS="/home/c/fastlio_ws"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null || true

echo "清理旧进程..."
pkill -f "gz sim|ros_gz_bridge|fastlio_mapping|rviz2|static_transform|multi_image_view|drive_control|camera_info_publisher|yolo_detector|lidar_camera_fusion" 2>/dev/null || true
sleep 2

# ================================================================
# 窗口 2: 三合一图像查看器
# ================================================================
gnome-terminal --title="Vision Viewer" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null
echo '等待全栈就绪...'
sleep 14
echo '三合一图像: Camera | Detections (YOLO+HSV) | Fusion (LiDAR)'
echo '按 F 全屏 | ESC 关闭'
python3 $WS/scripts/multi_image_view.py
" &

sleep 1

# ================================================================
# 窗口 3: WASD 键盘控制
# ================================================================
gnome-terminal --title="WASD Drive" -- bash -c "
source /opt/ros/jazzy/setup.bash 2>/dev/null
source $WS/install/setup.bash 2>/dev/null
echo '========================================'
echo '  WASD 键盘控制'
echo '  W=前进 S=后退 A=右转 D=左转 Q=退出'
echo '========================================'
sleep 8
python3 $WS/scripts/drive_control.py
" &

echo ""
echo "========================================"
echo "  启动中，请等待 ~15s 加载"
echo ""
echo "  主终端  : 全栈日志 (Gazebo+Vision+SLAM+Mapping+RViz)"
echo "  窗口 2  : 三合一图像 (F键全屏)"
echo "  窗口 3  : WASD 键盘控制"
echo "========================================"
echo ""

# ================================================================
# 窗口 1: 全栈仿真 (当前终端，阻塞运行)
# ================================================================
ros2 launch robot_bringup sim.launch.py
