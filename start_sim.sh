#!/bin/bash
# ============================================================
# Ball Robot 仿真 — 3 窗口一键启动
# ============================================================
WS="/home/c/fastlio_ws"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null || true

echo "清理旧进程..."
pkill -f "gz sim|ros_gz_bridge|fastlio_mapping|multi_image_view|drive_control|camera_info_publisher|yolo_detector|lidar_camera_fusion|smart_navigator|simple_navigator|explore_coordinator|auto_explorer|pointcloud_to_laserscan|slam_toolbox" 2>/dev/null || true
sleep 2

# ── 窗口 2: 三合一图像 ──
gnome-terminal --title="Vision" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null
sleep 14
echo '三合一图像 | F=全屏 ESC=退出'
python3 $WS/scripts/multi_image_view.py
" &
disown

sleep 1

# ── 窗口 3: WASD 键盘 ──
gnome-terminal --title="WASD" -- bash -c "
sleep 8
echo 'WASD | W=前 S=后 A=右 D=左 Q=退出'
python3 $WS/scripts/drive_control.py
" &
disown

sleep 1

echo "========================================"
echo "  窗口 1: 全栈  |  窗口 2: 图像  |  窗口 3: 键盘"
echo "  Ctrl+C 关闭主栈"
echo "========================================"

ros2 launch robot_bringup sim.launch.py
