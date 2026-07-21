#!/bin/bash
# ============================================================
# Ball Robot 仿真 — 3 窗口一键启动
# ============================================================
WS="/home/c/fastlio_ws"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null || true

echo "清理旧进程..."
# 用 pgrep + kill -9 确保100%杀死
for proc in "gz sim" "gz server" "gz gui" "parameter_bridge" \
  "fastlio_mapping" "cloud_to_scan" "async_slam_toolbox_node" \
  "yolo_detector" "lidar_camera_fusion" "camera_info_publisher" \
  "auto_explorer" "rviz2" "multi_image_view" "drive_control" \
  "pointcloud_to_laserscan" "explore_coordinator" "smart_navigator" \
  "simple_navigator"; do
  pids=$(pgrep -f "$proc" 2>/dev/null) || true
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null || true
done
sleep 3

# 验证干净
LEFT=$(ps aux | grep -cE 'gz sim|fastlio|cloud_to_scan|slam_toolbox|yolo_detect|fusion|auto_exp|rviz2|ros_gz_bridge|multi_img|drive_con|camera_info_pub' 2>/dev/null || echo 0)
if [ "$LEFT" -gt 2 ]; then
  echo "⚠ 仍有${LEFT}个残留进程, 二次清理..."
  kill -9 $(ps aux | grep -E 'gz sim|fastlio|cloud_to_scan|slam_toolbox|yolo|fusion|auto_explorer|rviz2|ros_gz_bridge' | grep -v grep | awk '{print $2}') 2>/dev/null || true
  sleep 2
fi

# 清理 DDS 共享内存
rm -rf /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/*port* 2>/dev/null || true
unset FASTRTPS_DEFAULT_PROFILES_FILE

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
