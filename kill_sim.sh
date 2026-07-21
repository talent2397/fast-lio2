#!/bin/bash
# 一键清理所有仿真进程 (独立使用, 不在start_sim.sh内)
set -e

echo "=== 清理所有仿真进程 ==="
COUNT=0

for proc in "gz sim" "gz server" "gz gui" "parameter_bridge" \
  "fastlio_mapping" "cloud_to_scan" "async_slam_toolbox_node" \
  "yolo_detector" "lidar_camera_fusion" "camera_info_publisher" \
  "auto_explorer" "rviz2" "multi_image_view" "drive_control" \
  "pointcloud_to_laserscan" "ros2 launch\|robot_bringup" \
  "ros2/launch"; do
  pids=$(pgrep -f "$proc" 2>/dev/null) || true
  if [ -n "$pids" ]; then
    kill -9 $pids 2>/dev/null || true
    n=$(echo "$pids" | wc -w)
    echo "  killed $proc ($n个)"
    COUNT=$((COUNT + n))
  fi
done

# 杀gnome-terminal窗口
pkill -f "gnome-terminal.*fastlio\|gnome-terminal.*Vision\|gnome-terminal.*WASD" 2>/dev/null || true

# 清DDS
rm -rf /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/*port* 2>/dev/null || true

echo "=== 完成: 清理${COUNT}个进程 ==="
echo "内存: $(free -h | awk '/Mem:/{print $4"可用"}')"
