#!/bin/bash
# ============================================================
# Ball Robot 仿真 — 一键启动 (6个 gnome-terminal)
# 用法: bash ~/fastlio_ws/start_sim.sh
# ============================================================
set -e

WS="/home/c/fastlio_ws"
WORLD="$WS/src/robot_bringup/worlds/ball_robot.sdf"
BRIDGE_CFG="$WS/src/robot_bringup/config/gz_bridge.yaml"
LIO_CFG="$WS/src/robot_bringup/config/fastlio_params.yaml"
RViz_CFG="$WS/src/FAST_LIO_ROS2/rviz/fastlio.rviz"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null || true

echo "清理旧进程..."
pkill -f "gz sim|ros_gz_bridge|fastlio_mapping|rviz2|drive.sh|static_transform_publisher" 2>/dev/null || true
sleep 2

# 窗口1: Gazebo 仿真
gnome-terminal --title="1-Gazebo" -- bash -c "
source /opt/ros/jazzy/setup.bash
echo '========================================'
echo '  Gazebo 工厂场景 + 四轮差速小车'
echo '========================================'
gz sim -r $WORLD
" &

sleep 1

# 窗口2: ros_gz_bridge
gnome-terminal --title="2-Bridge" -- bash -c "
source /opt/ros/jazzy/setup.bash
echo '等待 Gazebo 就绪...'
sleep 6
echo 'ros_gz_bridge: /robot/lidar /robot/imu /robot/camera /robot/cmd_vel /robot/odom /tf'
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$BRIDGE_CFG
" &

sleep 1

# 窗口3: 静态 TF
gnome-terminal --title="3-TF" -- bash -c "
source /opt/ros/jazzy/setup.bash
sleep 4
echo '静态 TF: camera_init -> robot/odom'
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_init robot/odom
" &

sleep 1

# 窗口3.5: YOLO 检测
gnome-terminal --title="3.5-YOLO" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash 2>/dev/null
echo '等待 Camera 就绪...'
sleep 8
echo 'YOLOv8 目标检测 (COCO预训练, sports ball + chair)'
ros2 run robot_vision yolo_detector --ros-args -p model:=yolov8n.pt -p inference_every:=3
" &

sleep 1

# 窗口4: FAST-LIO2
gnome-terminal --title="4-FAST-LIO2" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
echo '等待 Bridge 就绪...'
sleep 10
echo 'FAST-LIO2 实时建图 (use_sim_time:=true)'
ros2 run fast_lio fastlio_mapping --ros-args -p use_sim_time:=true --params-file $LIO_CFG
" &

sleep 1

# 窗口5: RViz
gnome-terminal --title="5-RViz" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
echo '等待 FAST-LIO2 就绪...'
sleep 14
echo 'RViz 启动 (点云地图 + Path + Odometry + Camera)'
rviz2 -d $RViz_CFG
" &

sleep 1

# 窗口6: WASD 键盘控制
gnome-terminal --title="6-Drive(WASD)" -- bash -c "
source /opt/ros/jazzy/setup.bash 2>/dev/null
source $WS/install/setup.bash 2>/dev/null
echo '========================================'
echo '  WASD 键盘控制'
echo '  W = 前进 (3.0 m/s)   S = 后退'
echo '  A = 右转 (2.0 rad/s)  D = 左转'
echo '  Q = 退出'
echo '========================================'
sleep 8
python3 $WS/scripts/drive_control.py
" &

echo ""
echo "========================================"
echo "  6 个窗口启动中，请等待 ~15s 加载"
echo ""
echo "  窗口1: Gazebo       - 30m×30m 工厂 + 障碍物"
echo "  窗口2: Bridge       - 传感器桥接 (含 /clock)"
echo "  窗口3: TF           - camera_init → robot/odom"
echo "  窗口4: FAST-LIO2    - 激光惯性实时建图"
echo "  窗口5: RViz         - 点云地图 + 轨迹 + 相机"
echo "  窗口6: Drive        - WASD 键盘控制"
echo ""
echo "  统一话题:"
echo "    /robot/lidar    /robot/imu    /robot/camera"
echo "    /robot/cmd_vel  /robot/odom"
echo "  RViz Fixed Frame -> camera_init"
echo "  TF: camera_init → robot/odom → robot/base_link"
echo "========================================"
