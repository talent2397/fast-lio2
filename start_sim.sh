#!/bin/bash
# ============================================================
# FAST-LIO2 工厂场景仿真 — 一键启动 (5个 gnome-terminal)
# 用法: bash ~/fastlio_ws/start_sim.sh
# ============================================================
set -e

WS="/home/c/fastlio_ws"
WORLD="$WS/src/fastlio_sim/worlds/ball_robot.sdf"
BRIDGE_CFG="$WS/src/fastlio_sim/config/gz_bridge.yaml"
LIO_CFG="$WS/src/fastlio_sim/config/velodyne.yaml"
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
echo 'ros_gz_bridge: /clock + /lidar0 + /imu0 + /cmd_vel'
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$BRIDGE_CFG
" &

sleep 1

# 窗口2.5 静态 TF (camera_init→diff_car/odom→body 连接)
gnome-terminal --title="2.5-TF" -- bash -c "
source /opt/ros/jazzy/setup.bash
sleep 4
echo '静态 TF: camera_init -> diff_car/odom -> body'
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_init diff_car/odom &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 diff_car/chassis body &
wait
" &

sleep 1

# 窗口3: FAST-LIO2
gnome-terminal --title="3-FAST-LIO2" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
echo '等待 Bridge 就绪...'
sleep 10
echo 'FAST-LIO2 实时建图 (use_sim_time:=true)'
ros2 run fast_lio fastlio_mapping --ros-args -p use_sim_time:=true --params-file $LIO_CFG
" &

sleep 1

# 窗口4: RViz
gnome-terminal --title="4-RViz" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
echo '等待 FAST-LIO2 就绪...'
sleep 14
echo 'RViz 启动 (预配 Laser_map + Path + Odometry)'
rviz2 -d $RViz_CFG
" &

sleep 1

# 窗口5: 固定速度键盘控制 (WASD)
gnome-terminal --title="5-Drive(WASD)" -- bash -c "
source /opt/ros/jazzy/setup.bash 2>/dev/null
source $WS/install/setup.bash 2>/dev/null
echo '========================================'
echo '  固定速度键盘控制'
echo '  W = 前进 (1.0 m/s)   S = 后退'
echo '  A = 左转 (0.8 rad/s)  D = 右转'
echo '  Q = 退出'
echo '========================================'
sleep 8
python3 $WS/scripts/drive_control.py
" &

echo ""
echo "========================================"
echo "  5个窗口启动中，请等待 ~15s 加载"
echo ""
echo "  窗口1: Gazebo  - 30m×30m 工厂+走廊+障碍物"
echo "  窗口2: Bridge  - 传感器桥接 (含 /clock)"
echo "  窗口3: FAST    - LIO2 实时激光-惯性建图"
echo "  窗口4: RViz    - 点云地图 + 轨迹可视化"
echo "  窗口5: Drive   - WASD 固定速度控制"
echo ""
echo "  控制: W前进 S后退 A左转 D右转"
echo "    线速度 1.0 m/s, 角速度 0.8 rad/s"
echo "  RViz Fixed Frame -> camera_init"
echo "========================================"
