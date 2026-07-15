# CLAUDE.md — 球式机器人项目总控

## 会话启动规则

**每次新会话启动时必须：**

1. 先读取 `需求.md`，了解项目目标与架构设计
2. 再读取 `进度.md`，了解当前进度、已完成事项、遇到的问题
3. 所有回答的第一句必须以"少爷"开头
4. 全程使用中文回答

```bash
# 启动时执行
cat /home/c/fastlio_ws/需求.md
cat /home/c/fastlio_ws/进度.md
```

## 项目根路径

```
/home/c/fastlio_ws
```

## 核心技术栈

| 层级 | 技术 |
|------|------|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| 仿真引擎 | Gazebo Harmonic |
| SLAM | FAST-LIO2 (LiDAR-惯性里程计) |
| 2D 地图 | slam_toolbox (online_async, 以 FAST-LIO odometry 为定位源) |
| 导航 | Nav2 (路径规划 + 避障 + 探索) |
| 视觉检测 | YOLOv8 (目标识别) |
| 3D 定位 | LiDAR 点云投射 + 深度关联 (bbox 内点云均值 → 世界坐标) |
| 底层控制 | diff-drive / cmd_vel |
| 版本管理 | Git + GitHub (git@github.com:talent2397/fast-lio2.git) |

## 关键设计原则

### 仿真→真机迁移

统一话题命名，仿真和真机用**完全相同的话题名**。切换时只改启动方式，不改代码：

| 传感器 | 统一话题 | 消息类型 |
|--------|---------|---------|
| LiDAR 点云 | `/robot/lidar` | `sensor_msgs/PointCloud2` |
| IMU | `/robot/imu` | `sensor_msgs/Imu` |
| Camera 图像 | `/robot/camera` | `sensor_msgs/Image` |
| 里程计 | `/robot/odom` | `nav_msgs/Odometry` |
| 速度指令 | `/robot/cmd_vel` | `geometry_msgs/Twist` |

- **仿真**：Gazebo sensor → ros_gz_bridge → 统一话题
- **真机**：传感器驱动节点 → 直接发布到统一话题
- **FAST-LIO2 配置**：`lid_topic` / `imu_topic` 指向统一话题
- **YOLO 节点**：订阅 `/robot/camera`
- **Nav2**：使用 `/robot/odom` 做定位源

### 地图管线

```
FAST-LIO2 → /cloud_registered (3D点云 + 高精度里程计)
                ↓
      pointcloud_to_laserscan → /scan (2D激光扫描)
                ↓
      slam_toolbox (online_async, 以 FAST-LIO /Odometry 为 pose)
                ↓
      /map (2D 占据栅格地图 → Nav2 costmap)
```

### 模块化包结构

| 包名 | 职责 | 位置 |
|------|------|------|
| `fast_lio` | FAST-LIO2 核心 SLAM | `src/FAST_LIO_ROS2/` |
| `robot_bringup` | 启动配置、launch 文件、世界文件、bridge 配置 | `src/robot_bringup/` |
| `robot_navigation` | Nav2 配置、探索策略、状态机 | `src/robot_navigation/` |
| `robot_vision` | YOLO 检测、3D 定位 | `src/robot_vision/` |
| `robot_description` | URDF/XACRO 机器人模型 | `src/robot_description/` |
| `livox_ros_driver2` | Livox 驱动 (真机用) | `src/livox_ros_driver2/` |

> 注：当前 `fastlio_sim` 包后续重构到各模块。

### 状态机

```
INIT → EXPLORE → VERIFY → NAVIGATE → REACHED
         ↑          │         │
         └──────────┘         │
         (目标丢失/重新搜索)   │
                              │
                    NOT_FOUND (探索完成未找到 → 日志输出)
```

## 系统环境

- **sudo 密码**：`1`
- **pip 安装**：需要 `--break-system-packages` 参数（Ubuntu 24.04 PEP 668）
- **YOLO 策略**：先使用 COCO 预训练权重（含 ball、chair 类别），仿真验证通过后再采集真实数据微调

## Git 规范

- 远程：`git@github.com:talent2397/fast-lio2.git`
- 分支：`main`
- 每次功能完成 commit，消息用中文
- `build/`、`install/`、`log/` 已 gitignore

## 编译命令

```bash
cd /home/c/fastlio_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <包名>
source install/setup.bash
```

## 当前阶段

**Phase 2: 从手动建图转向自主探索 + YOLO + Nav2 导航**

参见 `进度.md` 获取最新进度。
