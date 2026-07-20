#!/bin/bash
# Nav2 lifecycle 激活 — 等待 /map 就绪后逐一 configure + activate
set +e

source /opt/ros/jazzy/setup.bash
source /home/c/fastlio_ws/install/setup.bash 2>/dev/null || true

echo "[nav2_activate] 等待 /map..."
for i in $(seq 1 30); do
    if ros2 topic info /map 2>/dev/null | grep -q "Publisher count: 1"; then
        echo "[nav2_activate] /map 就绪 (${i}s)"
        break
    fi
    sleep 1
done

NODES=(planner_server controller_server behavior_server bt_navigator velocity_smoother)

for node in "${NODES[@]}"; do
    echo "[nav2_activate] 配置 $node..."
    ros2 lifecycle set "$node" configure 2>&1
    sleep 0.5
done

for node in "${NODES[@]}"; do
    echo "[nav2_activate] 激活 $node..."
    ros2 lifecycle set "$node" activate 2>&1
    sleep 0.3
done

echo "[nav2_activate] Nav2 全部激活!"
