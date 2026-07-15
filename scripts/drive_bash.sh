#!/bin/bash
# 极简键盘控制 (纯bash, 零依赖)
# 按W/S/A/D移动, 无组合键(但响应极快), Q退出
LIN="1.0"; ANG="0.8"
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/c/fastlio_ws/install/setup.bash 2>/dev/null

STTY=$(stty -g)
stty -echo -icanon min 1 time 0
trap "stty $STTY" EXIT

echo "W/S/A/D 前进/后退/左转/右转 (${LIN}m/s ${ANG}rad/s) Q=退出"
echo "按下即走,松手即停. 提示: 无组合键, 仅单键控制"

while true; do
    K=$(dd bs=3 count=1 2>/dev/null | tr '[:upper:]' '[:lower:]')
    VX=0; VZ=0
    case "$K" in
        q) break;;
        w) VX=$LIN;;
        s) VX=-${LIN};;
        a) VZ=-${ANG};;
        d) VZ=$ANG;;
    esac
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear:{x:${VX}},angular:{z:${VZ}}}" 2>/dev/null
    # 立即停车 (下次按键时才会更新)
    sleep 0.05
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear:{x:0},angular:{z:0}}" 2>/dev/null
done
echo "Bye"
