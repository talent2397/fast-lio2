#!/bin/bash
LIN="1.0"; ANG="0.8"

source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/c/fastlio_ws/install/setup.bash 2>/dev/null

exec 3</dev/tty
OLD=$(stty -g -F /dev/tty)
stty -F /dev/tty -echo -icanon min 0 time 0

stopcar() {
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}' 2>/dev/null &
}
trap 'stty -F /dev/tty "$OLD" 2>/dev/null; stopcar; echo Bye' EXIT

echo "W=前 S=后 A=左 D=右 Q=退出 | 按下走,松手停"

LAST="0:0"

while true; do
  KEY=""
  while read -r -t 0.04 -n 1 -u 3 C 2>/dev/null; do
    [ -z "$C" ] && break; KEY="${KEY}${C}"
  done

  KL=$(echo "$KEY" | tr '[:upper:]' '[:lower:]')
  if echo "$KL" | grep -q 'q'; then break; fi

  VX=0; VZ=0
  if [ -n "$KL" ]; then
    echo "$KL" | grep -q 'w' && VX=$LIN
    echo "$KL" | grep -q 's' && VX=-${LIN}
    echo "$KL" | grep -q 'a' && VZ=-${ANG}
    echo "$KL" | grep -q 'd' && VZ=$ANG
  fi

  CUR="$VX:$VZ"
  if [ "$CUR" != "$LAST" ]; then
    # 杀掉旧的pub进程
    [ -n "$PUB_PID" ] && kill $PUB_PID 2>/dev/null
    if [ "$VX" = "0" ] && [ "$VZ" = "0" ]; then
      stopcar; PUB_PID=""
    else
      ros2 topic pub -r 30 /cmd_vel geometry_msgs/msg/Twist \
        "{linear:{x:$VX,y:0,z:0},angular:{x:0,y:0,z:$VZ}}" 2>/dev/null &
      PUB_PID=$!
    fi
    LAST="$CUR"
  fi

done
