#!/usr/bin/env python3
"""Simple WASD keyboard control — hold to move, release to stop"""
import os
import select
import termios
import tty
from time import time

import rclpy
from geometry_msgs.msg import Twist

LIN = 3.0
ANG = 2.0

# Linux key-repeat: ~660ms initial delay, then ~33ms repeat interval.
# We must survive the 660ms gap between first press and first repeat.
# After repeats are flowing, we can use a shorter window for release detection.
INITIAL_GRACE = 0.70   # covers the first-repeat gap
REPEAT_WINDOW  = 0.20   # once repeats confirmed, release is detected fast


def main():
    try:
        fd = os.open('/dev/tty', os.O_RDONLY | os.O_NOCTTY)
    except OSError:
        print('Cannot open /dev/tty. Run from a real terminal.')
        return

    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    rclpy.init()
    node = rclpy.create_node('keyboard_drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print(f'W/S = ±{LIN} m/s | A/D = ±{ANG} rad/s | Q = quit')
    print('Hold key to move, release to stop')

    vx = 0.0
    vz = 0.0
    last_linear = 0.0
    last_angular = 0.0
    linear_primed = False   # switched to short timeout after seeing first repeat
    angular_primed = False

    try:
        while rclpy.ok():
            r, _, _ = select.select([fd], [], [], 0.03)
            now = time()

            # Snapshot before processing new events (for repeat-detection)
            prev_linear = last_linear
            prev_angular = last_angular

            if r:
                data = os.read(fd, 64)
                if not data:
                    break
                for b in data:
                    c = chr(b).lower()
                    if c in ('q', '\x03'):
                        vx = 0.0
                        vz = 0.0
                        msg = Twist()
                        pub.publish(msg)
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        os.close(fd)
                        node.destroy_node()
                        rclpy.shutdown()
                        return
                    elif c == 'w':
                        vx = LIN
                        last_linear = now
                    elif c == 's':
                        vx = -LIN
                        last_linear = now
                    elif c == 'a':
                        vz = -ANG    # A = 右转
                        last_angular = now
                    elif c == 'd':
                        vz = ANG     # D = 左转
                        last_angular = now

            # Two events on same axis close together (< 0.10 s) →
            # key-repeat is flowing, switch to fast release detection.
            # prev_linear > 0 guards against the first-ever press
            # (prev_linear was 0, so condition fails → stays in grace period).
            if not linear_primed and vx != 0.0 and prev_linear > 0.0 and now - prev_linear < 0.10:
                linear_primed = True
            if not angular_primed and vz != 0.0 and prev_angular > 0.0 and now - prev_angular < 0.10:
                angular_primed = True

            # Per-axis timeout — long grace until first repeat arrives
            linear_limit = REPEAT_WINDOW if linear_primed else INITIAL_GRACE
            angular_limit = REPEAT_WINDOW if angular_primed else INITIAL_GRACE

            if vx != 0.0 and now - last_linear > linear_limit:
                vx = 0.0
                linear_primed = False
            if vz != 0.0 and now - last_angular > angular_limit:
                vz = 0.0
                angular_primed = False

            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vz
            pub.publish(msg)

    except KeyboardInterrupt:
        pass
    finally:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub.publish(msg)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        os.close(fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
