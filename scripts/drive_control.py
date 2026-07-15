#!/usr/bin/env python3
"""
WASD keyboard — adaptive per-key timeout (all logic in reader thread).
Initial press: 750ms grace. After 2nd repeat: 150ms fast release.
"""
import os
import select
import termios
import tty
import threading
from time import time

import rclpy
from geometry_msgs.msg import Twist

LIN   = 2.0
ANG   = 2.0
TOPIC = '/robot/cmd_vel'
INIT  = 0.75    # covers 660ms first-repeat gap
FAST  = 0.18    # release detected quickly after repeat confirmed


def reader_thread(fd, ts, timeout, running):
    """Reads keys and manages per-key state entirely in this thread."""
    prev_ts = [0.0, 0.0, 0.0, 0.0]   # previous event time for gap detection

    while running[0]:
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            continue
        try:
            data = os.read(fd, 64)
        except Exception:
            running[0] = False
            break
        if not data:
            running[0] = False
            break

        now = time()
        for b in data:
            ch = chr(b).lower()
            if ch == 'q' or b == b'\x03':
                running[0] = False
                return
            idx = -1
            if ch == 'w':
                idx = 0
            elif ch == 's':
                idx = 1
            elif ch == 'a':
                idx = 2
            elif ch == 'd':
                idx = 3
            if idx < 0:
                continue

            # Detect repeat flow: two events < 100ms apart on same key
            if timeout[idx] == INIT and prev_ts[idx] > 0 and now - prev_ts[idx] < 0.10:
                timeout[idx] = FAST

            prev_ts[idx] = ts[idx] if ts[idx] > 0 else now
            ts[idx] = now


def main():
    try:
        fd = os.open('/dev/tty', os.O_RDONLY | os.O_NOCTTY)
    except OSError:
        print('ERROR: cannot open /dev/tty')
        return

    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    rclpy.init()
    node = rclpy.create_node('keyboard_drive')
    pub = node.create_publisher(Twist, TOPIC, 10)

    print(f'\nW/S=+-{LIN:.0f}m/s  A/D=+-{ANG:.0f}rad/s  Q=quit')
    print('Hold=move  Release=stop (fast release ~180ms)\n')

    ts = [0.0, 0.0, 0.0, 0.0]            # w, s, a, d
    timeout = [INIT, INIT, INIT, INIT]     # per-key current timeout
    running = [True]

    threading.Thread(target=reader_thread,
                     args=(fd, ts, timeout, running), daemon=True).start()

    last_vx = 0.0
    last_vz = 0.0

    try:
        while rclpy.ok():
            if not running[0]:
                break

            now = time()

            w_on = ts[0] > 0 and now - ts[0] < timeout[0]
            s_on = ts[1] > 0 and now - ts[1] < timeout[1]
            a_on = ts[2] > 0 and now - ts[2] < timeout[2]
            d_on = ts[3] > 0 and now - ts[3] < timeout[3]

            # Reset timeout to INIT when key expires
            if not w_on and ts[0] > 0:
                timeout[0] = INIT
            if not s_on and ts[1] > 0:
                timeout[1] = INIT
            if not a_on and ts[2] > 0:
                timeout[2] = INIT
            if not d_on and ts[3] > 0:
                timeout[3] = INIT

            # Linear: latest W/S wins
            if w_on and (not s_on or ts[0] >= ts[1]):
                vx = LIN
            elif s_on:
                vx = -LIN
            else:
                vx = 0.0

            # Angular: A=右转 D=左转
            if a_on and (not d_on or ts[2] >= ts[3]):
                vz = -ANG
            elif d_on:
                vz = ANG
            else:
                vz = 0.0

            if vx != last_vx or vz != last_vz:
                msg = Twist()
                msg.linear.x = vx
                msg.angular.z = vz
                pub.publish(msg)
                last_vx = vx
                last_vz = vz

            rclpy.spin_once(node, timeout_sec=0.03)

    except KeyboardInterrupt:
        pass
    finally:
        running[0] = False
        msg = Twist()
        pub.publish(msg)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        os.close(fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
