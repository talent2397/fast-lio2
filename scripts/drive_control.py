#!/usr/bin/env python3
"""
WASD keyboard control — thread reads keys from /dev/tty, main loop publishes.
800ms hold: covers the ~660ms initial key-repeat delay for ALL keys equally.
"""
import os
import select
import termios
import tty
import threading
from time import time

import rclpy
from geometry_msgs.msg import Twist

LIN = 2.0
ANG = 2.0
TOPIC = '/robot/cmd_vel'
HOLD = 0.80   # 800ms > 660ms key-repeat initial gap, same for every key


def reader_thread(fd, ts, running):
    """Blocking read from /dev/tty. Updates timestamp array on key events."""
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
            elif ch == 'w':
                ts[0] = now
            elif ch == 's':
                ts[1] = now
            elif ch == 'a':
                ts[2] = now
            elif ch == 'd':
                ts[3] = now


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

    print(f'\nW/S = +/- {LIN} m/s  |  A/D = +/- {ANG} rad/s')
    print('Hold = move.  Release = stop.  Q = quit.\n')

    # ts[0]=W, [1]=S, [2]=A, [3]=D    (lists are mutable across threads)
    ts = [0.0, 0.0, 0.0, 0.0]
    running = [True]

    t = threading.Thread(target=reader_thread, args=(fd, ts, running), daemon=True)
    t.start()

    last_vx = 0.0
    last_vz = 0.0

    try:
        while rclpy.ok():
            if not running[0]:
                break

            now = time()

            w_on = now - ts[0] < HOLD
            s_on = now - ts[1] < HOLD
            a_on = now - ts[2] < HOLD
            d_on = now - ts[3] < HOLD

            # Linear: last W/S wins
            if w_on and (not s_on or ts[0] >= ts[1]):
                vx = LIN
            elif s_on:
                vx = -LIN
            else:
                vx = 0.0

            # Angular: last A/D wins
            if a_on and (not d_on or ts[2] >= ts[3]):
                vz = ANG
            elif d_on:
                vz = -ANG
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
