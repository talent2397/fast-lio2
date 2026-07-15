#!/usr/bin/env python3
"""WASD keyboard control — hold to move, release to stop"""
import os
import select
import termios
import tty
from time import time

import rclpy
from geometry_msgs.msg import Twist

LIN = 2.0
ANG = 2.0

# Key-repeat timing (Linux defaults: ~660ms initial gap, ~33ms repeat)
FIRST_HOLD = 0.80   # survive the initial 660ms gap before first repeat
FAST_CHECK = 0.20   # after first repeat confirmed, fast release detection


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
    pub = node.create_publisher(Twist, '/robot/cmd_vel', 10)

    print(f'W/S = ±{LIN} m/s | A/D = ±{ANG} rad/s | Q = quit')
    print('Hold key to move, release to stop')

    # Per-key state: last timestamp and whether repeat is confirmed
    keys = {
        'w': {'ts': 0.0, 'vx':  LIN, 'confirmed': False},
        's': {'ts': 0.0, 'vx': -LIN, 'confirmed': False},
        'a': {'ts': 0.0, 'vz':  ANG, 'confirmed': False},
        'd': {'ts': 0.0, 'vz': -ANG, 'confirmed': False},
    }

    vx = 0.0
    vz = 0.0

    try:
        while rclpy.ok():
            r, _, _ = select.select([fd], [], [], 0.02)
            now = time()
            got_keys = False

            if r:
                data = os.read(fd, 64)
                if not data:
                    break
                got_keys = True
                for b in data:
                    c = chr(b).lower()
                    if c in ('q', '\x03'):
                        pub.publish(Twist())
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                        os.close(fd)
                        node.destroy_node()
                        rclpy.shutdown()
                        return
                    if c in keys:
                        prev_ts = keys[c]['ts']
                        keys[c]['ts'] = now
                        # Second event within 0.15s → repeat confirmed
                        if not keys[c]['confirmed'] and prev_ts > 0.0 and now - prev_ts < 0.15:
                            keys[c]['confirmed'] = True

            # Compute velocity: latest W/S wins for vx, latest A/D wins for vz
            vx = 0.0
            vz = 0.0

            # Linear (W/S): pick the key with the most recent timestamp that hasn't expired
            w_info = keys['w']
            s_info = keys['s']
            w_active = w_info['ts'] > 0.0
            s_active = s_info['ts'] > 0.0

            if w_active:
                timeout = FAST_CHECK if w_info['confirmed'] else FIRST_HOLD
                if now - w_info['ts'] > timeout:
                    w_info['ts'] = 0.0
                    w_info['confirmed'] = False
                    w_active = False
            if s_active:
                timeout = FAST_CHECK if s_info['confirmed'] else FIRST_HOLD
                if now - s_info['ts'] > timeout:
                    s_info['ts'] = 0.0
                    s_info['confirmed'] = False
                    s_active = False

            if w_active and s_active:
                # Both recently pressed → last one wins
                if w_info['ts'] > s_info['ts']:
                    vx = w_info['vx']
                else:
                    vx = s_info['vx']
            elif w_active:
                vx = w_info['vx']
            elif s_active:
                vx = s_info['vx']

            # Angular (A/D)
            a_info = keys['a']
            d_info = keys['d']
            a_active = a_info['ts'] > 0.0
            d_active = d_info['ts'] > 0.0

            if a_active:
                timeout = FAST_CHECK if a_info['confirmed'] else FIRST_HOLD
                if now - a_info['ts'] > timeout:
                    a_info['ts'] = 0.0
                    a_info['confirmed'] = False
                    a_active = False
            if d_active:
                timeout = FAST_CHECK if d_info['confirmed'] else FIRST_HOLD
                if now - d_info['ts'] > timeout:
                    d_info['ts'] = 0.0
                    d_info['confirmed'] = False
                    d_active = False

            if a_active and d_active:
                if a_info['ts'] > d_info['ts']:
                    vz = a_info['vz']
                else:
                    vz = d_info['vz']
            elif a_active:
                vz = a_info['vz']
            elif d_active:
                vz = d_info['vz']

            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vz
            pub.publish(msg)

    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        os.close(fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
