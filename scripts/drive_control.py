#!/usr/bin/env python3
"""
WASD keyboard control — blocking read, same approach as ROS teleop_twist_keyboard.
Press W/A/S/D to move. Press Q/ESC to quit. Press SPACE to stop.
Car keeps moving at last command until a new key is received.
"""
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist

LIN = 2.0
ANG = 2.0
TOPIC = '/robot/cmd_vel'

# Movement bindings: (vx, vz)
BINDINGS = {
    'w': (LIN, 0),
    's': (-LIN, 0),
    'a': (0, ANG),     # Left turn
    'd': (0, -ANG),    # Right turn
}


def get_key(settings):
    """Read a single character from stdin (blocking)"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('keyboard_drive')
    pub = node.create_publisher(Twist, TOPIC, 10)

    print(f'W/S=±{LIN}m/s  A/D=±{ANG}rad/s  SPACE=stop  Q=quit')
    print('Car keeps moving at last speed until next key press')

    vx = 0.0
    vz = 0.0

    try:
        while rclpy.ok():
            key = get_key(settings).lower()

            if key in BINDINGS:
                vx, vz = BINDINGS[key]
            elif key in ('q', '\x03'):
                break
            else:
                # SPACE or any other key → stop
                vx = 0.0
                vz = 0.0

            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vz
            pub.publish(msg)

    except Exception as e:
        print(e)
    finally:
        msg = Twist()
        pub.publish(msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
