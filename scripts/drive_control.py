#!/usr/bin/env python3
"""WASD keyboard control using curses — proper key-repeat handling without external deps"""
import curses
import rclpy
from geometry_msgs.msg import Twist

LIN = 2.0
ANG = 2.0
IDLE_THRESHOLD = 13   # 13 × 50ms = 650ms → covers key-repeat initial gap


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)   # 50ms polling, non-blocking getch

    rclpy.init()
    node = rclpy.create_node('keyboard_drive')
    pub = node.create_publisher(Twist, '/robot/cmd_vel', 10)

    stdscr.addstr(0, 0, f'W/S=±{LIN} m/s | A/D=±{ANG} rad/s | Q=quit')
    stdscr.addstr(1, 0, 'Hold key to move, release to stop')
    stdscr.refresh()

    vx = 0.0
    vz = 0.0
    idle_count = 0
    last_vx = None
    last_vz = None

    while rclpy.ok():
        c = stdscr.getch()
        if c >= 0:
            idle_count = 0
            if c in (ord('q'), ord('Q'), 27):
                break
            elif c == ord('w') or c == ord('W'):
                vx = LIN
            elif c == ord('s') or c == ord('S'):
                vx = -LIN
            elif c == ord('a') or c == ord('A'):
                vz = ANG
            elif c == ord('d') or c == ord('D'):
                vz = -ANG
            else:
                pass   # ignore other keys, keep current velocity
        else:
            idle_count += 1
            if idle_count >= IDLE_THRESHOLD:
                # No key events for 650ms → truly released
                vx = 0.0
                vz = 0.0
                idle_count = 0

        # Only publish on change (reduces ROS traffic)
        if vx != last_vx or vz != last_vz:
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vz
            pub.publish(msg)
            last_vx = vx
            last_vz = vz

        rclpy.spin_once(node, timeout_sec=0.01)

    # Stop on exit
    msg = Twist()
    pub.publish(msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)
