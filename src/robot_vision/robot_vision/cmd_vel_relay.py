#!/usr/bin/env python3
"""Relay cmd_vel_nav → /robot/cmd_vel"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Relay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_nav', self.cb, 10)
    def cb(self, msg): self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(Relay())
