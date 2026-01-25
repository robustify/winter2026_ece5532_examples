#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class TimerExample(Node):
    def __init__(self):
        super().__init__('timer_example')


if __name__ == '__main__':
    rclpy.init()
    node_instance = TimerExample()
    rclpy.spin(node_instance)
