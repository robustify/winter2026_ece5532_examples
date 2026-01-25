#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class TimeStampExample(Node):
    def __init__(self):
        super().__init__('time_stamp_example')


if __name__ == '__main__':
    rclpy.init()
    node_instance = TimeStampExample()
    rclpy.spin(node_instance)
