#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class NamespaceExample(Node):
    def __init__(self):
        super().__init__('namespace_example')


if __name__ == '__main__':
    rclpy.init()
    node_instance = NamespaceExample()
    rclpy.spin(node_instance)
