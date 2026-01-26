#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NamespaceExample(Node):
    def __init__(self):
        super().__init__('namespace_example')
        self.pub_global = self.create_publisher(msg_type=String, topic='/global_pub_topic', qos_profile=1)
        self.pub_private = self.create_publisher(msg_type=String, topic='private_pub_topic', qos_profile=1)
        self.sub_global = self.create_subscription(msg_type=String, topic='/global_sub_topic', qos_profile=1, callback=self.recv_global)
        self.sub_local = self.create_subscription(msg_type=String, topic='local_sub_topic', qos_profile=1, callback=self.recv_local)

    def recv_global(self, msg):
        pass

    def recv_local(self, msg):
        pass


if __name__ == '__main__':
    rclpy.init()
    node_instance = NamespaceExample()
    rclpy.spin(node_instance)
