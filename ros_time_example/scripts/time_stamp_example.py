#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TimeStampExample(Node):
    def __init__(self):
        super().__init__('time_stamp_example')
        self.sub_twist = self.create_subscription(topic='twist', msg_type=Twist, callback=self.recv_twist, qos_profile=1)
        self.pub_twist_stamped = self.create_publisher(topic='twist_stamped', msg_type=TwistStamped, qos_profile=1)

    def recv_twist(self, msg):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg

        self.pub_twist_stamped.publish(twist_stamped_msg)


if __name__ == '__main__':
    rclpy.init()
    node_instance = TimeStampExample()
    rclpy.spin(node_instance)
