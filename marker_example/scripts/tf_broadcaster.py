#! /usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import tf2_ros
from geometry_msgs.msg import TransformStamped


class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        # Timer to update TF frame periodically
        self.timer = self.create_timer(timer_period_sec=0.02, callback=self.timer_callback)

        # Initialize a TF broadcaster instance
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Create stamped transform message property and set the parent and child frame IDs
        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = 'map'
        self.transform_msg.child_frame_id = 'frame1'

        # Declare parameters for 2-D position and angle, and bind an update callback
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.add_on_set_parameters_callback(self.param_change_cb)

    def param_change_cb(self, params):
        # Update values in stamped transform with the latest parameter values
        for p in params:
            if p.name == 'x':
                self.transform_msg.transform.translation.x = p.value
            elif p.name == 'y':
                self.transform_msg.transform.translation.y = p.value
            elif p.name == 'yaw':
                self.transform_msg.transform.rotation.w = math.cos(p.value)
                self.transform_msg.transform.rotation.z = math.sin(p.value)

        return SetParametersResult(successful=True)

    def timer_callback(self):
        # Update the timestamp in the stamped transform header and update the TF frame using the broadcaster
        self.transform_msg.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transform_msg)


if __name__ == '__main__':
    rclpy.init()
    node_instance = TfBroadcaster()
    rclpy.spin(node_instance)
