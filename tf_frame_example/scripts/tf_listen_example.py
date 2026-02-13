#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, TransformException
from tf2_ros.buffer import Buffer
from transforms3d.euler import quat2euler
from rcl_interfaces.msg import SetParametersResult


class TfListenExample(Node):
    def __init__(self):
        super().__init__('tf_listen_example')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.parent_frame = self.declare_parameter('parent_frame', 'frame1').value
        self.child_frame = self.declare_parameter('child_frame', 'frame2').value

        self.add_on_set_parameters_callback(self.param_change_cb)

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, Time())
            quat = (t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z)
            euler = quat2euler(quat)

            self.get_logger().info(
                f'Found transform from {self.parent_frame} to {self.child_frame}:\nXYZ: {t.transform.translation.x} {t.transform.translation.y} {t.transform.translation.z}\nRoll: {euler[0]}\nPitch: {euler[1]}\nYaw: {euler[2]}')

        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform from {self.parent_frame} to {self.child_frame}: {ex}')

    def param_change_cb(self, params):
        for p in params:
            if p.name == 'parent_frame':
                self.parent_frame = p.value
            elif p.name == 'child_frame':
                self.child_frame = p.value

        return SetParametersResult(successful=True)


if __name__ == '__main__':
    rclpy.init()
    node_instance = TfListenExample()
    rclpy.spin(node_instance)
    rclpy.shutdown()
