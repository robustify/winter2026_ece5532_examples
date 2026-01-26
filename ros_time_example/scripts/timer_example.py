#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration


class TimerExample(Node):
    def __init__(self):
        super().__init__('timer_example')
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)
        self.last_time = Time()

    def timer_callback(self):
        if self.last_time.clock_type != self.get_clock().now().clock_type:
            self.last_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        time_diff = Duration()
        time_diff = current_time - self.last_time
        self.last_time = current_time

        self.get_logger().info(f'Time since last trigger: {1e-9 * time_diff.nanoseconds}')


if __name__ == '__main__':
    rclpy.init()
    node_instance = TimerExample()
    rclpy.spin(node_instance)
