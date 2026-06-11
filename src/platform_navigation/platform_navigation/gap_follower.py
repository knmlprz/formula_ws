#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        
        self.create_timer(1.0, self.timer_callback)

        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        
        self.get_logger().info("Gap Follower node has been started")


    def laser_scan_callback(self, msg):
        self.get_logger().info("Laser scan received")
        # Process the laser scan data here
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
