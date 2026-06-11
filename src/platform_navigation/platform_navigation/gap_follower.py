#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class GapFollowerNode(Node):
    def __init__(self):
        super().__init__("gap_follower")
        
        self.create_timer(1.0, self.timer_callback)

        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        
        self.get_logger().info("Gap Follower node has been started")


    def laser_scan_callback(self, msg):
        self.get_logger().info("Laser scan received")

        pass


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
