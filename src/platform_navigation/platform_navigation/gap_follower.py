#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

class GapFollowerNode(Node):
    def __init__(self):
        super().__init__("gap_follower")
        
        self.get_logger().info("Initializing Gap Follower node")

        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)

        self.get_logger().info("Gap Follower node has been started")


    def laser_scan_callback(self, msg):
        self.get_logger().info("Laser scan received")
        ranges = msg.ranges
        
        closest_distance = float('inf')
        closest_index = 0
        
        for i in range(len(ranges)):
            distance = ranges[i]
            
            if distance > 0.0:
                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = i



def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
