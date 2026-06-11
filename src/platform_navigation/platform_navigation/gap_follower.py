#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

class GapFollowerNode(Node):
    def __init__(self):
        super().__init__("gap_follower")
        
        self.get_logger().info("Initializing Gap Follower node")

        # Parameters
        self.max_distance = self.declare_parameter("max_distance", 1.0).value

        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)

        self.get_logger().info("Gap Follower node has been started")

    def laser_scan_callback(self, msg):
        self.get_logger().info("Laser scan received")

        msg.scan = self._cut_scan(msg.scan)
        msg.scan = self._process_scan(msg.scan)

    def _cut_scan(self, scan):
        # cut the scan to the front 180 degrees
        num_points = len(scan)
        start_index = num_points // 4
        end_index = 3 * num_points // 4
        return scan[start_index:end_index]

    def _process_scan(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=scan.range_max, neginf=0.0)

        ranges = np.clip(ranges, self.range_min, self.max_distance)
        
        return ranges
    

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
