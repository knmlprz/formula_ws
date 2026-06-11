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
        self.max_distance = self.declare_parameter("max_distance", 3.0).value
        self.fov_deg = self.declare_parameter("fov_deg", 180.0).value

        self.laser_subscription = self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        
        self.cmd_publisher = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        self.laser_publisher = self.create_publisher(LaserScan, "/processed_scan", 10)
        
        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Gap Follower node has been started")

    def timer_callback(self):
        # This is where you would implement the logic to compute and publish the control commands
        # based on the processed laser scan data. For now, it's just a placeholder.
        pass

    def laser_scan_callback(self, msg):        
        ranges = self._process_scan(msg)        
        ranges, start_idx = self._cut_scan(msg, ranges)
        self._publish_processed_scan(msg, ranges, start_idx)

    def _cut_scan(self, scan: LaserScan, ranges: np.ndarray):
        # cut the scan to the front 180 degrees
        fov = np.radians(self.fov_deg)
        center = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        half = int(round((fov / 2) / scan.angle_increment))
        start = max(0, center - half)
        end = min(len(ranges), center + half)
        return ranges[start:end], start

    def _process_scan(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=scan.range_max, neginf=0.0)
        ranges[ranges > self.max_distance] = 0.0
        ranges = np.clip(ranges, scan.range_min, self.max_distance)
        return ranges
    
    def _publish_processed_scan(self, scan: LaserScan, ranges: np.ndarray, start_idx: int):
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min + start_idx * scan.angle_increment
        out.angle_increment = scan.angle_increment
        out.angle_max = out.angle_min + (len(ranges) - 1) * scan.angle_increment
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = ranges.tolist()
        self.laser_publisher.publish(out)
    

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
