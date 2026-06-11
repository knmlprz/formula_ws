#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped 
from geometry_msgs.msg import TwistStamped

class GapFollowerNode(Node):
    def __init__(self):
        super().__init__("gap_follower")
        self.get_logger().info("Initializing Gap Follower node")
        self.publisher = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        self.get_logger().info("Gap Follower node has been started")

        self.max_distance = self.declare_parameter("max_distance", 3.0).value
        self.fov_deg = self.declare_parameter("fov_deg", 180.0).value

        self.laser_subscription = self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        
        self.laser_publisher = self.create_publisher(LaserScan, "/processed_scan", 10)
        
        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Gap Follower node has been started")

    def timer_callback(self):
        # This is where you would implement the logic to compute and publish the control commands
        # based on the processed laser scan data. For now, it's just a placeholder.
        pass
    def laser_scan_callback(self, msg):
        

        clean_ranges = self._process_scan(msg)
        ranges, start_idx = self._cut_scan(msg, clean_ranges)

        ranges_list = ranges.tolist()
        bubble_radius = 150 

        closest_distance = float('inf')
        closest_index = 0
        
        for i in range(len(ranges)):
            distance = ranges[i]
            if distance > 0.0:
                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = i
                    
        start_index = max(0, closest_index - bubble_radius)
        end_index = min(len(ranges_list), closest_index + bubble_radius + 1)
        
        for i in range(start_index, end_index):
            ranges_list[i] = 0.0

        max_start = 0
        max_length = 0
        
        current_start = 0
        current_length = 0
        
        for i in range(len(ranges_list)):
            if ranges_list[i] > 0.0:
                if current_length == 0:
                    current_start = i
                current_length += 1
            else:
                if current_length > max_length:
                    max_length = current_length
                    max_start = current_start
                current_length = 0
                
        if current_length > max_length:
            max_length = current_length
            max_start = current_start

        target_index = max_start + (max_length // 2)
        
        # Przywracamy ucięte przesunięcie!
        real_target_index = start_idx + target_index 
        target_angle = msg.angle_min + (real_target_index * msg.angle_increment)
        
        self.publish_drive_msg(target_angle)
    

    def publish_drive_msg(self, target_angle):
        drive_msg = TwistStamped()
        
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link" 
        
        drive_msg.twist.linear.x = float(1.0)
        drive_msg.twist.linear.y = 0.0
        drive_msg.twist.linear.z = 0.0
        
        drive_msg.twist.angular.x = 0.0
        drive_msg.twist.angular.y = 0.0
        drive_msg.twist.angular.z = float(target_angle)

        self.publisher.publish(drive_msg)


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
        ranges = np.clip(ranges, scan.range_min, self.max_distance)
        return ranges
    
    def publish_processed_scan(self, scan: LaserScan, ranges: np.ndarray, start_idx: int):
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()