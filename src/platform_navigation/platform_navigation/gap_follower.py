#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped 

class GapFollowerNode(Node):
    def __init__(self):
        super().__init__("gap_follower")
        
        self.get_logger().info("Initializing Gap Follower node")

        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        
        self.publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_steering_controller/reference", 10)

        self.get_logger().info("Gap Follower node has been started")


    def laser_scan_callback(self, msg):
        
        ranges = msg.ranges
        ranges_list = list(msg.ranges)
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
        target_angle = msg.angle_min + (target_index * msg.angle_increment)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = target_angle
        drive_msg.drive.speed = 1.5 

        self.publisher.publish(drive_msg)

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