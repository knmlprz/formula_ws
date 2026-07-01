#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

class ittcNode(Node): 
    def __init__(self):
        super().__init__("ittc")
        self.get_logger().info("Ittc node initialized")
        
        self.vehicle_speed = 0.0
        self.intended_speed = 0.0

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_subscriber_callback, 10)
        # Zapisujemy komendy z teleop, aby wiedzieć, czy kierowca próbuje cofać
        self.teleop_sub = self.create_subscription(TwistStamped, "/ackermann_steering_controller/reference", self.teleop_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_scan_subscriber_callback, 10)        

        self.drive_pub = self.create_publisher(TwistStamped, "/ittc/cmd_vel", 10)

    def odom_subscriber_callback(self, msg: Odometry):
        self.vehicle_speed = msg.twist.twist.linear.x

    def teleop_callback(self, msg: TwistStamped):
        self.intended_speed = msg.twist.linear.x

    def send_stop_message(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0

        self.drive_pub.publish(stop_msg)

    def laser_scan_subscriber_callback(self, msg: LaserScan):
        if self.intended_speed < 0.0:
            return

        ranges = msg.ranges
        min_angle = msg.angle_min
        increment = msg.angle_increment

        obstacle_detected = False

        for i in range(len(ranges)):
            r = ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue 
            
            current_angle = min_angle + (i * increment)
            
            approach_speed = self.vehicle_speed * math.cos(current_angle)
            if approach_speed > 0: 
                ittc = r / approach_speed
                if ittc < 0.5:
                    obstacle_detected = True
                    break 
            
            if abs(current_angle) < 0.5 and r < 0.3:
                obstacle_detected = True
                break 

        if obstacle_detected:
            self.send_stop_message()

def main(args=None):
    rclpy.init(args=args)
    node = ittcNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()