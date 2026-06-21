#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

class GapFollowerNode(Node):
    def __init__(self):
        super().__init__("gap_follower")
        self.get_logger().info("Initializing Gap Follower node")

        self.publisher = self.create_publisher(
            TwistStamped, "/ackermann_steering_controller/reference", 10
        )
        self.laser_publisher = self.create_publisher(LaserScan, "/processed_scan", 10)

        self.max_distance = self.declare_parameter("max_distance", 3.0).value
        self.fov_deg = self.declare_parameter("fov_deg", 180.0).value

        # prog odleglosci - ponizej tej wartosci kierunek uznajemy za "zajety"
        self.gap_threshold = self.declare_parameter("gap_threshold", 0.5).value
        # rozmiar banki w METRACH
        self.bubble_size_m = self.declare_parameter("bubble_size_m", 0.30).value

        # parametry do przeliczenia kata skretu na predkosc katowa
        self.wheelbase = self.declare_parameter("wheelbase", 0.33).value
        self.max_steering_angle = self.declare_parameter(
            "max_steering_angle", 0.61
        ).value
        self.drive_speed = self.declare_parameter("drive_speed", 1.0).value

        self.laser_subscription = self.create_subscription(
            LaserScan, "/scan", self.laser_scan_callback, 10
        )

        self.get_logger().info("Gap Follower node has been started")

    def laser_scan_callback(self, msg):
        clean_ranges = self._process_scan(msg)
        ranges, start_idx = self._cut_scan(msg, clean_ranges)

        work = ranges.copy()
        work[work < self.gap_threshold] = 0.0
        ranges_list = work.tolist()

        closest_distance = float("inf")
        closest_index = 0
        for i in range(len(ranges_list)):
            distance = ranges_list[i]
            if distance > 0.0 and distance < closest_distance:
                closest_distance = distance
                closest_index = i

        if math.isinf(closest_distance) or closest_distance <= 0.0:
            bubble_radius = 0
        else:
            angle = math.atan2(self.bubble_size_m, closest_distance)
            bubble_radius = int(round(angle / msg.angle_increment))

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
        # środek przyciętej tablicy = przód pojazdu (0 rad)
        center = len(ranges_list) // 2
        target_angle = (target_index - center) * msg.angle_increment

        self._publish_processed_scan(msg, np.array(ranges_list), start_idx)
        self.publish_drive_msg(target_angle)

    def publish_drive_msg(self, target_angle):
        drive_msg = TwistStamped()

        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        steering_angle = max(
            -self.max_steering_angle, min(self.max_steering_angle, float(target_angle))
        )

        v = self.drive_speed
        omega = v * math.tan(steering_angle) / self.wheelbase

        drive_msg.twist.linear.x = v
        drive_msg.twist.linear.y = 0.0
        drive_msg.twist.linear.z = 0.0

        drive_msg.twist.angular.x = 0.0
        drive_msg.twist.angular.y = 0.0
        drive_msg.twist.angular.z = omega

        self.publisher.publish(drive_msg)

    def _cut_scan(self, scan: LaserScan, ranges: np.ndarray):
        # Lidar: angle_min=0, angle_max~2pi, przod pojazdu przy indeksie 0.
        # FOV symetryczne wokol przodu = koncowka tablicy (prawo) + poczatek (lewo).
        fov = np.radians(self.fov_deg)
        half = int(round((fov / 2.0) / scan.angle_increment))

        # half nie moze przekroczyc polowy skanu
        half = min(half, len(ranges) // 2)

        cut_ranges = np.concatenate([ranges[-half:], ranges[:half]])

        return cut_ranges, len(ranges) - half

    def _process_scan(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=scan.range_max, neginf=0.0)
        ranges = np.clip(ranges, scan.range_min, self.max_distance)
        return ranges

    def _publish_processed_scan(
        self, scan: LaserScan, ranges: np.ndarray, start_idx: int
    ):
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
