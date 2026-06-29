#!/usr/bin/env python3
"""
Detekcja pachołków z danych LaserScan metodą segmentacji klastrów.
Publikuje pozycje pachołków jako MarkerArray i PoseArray w wybranej ramce.
"""
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 1.0)
        self.declare_parameter('cluster_threshold', 0.15)
        self.declare_parameter('min_cluster_size', 2)
        self.declare_parameter('max_cluster_size', 50)
        self.declare_parameter('min_cone_width', 0.05)
        self.declare_parameter('max_cone_width', 0.40)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub_poses = self.create_publisher(PoseArray, '/cones/poses', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cones/markers', 10)

    def scan_callback(self, msg: LaserScan):
        min_r = self.get_parameter('min_range').value
        max_r = self.get_parameter('max_range').value

        ranges = np.array(msg.ranges)
        valid_mask = (ranges > min_r) & (ranges < max_r) & np.isfinite(ranges)
        
        if not np.any(valid_mask):
            return

        valid_ranges = ranges[valid_mask]
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_angles = angles[valid_mask]

        xs = valid_ranges * np.cos(valid_angles)
        ys = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((xs, ys))

        clusters = self.cluster_points(points)
        local_cone_positions = []
        
        for cluster in clusters:
            if self.is_cone(cluster):
                cx = np.mean(cluster[:, 0])
                cy = np.mean(cluster[:, 1])
                local_cone_positions.append((cx, cy))

        self.publish_results(local_cone_positions, msg.header)

    def cluster_points(self, points):
        """Segmentacja wektorowa przez próg odległości."""
        threshold = self.get_parameter('cluster_threshold').value
        min_size = self.get_parameter('min_cluster_size').value
        
        diffs = np.diff(points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        
        split_indices = np.where(distances > threshold)[0] + 1
        
        clusters = np.split(points, split_indices)
        
        return [c for c in clusters if len(c) >= min_size]

    def is_cone(self, cluster):
        """Sprawdź wymiary klastra."""
        max_size = self.get_parameter('max_cluster_size').value
        min_w = self.get_parameter('min_cone_width').value
        max_w = self.get_parameter('max_cone_width').value

        if len(cluster) > max_size:
            return False

        width = np.linalg.norm(np.max(cluster, axis=0) - np.min(cluster, axis=0))
        return min_w <= width <= max_w

    def publish_results(self, local_positions, header):
        out_frame = self.get_parameter('output_frame').value
        
        transform = None
        if header.frame_id != out_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    out_frame, header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.0)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF Error: {e}", throttle_duration_sec=1.0)
                return

        pose_array = PoseArray()
        pose_array.header = Header(stamp=header.stamp, frame_id=out_frame)

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, (cx, cy) in enumerate(local_positions):
            ps = PoseStamped()
            ps.header = header
            ps.pose.position.x = float(cx)
            ps.pose.position.y = float(cy)
            ps.pose.orientation.w = 1.0

            if transform:
                ps = tf2_geometry_msgs.do_transform_pose(ps.pose, transform)
                final_pose = ps
            else:
                final_pose = ps.pose

            pose_array.poses.append(final_pose)

            m = Marker()
            m.header = Header(stamp=header.stamp, frame_id=out_frame)
            m.ns = 'detected_cones'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose = final_pose
            m.pose.position.z = 0.15 
            
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 0
            m.lifetime.nanosec = 500000000 # 0.5 sec
            
            marker_array.markers.append(m)

        self.pub_poses.publish(pose_array)
        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()