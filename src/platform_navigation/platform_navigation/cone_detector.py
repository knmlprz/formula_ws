#!/usr/bin/env python3
"""
Detekcja pachołków z danych LaserScan metodą segmentacji klastrów.
Publikuje pozycje pachołków jako MarkerArray i PoseArray.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped


class ConeDetector(Node):
    def __init__(self):
        super().__init__("cone_detector")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("output_frame", "base_link")
        # Parametry – dostosuj do RPLiDAR
        self.declare_parameter("min_range", 0.45)
        self.declare_parameter("max_range", 16.0)
        self.declare_parameter(
            "cluster_threshold", 0.15
        )  # max odległość między punktami klastra [m]
        self.declare_parameter("min_cluster_size", 2)
        self.declare_parameter("max_cluster_size", 15)
        self.declare_parameter("min_cone_width", 0.05)
        self.declare_parameter("max_cone_width", 0.40)  # średnica pachołka FS = ~0.305m

        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_poses = self.create_publisher(PoseArray, "/cones/poses", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/cones/markers", 10)

    def scan_callback(self, msg: LaserScan):
        min_r = self.get_parameter("min_range").value
        max_r = self.get_parameter("max_range").value

        # Konwertuj polar → kartezjański, filtruj NaN i out-of-range
        points = []
        for i, r in enumerate(msg.ranges):
            if min_r < r < max_r and not math.isnan(r) and not math.isinf(r):
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

        if not points:
            return

        # Segmentacja klastrów (Euclidean clustering 2D)
        clusters = self.cluster_points(points)

        # Filtruj klastry które wyglądają jak pachołki
        cone_positions = []
        for cluster in clusters:
            if not self.is_cone(cluster):
                continue
            cx = np.mean([p[0] for p in cluster])
            cy = np.mean([p[1] for p in cluster])
            cone_positions.append((cx, cy))

        self.publish_results(cone_positions, msg.header)

    def cluster_points(self, points):
        """Prosta segmentacja przez próg odległości między sąsiadującymi punktami."""
        threshold = self.get_parameter("cluster_threshold").value
        clusters = []
        current = [points[0]]

        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < threshold:
                current.append(points[i])
            else:
                if len(current) >= self.get_parameter("min_cluster_size").value:
                    clusters.append(current)
                current = [points[i]]

        if len(current) >= self.get_parameter("min_cluster_size").value:
            clusters.append(current)
        return clusters

    def is_cone(self, cluster):
        """Sprawdź czy klaster ma wymiary zgodne z pachołkiem FS."""
        min_size = self.get_parameter("min_cluster_size").value
        max_size = self.get_parameter("max_cluster_size").value
        min_w = self.get_parameter("min_cone_width").value
        max_w = self.get_parameter("max_cone_width").value

        if not (min_size <= len(cluster) <= max_size):
            return False

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        width = math.sqrt((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2)
        return min_w <= width <= max_w

    def publish_results(self, cone_positions, header):
        stamp = header.stamp
        frame = header.frame_id  # lidar_link
        src_frame = header.frame_id
        # PoseArray
        pose_array = PoseArray()
        pose_array.header = Header(stamp=stamp, frame_id=frame)
        for cx, cy in cone_positions:
            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.position.z = 0.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)
        self.pub_poses.publish(pose_array)

        # MarkerArray (sfery w RViz)
        marker_array = MarkerArray()
        for i, (cx, cy) in enumerate(cone_positions):
            m = Marker()
            m.header = Header(stamp=stamp, frame_id=frame)
            m.ns = "detected_cones"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 1  # znikaj po 1s jeśli brak aktualizacji
            marker_array.markers.append(m)
        self.pub_markers.publish(marker_array)
        if cone_positions:
            cx, cy = cone_positions[-1]
        out_frame = self.get_parameter("output_frame").value
        pose_array = PoseArray()
        pose_array.header = Header(stamp=stamp, frame_id=out_frame)  # Rama wyjściowa!

        for cx, cy in cone_positions:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = src_frame  # lidar_link
            ps.pose.position.x = cx
            ps.pose.position.y = cy
            ps.pose.orientation.w = 1.0

            try:
                ps_out = self.tf_buffer.transform(
                    ps, out_frame, timeout=rclpy.duration.Duration(seconds=0.05)
                )
                p = Pose()
                p.position.x = ps_out.pose.position.x
                p.position.y = ps_out.pose.position.y
                p.position.z = 0.0
                p.orientation.w = 1.0
                pose_array.poses.append(p)
            except Exception as e:
                self.get_logger().warn(f"Failed to transform cone: {e}")

        self.pub_poses.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
