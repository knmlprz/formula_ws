#!/usr/bin/env python3
"""
Pure pursuit dla platformy ackermann.

Sterowanie: publikuje geometry_msgs/TwistStamped na
/ackermann_steering_controller/reference. Kontroler sam przelicza
(v, omega) na kat skretu kol z rozstawu osi, wiec liczymy klasyczne
pure pursuit (predkosc katowa nadwozia).

Poprawki wzgledem wersji z /cmd_vel + Twist:
  1. TwistStamped na wlasciwy topic kontrolera (Twist na /cmd_vel byl
     publikowany w prozne - kontroler go nie sluchal).
  2. Najpierw znajdujemy najblizszy punkt sciezki, potem szukamy punktu
     lookahead IDAC DO PRZODU od niego - zeby nie celowac w punkt za
     robotem, gdy sciezka zaczyna sie za nim.
  3. Gdy zaden punkt nie jest dalej niz lookahead, celujemy w OSTATNI
     punkt (dojedz do konca), zamiast zatrzymywac sie w srodku toru.
  4. Adaptacyjny lookahead (rosnie z predkoscia) - opcjonalnie.
  5. use_sim_time obslugiwane przez --ros-args -p use_sim_time:=true.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        self.declare_parameter("lookahead_distance", 2.0)
        self.declare_parameter("lookahead_gain", 0.0)  # Ld = base + gain*v (0 = staly)
        self.declare_parameter("lookahead_min", 0.8)
        self.declare_parameter("linear_velocity", 1.5)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("goal_tolerance", 0.5)  # dystans do konca -> stop
        self.declare_parameter("cmd_topic", "/ackermann_steering_controller/reference")

        self.path = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, "/centerline", self.path_callback, 10)

        cmd_topic = self.get_parameter("cmd_topic").value
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.marker_pub = self.create_publisher(Marker, "/lookahead_point", 10)

        rate = self.get_parameter("control_rate").value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self._warned_no_tf = False

    def path_callback(self, msg):
        self.path = msg

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.path is None or len(self.path.poses) == 0:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            if not self._warned_no_tf:
                self.get_logger().warn("Brak transformu map->base_link.")
                self._warned_no_tf = True
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        pts = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]

        # adaptacyjny lookahead
        v = self.get_parameter("linear_velocity").value
        base = self.get_parameter("lookahead_distance").value
        gain = self.get_parameter("lookahead_gain").value
        ld_min = self.get_parameter("lookahead_min").value
        lookahead = max(ld_min, base + gain * v)

        # 1) najblizszy punkt sciezki (indeks)
        closest_i = min(
            range(len(pts)),
            key=lambda i: math.hypot(pts[i][0] - rx, pts[i][1] - ry),
        )

        # 2) warunek konca: blisko ostatniego punktu -> stop
        last = pts[-1]
        dist_to_end = math.hypot(last[0] - rx, last[1] - ry)
        goal_tol = self.get_parameter("goal_tolerance").value
        if closest_i >= len(pts) - 1 and dist_to_end < goal_tol:
            self.stop_robot()
            return

        # 3) szukaj lookahead idac DO PRZODU od najblizszego punktu
        target = None
        for i in range(closest_i, len(pts)):
            d = math.hypot(pts[i][0] - rx, pts[i][1] - ry)
            if d >= lookahead:
                target = pts[i]
                break
        if target is None:
            # nic dalej niz lookahead -> cel = ostatni punkt (dojedz do konca)
            target = last

        # 4) transformacja celu do ukladu robota
        dx = target[0] - rx
        dy = target[1] - ry
        x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_r = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        # rzeczywista odleglosc do celu (nie zakladamy = lookahead)
        ld_actual = math.hypot(x_r, y_r)
        if ld_actual < 1e-3:
            self.stop_robot()
            return

        alpha = math.atan2(y_r, x_r)
        omega = (2.0 * v * math.sin(alpha)) / ld_actual

        max_w = self.get_parameter("max_angular_velocity").value
        omega = max(-max_w, min(max_w, omega))

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)

        self.publish_marker(target)

    # ------------------------------------------------------------------
    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self.cmd_pub.publish(cmd)

    def publish_marker(self, point):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "lookahead"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=float(point[0]), y=float(point[1]), z=0.25)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color.r = 1.0
        m.color.a = 1.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
