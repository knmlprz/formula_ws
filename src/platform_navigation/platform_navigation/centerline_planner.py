#!/usr/bin/env python3
"""
Centerline planner pod pure pursuit.

Wejscie:  PoseArray na /map/cones_confirmed (tylko pozycje XY, bez kolorow).
Wyjscie:  Path na /centerline oraz MarkerArray na /centerline_markers.

Idea:
  1. Bierzemy tylko stozki PRZED robotem (w stozku widocznosci wzgledem
     kierunku jazdy) - nie parujemy miniete.
  2. Parujemy stozki tak, aby lezaly po PRZECIWNYCH stronach lokalnego
     kierunku toru (lewy vs prawy brzeg), a nie po globalnej najkrotszej
     odleglosci. To eliminuje parowanie dwoch stozkow z tej samej krawedzi.
  3. Srodki par porzadkujemy WZDLUZ toru metoda lancuchowa (greedy
     nearest-neighbor startujac od robota, idac w kierunku jazdy), a nie
     po odleglosci euklidesowej od robota.
  4. Interpolujemy srodki, zeby pure pursuit dostal gesta, gladka sciezke,
     i liczymy yaw z kierunku do nastepnego punktu.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(x, y, z, w):
    """Yaw (obrot wokol Z) z kwaternionu."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    """Kwaternion (tylko yaw) -> (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class CenterlinePlanner(Node):
    def __init__(self):
        super().__init__("centerline_planner")

        # --- parametry ---
        self.declare_parameter(
            "pair_distance_min", 1.0
        )  # min. rozstaw brzegow toru [m]
        self.declare_parameter(
            "pair_distance_max", 6.0
        )  # max. rozstaw brzegow toru [m]
        self.declare_parameter(
            "forward_fov_deg", 100.0
        )  # polowa kata stozka widocznosci [deg]
        self.declare_parameter(
            "behind_margin", 0.5
        )  # ile [m] za robotem jeszcze akceptujemy
        self.declare_parameter(
            "pair_lateral_min", 0.4
        )  # min. rozstaw poprzeczny pary [m]
        self.declare_parameter(
            "chain_max_step", 5.0
        )  # max. skok miedzy kolejnymi srodkami [m]
        self.declare_parameter(
            "interp_spacing", 0.3
        )  # gestosc waypointow po interpolacji [m]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseArray, "/map/cones_confirmed", self.cones_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/centerline", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/centerline_markers", 10)

        self._warned_no_tf = False

    # ------------------------------------------------------------------ TF
    def get_robot_pose(self):
        """Zwraca (pozycja_xy, yaw) robota w ramce map, albo None."""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            if not self._warned_no_tf:
                self.get_logger().warn("Brak transformu map->base_link (jeszcze?).")
                self._warned_no_tf = True
            return None
        pos = np.array([tf.transform.translation.x, tf.transform.translation.y])
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return pos, yaw

    # -------------------------------------------------------------- callback
    def cones_callback(self, msg):
        pose = self.get_robot_pose()
        if pose is None:
            return
        robot, robot_yaw = pose
        heading = np.array([math.cos(robot_yaw), math.sin(robot_yaw)])

        cones = np.array([[p.position.x, p.position.y] for p in msg.poses])
        if len(cones) < 2:
            return

        cones = self._filter_forward(cones, robot, heading)
        if len(cones) < 2:
            return

        centers = self._pair_cones(cones, heading)
        if len(centers) < 2:
            return

        ordered = self._order_along_track(centers, robot, heading)
        if len(ordered) < 2:
            return

        smooth = self._interpolate(ordered)

        self._publish(smooth)

    # ------------------------------------------------------ krok 1: filtracja
    def _filter_forward(self, cones, robot, heading):
        """Zostaw tylko stozki przed robotem (w stozku widocznosci)."""
        fov = math.radians(self.get_parameter("forward_fov_deg").value)
        margin = self.get_parameter("behind_margin").value
        out = []
        for c in cones:
            v = c - robot
            dist = np.linalg.norm(v)
            if dist < 1e-6:
                continue
            forward = float(np.dot(v, heading))
            if forward < -margin:  # wyraznie za robotem
                continue
            ang = math.acos(np.clip(forward / dist, -1.0, 1.0))
            if ang > fov:  # poza stozkiem widocznosci
                continue
            out.append(c)
        return np.array(out) if out else np.empty((0, 2))

    # -------------------------------------------------- krok 2: parowanie
    def _pair_cones(self, cones, heading):
        """
        Paruj stozki lezace po PRZECIWNYCH stronach kierunku jazdy.
        Wektor 'left' jest prostopadly do heading (obrot +90 deg).
        Znak rzutu stozka na 'left' mowi, czy jest po lewej (+) czy prawej (-).
        Wybieramy pary o minimalnej odleglosci sposrod dopuszczalnych.
        """
        dmin = self.get_parameter("pair_distance_min").value
        dmax = self.get_parameter("pair_distance_max").value
        lat_min = self.get_parameter("pair_lateral_min").value
        left = np.array([-heading[1], heading[0]])

        side = np.array([float(np.dot(c, left)) for c in cones])

        # kandydaci: (odleglosc, i, j) tylko dla przeciwnych stron
        candidates = []
        n = len(cones)
        for i in range(n):
            for j in range(i + 1, n):
                if (side[i] >= 0) == (side[j] >= 0):
                    continue  # ta sama strona toru -> odrzuc
                d = float(np.linalg.norm(cones[i] - cones[j]))
                if not (dmin < d < dmax):
                    continue
                # rozstaw poprzeczny (wzdluz 'left') musi byc sensowny
                lateral = abs(side[i] - side[j])
                if lateral < lat_min:
                    continue
                candidates.append((d, i, j))

        candidates.sort(key=lambda t: t[0])

        used = set()
        centers = []
        for d, i, j in candidates:
            if i in used or j in used:
                continue
            used.add(i)
            used.add(j)
            centers.append((cones[i] + cones[j]) / 2.0)
        return centers

    # ----------------------------------------- krok 3: porzadek wzdluz toru
    def _order_along_track(self, centers, robot, heading):
        """
        Greedy nearest-neighbor: zaczynamy od srodka najlepiej lezacego
        z przodu robota, potem zawsze skaczemy do najblizszego jeszcze
        nieuzytego srodka (o ile skok < chain_max_step).
        """
        max_step = self.get_parameter("chain_max_step").value
        pts = [np.asarray(c) for c in centers]
        remaining = set(range(len(pts)))

        # start: srodek najblizej robota, ale przed nim
        def start_score(idx):
            v = pts[idx] - robot
            forward = float(np.dot(v, heading))
            dist = float(np.linalg.norm(v))
            # preferuj z przodu (dodatni forward), blisko
            return (0 if forward >= 0 else 1, dist)

        start = min(remaining, key=start_score)
        order = [start]
        remaining.discard(start)

        while remaining:
            last = pts[order[-1]]
            nxt = min(remaining, key=lambda k: np.linalg.norm(pts[k] - last))
            if np.linalg.norm(pts[nxt] - last) > max_step:
                break  # przerwa w torze -> nie doklejaj odleglych smieci
            order.append(nxt)
            remaining.discard(nxt)

        return [pts[k] for k in order]

    # -------------------------------------------- krok 4: interpolacja
    def _interpolate(self, ordered):
        """Rownomierna interpolacja liniowa miedzy srodkami."""
        spacing = self.get_parameter("interp_spacing").value
        if len(ordered) < 2:
            return ordered
        out = [ordered[0]]
        for a, b in zip(ordered[:-1], ordered[1:]):
            seg = b - a
            length = np.linalg.norm(seg)
            if length < 1e-6:
                continue
            steps = max(1, int(length / spacing))
            for s in range(1, steps + 1):
                out.append(a + seg * (s / steps))
        return out

    # --------------------------------------------------- publikacja
    def _publish(self, points):
        header_stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = header_stamp

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, c in enumerate(points):
            # yaw z kierunku do nastepnego punktu (ostatni dziedziczy poprzedni)
            if i < len(points) - 1:
                d = points[i + 1] - c
            else:
                d = c - points[i - 1]
            yaw = math.atan2(d[1], d[0])
            qx, qy, qz, qw = quat_from_yaw(yaw)

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(c[0])
            ps.pose.position.y = float(c[1])
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)

            m = Marker()
            m.header = path.header
            m.ns = "centerline"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = ps.pose
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.g = 1.0
            m.color.a = 1.0
            markers.markers.append(m)

        self.path_pub.publish(path)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CenterlinePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
