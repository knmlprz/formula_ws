#!/usr/bin/env python3
"""
cmd_vel_controller.py

Wezel ROS2 (Jazzy) subskrybujacy geometry_msgs/msg/Twist na topicu /cmd_vel
oraz geometry_msgs/msg/TwistStamped na /ackermann_steering_controller/reference
i przekladajacy go na sterowanie fizycznym pojazdem.

Wyznacza odometrie kolowa na podstawie zwrotnych ramek CAN z ODrive (0x009)
oraz publikuje ja na temat /odometry/filtered wraz z transformacja TF (odom -> base_link).

Zawiera wbudowane zatrzymanie awaryjne (E-Stop) na podstawie skanow z /scan.
"""

import struct
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros
import can
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory


# ================= KONFIGURACJA =================
SERVO_PIN = 12
SERVO_MIN_PULSE = 0.0007
SERVO_MAX_PULSE = 0.0023
STEER_LIMIT = 0.4  # Maksymalny bezpieczny wychyl serwa (+/-)

AXIS0_NODE_ID = 0
AXIS1_NODE_ID = 1
AXIS0_DIRECTION = -1
AXIS1_DIRECTION = 1
MAX_ODRIVE_SPEED = 6.0
CAN_INTERFACE = "can0"

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0

WATCHDOG_TIMEOUT = 0.5
CONTROL_LOOP_HZ = 20.0


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class OdriveAxis:
    """Sterowanie pojedyncza osia ODrive przez CAN (jedno wspoldzielone polaczenie bus)."""

    def __init__(self, bus: can.BusABC, node_id: int, direction: int, logger):
        self.bus = bus
        self.node_id = node_id
        self.direction = direction
        self.logger = logger

    def _send(self, cmd_id, data):
        arbitration_id = (self.node_id << 5) | cmd_id
        msg = can.Message(
            arbitration_id=arbitration_id, data=data, is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanOperationError as e:
            self.logger.warn(f"Blad wysylki CAN (node {self.node_id}): {e}")

    def set_state(self, state_id: int):
        self._send(0x07, struct.pack("<I", state_id))

    def set_controller_mode(self, control_mode=2, input_mode=1):
        self._send(0x0B, struct.pack("<II", control_mode, input_mode))

    def clear_errors(self):
        self._send(0x18, struct.pack("<Q", 0))

    def set_velocity(self, velocity: float, torque_ff: float = 0.0):
        velocity = clamp(velocity, -MAX_ODRIVE_SPEED, MAX_ODRIVE_SPEED)
        vel_to_send = velocity * self.direction
        self._send(0x0D, struct.pack("<ff", float(vel_to_send), float(torque_ff)))

    def arm_velocity_mode(self):
        self.clear_errors()
        self.set_state(8)  # AXIS_STATE_CLOSED_LOOP_CONTROL
        self.set_controller_mode(
            control_mode=2, input_mode=2
        )  # 2 = INPUT_MODE_VEL_RAMP (lagodny start)

    def disarm(self):
        self.set_state(1)  # AXIS_STATE_IDLE


class CmdVelController(Node):
    def __init__(self):
        super().__init__("cmd_vel_controller")

        self.declare_parameter("max_linear_speed", MAX_LINEAR_SPEED)
        self.declare_parameter("max_angular_speed", MAX_ANGULAR_SPEED)
        self.declare_parameter("max_odrive_speed", MAX_ODRIVE_SPEED)
        self.declare_parameter("steer_limit", STEER_LIMIT)
        self.declare_parameter("watchdog_timeout", WATCHDOG_TIMEOUT)
        self.declare_parameter("can_interface", CAN_INTERFACE)
        self.declare_parameter("wheel_radius", 0.05)  # Promien kola (np. 5 cm)
        self.declare_parameter("wheelbase", 0.33)  # Rozstaw osi (np. 33 cm)

        # PARAMETRY ZATRZYMANIA AWARYJNEGO (E-Stop):
        self.declare_parameter("estop_distance", 0.10)  # Odleglosc zatrzymania (20 cm)
        self.declare_parameter(
            "estop_min_range", 0.08
        )  # Ignoruj odbicia blisze niz 12 cm (zderzak)

        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.max_odrive_speed = self.get_parameter("max_odrive_speed").value
        self.steer_limit = self.get_parameter("steer_limit").value
        self.watchdog_timeout = self.get_parameter("watchdog_timeout").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.estop_distance = self.get_parameter("estop_distance").value
        self.estop_min_range = self.get_parameter("estop_min_range").value
        can_interface = self.get_parameter("can_interface").value

        try:
            lgpio_factory = LGPIOFactory()
            self.servo = Servo(
                SERVO_PIN,
                min_pulse_width=SERVO_MIN_PULSE,
                max_pulse_width=SERVO_MAX_PULSE,
                pin_factory=lgpio_factory,
            )
        except Exception as e:
            self.get_logger().error(f"Blad inicjalizacji serwa: {e}")
            sys.exit(1)

        try:
            self.bus = can.interface.Bus(can_interface, bustype="socketcan")
        except Exception as e:
            self.get_logger().error(f"Blad inicjalizacji CAN ({can_interface}): {e}")
            sys.exit(1)

        self.axis0 = OdriveAxis(
            self.bus, AXIS0_NODE_ID, AXIS0_DIRECTION, self.get_logger()
        )
        self.axis1 = OdriveAxis(
            self.bus, AXIS1_NODE_ID, AXIS1_DIRECTION, self.get_logger()
        )

        self.get_logger().info(
            "Czyszczenie bledow i uzbrajanie obu osi ODrive (velocity mode)..."
        )
        self.axis0.arm_velocity_mode()
        self.axis1.arm_velocity_mode()

        # Zmienne sterowania ruchem
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Zmienne Odometrii
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = self.get_clock().now()

        self.vel_axis0 = 0.0
        self.vel_axis1 = 0.0
        self.last_axis0_feed_time = self.get_clock().now()
        self.last_axis1_feed_time = self.get_clock().now()

        # Stan zatrzymania awaryjnego
        self.estop_active = False

        # Konfiguracja wydawców i subskrybentów ROS2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subskrypcja 1: Manualna klawiatura (Twist)
        self.sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, qos
        )

        # Subskrypcja 2: Autonomiczny planer (TwistStamped)
        self.sub_stamped = self.create_subscription(
            TwistStamped,
            "/ackermann_steering_controller/reference",
            self.cmd_vel_stamped_callback,
            qos,
        )

        # Subskrypcja 3: Skaner Laserowy (dla E-Stopu)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.control_timer = self.create_timer(1.0 / CONTROL_LOOP_HZ, self.control_loop)

        self.get_logger().info(
            f"cmd_vel_controller gotowy. E-Stop aktywny (dystans: {self.estop_distance}m). "
            f"Nasluchuje na /cmd_vel, /ackermann_steering_controller/reference oraz /scan..."
        )

    def cmd_vel_callback(self, msg: Twist):
        self.target_linear = clamp(
            msg.linear.x, -self.max_linear_speed, self.max_linear_speed
        )
        self.target_angular = clamp(
            msg.angular.z, -self.max_angular_speed, self.max_angular_speed
        )
        self.last_cmd_time = self.get_clock().now()

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        self.target_linear = clamp(
            msg.twist.linear.x, -self.max_linear_speed, self.max_linear_speed
        )
        self.target_angular = clamp(
            msg.twist.angular.z, -self.max_angular_speed, self.max_angular_speed
        )
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        """Analiza skanow LiDAR-u pod katem ochrony przed zderzeniem."""
        estop_triggered = False
        min_detected_dist = self.estop_distance

        for i, r in enumerate(msg.ranges):
            # Ignoruj niepoprawne odczyty oraz elementy obudowy (odbicia od zderzaka)
            if math.isnan(r) or math.isinf(r) or r < self.estop_min_range:
                continue

            # Oblicz kat wiazki i znormalizuj go do przedzialu [-pi, pi]
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Interesuje nas wycinek 60 stopni z przodu pojazdu (-30 do +30 stopni)
            if -0.52 <= angle <= 0.52:
                if r < self.estop_distance:
                    estop_triggered = True
                    min_detected_dist = min(min_detected_dist, r)

        self.estop_active = estop_triggered
        if self.estop_active:
            self.get_logger().warn(
                f"[ESTOP] Wykryto przeszkode z przodu: {min_detected_dist:.2f}m! ZATRZYMANIE.",
                throttle_duration_sec=1.0,  # Ograniczenie wysypu logow do jednego na sekunde
            )

    def read_can(self):
        """Nieblokujacy odczyt ramek CAN i aktualizacja pomiarow predkosci."""
        while True:
            try:
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    break

                node_id = msg.arbitration_id >> 5
                cmd_id = msg.arbitration_id & 0x1F

                if cmd_id == 0x09:  # Get_Encoder_Estimates
                    pos_est, vel_est = struct.unpack("<ff", msg.data)
                    now = self.get_clock().now()
                    if node_id == AXIS0_NODE_ID:
                        self.vel_axis0 = vel_est
                        self.last_axis0_feed_time = now
                    elif node_id == AXIS1_NODE_ID:
                        self.vel_axis1 = vel_est
                        self.last_axis1_feed_time = now
            except Exception:
                break

    def control_loop(self):
        # 1. Odczyt danych z magistrali CAN
        self.read_can()

        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        # Zabezpieczenie: Watchdog czasowy LUB aktywne zatrzymanie awaryjne (E-Stop)
        if elapsed > self.watchdog_timeout or self.estop_active:
            linear = 0.0
            angular = 0.0  # Wyprostowanie kol przy zatrzymaniu
        else:
            linear = self.target_linear
            angular = self.target_angular

        # 2. Obliczanie fizycznej prędkości liniowej pojazdu [m/s]
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        # Sprawdzenie, czy ODrive wysyła świeże ramki CAN (maksymalnie sprzed 0.5s)
        feed_timeout = 0.5
        axis0_ok = ((now - self.last_axis0_feed_time).nanoseconds / 1e9) < feed_timeout
        axis1_ok = ((now - self.last_axis1_feed_time).nanoseconds / 1e9) < feed_timeout

        if axis0_ok and axis1_ok:
            # Rzeczywista prędkość kół w m/s (turns/s * obwód koła)
            v0 = self.vel_axis0 * AXIS0_DIRECTION * (2.0 * math.pi * self.wheel_radius)
            v1 = self.vel_axis1 * AXIS1_DIRECTION * (2.0 * math.pi * self.wheel_radius)
            v = (v0 + v1) / 2.0
        else:
            # Fallback symulacyjny - uzyteczny przy braku odczytow / testach offline
            v = linear

        # Obliczanie kąta skrętu kół w radianach
        steer_angle_rad = (
            (angular / self.max_angular_speed) * self.steer_limit
            if self.max_angular_speed
            else 0.0
        )
        steer_angle_rad = clamp(steer_angle_rad, -self.steer_limit, self.steer_limit)

        # 3. Integracja Odometrii (Kinematyczny Model Ackermanna)
        if abs(v) > 0.001 and self.wheelbase > 0.0:
            self.odom_theta += (v * math.tan(steer_angle_rad) / self.wheelbase) * dt
            self.odom_x += v * math.cos(self.odom_theta) * dt
            self.odom_y += v * math.sin(self.odom_theta) * dt

        # 4. Publikowanie transformacji odom -> base_link (wymaganej przez SLAM)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0

        # Konwersja kata yaw na kwaternion
        half_theta = self.odom_theta / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_theta)
        t.transform.rotation.w = math.cos(half_theta)

        self.tf_broadcaster.sendTransform(t)

        # 5. Publikowanie wiadomosci Odometry na /odometry/filtered
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = t.transform.rotation

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = (
            v * math.tan(steer_angle_rad) / self.wheelbase
            if self.wheelbase > 0.0
            else 0.0
        )

        self.odom_pub.publish(odom_msg)

        # 6. Sterowanie wykonawcze (Wysyłanie zadanej prędkości do ODrive)
        velocity_rps = (
            (linear / self.max_linear_speed) * self.max_odrive_speed
            if self.max_linear_speed
            else 0.0
        )
        try:
            self.axis0.set_velocity(velocity_rps)
            self.axis1.set_velocity(velocity_rps)
        except Exception as e:
            self.get_logger().warn(f"Blad wysylania predkosci do ODrive: {e}")

        # Sterowanie wykonawcze (Wychylenie serwa)
        try:
            self.servo.value = steer_angle_rad
        except Exception as e:
            self.get_logger().warn(f"Blad sterowania serwem: {e}")

        self.get_logger().debug(
            f"lin={linear:.2f} ang={angular:.2f} -> vel_rps={velocity_rps:.2f} steer={steer_angle_rad:.2f}"
        )

    def destroy_node(self):
        self.get_logger().info("Zamykanie wezla. Bezpieczne zatrzymywanie pojazdu...")
        try:
            self.axis0.set_velocity(0.0)
            self.axis1.set_velocity(0.0)
            self.axis0.disarm()
            self.axis1.disarm()
        except Exception:
            pass
        try:
            self.servo.value = 0.0
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
