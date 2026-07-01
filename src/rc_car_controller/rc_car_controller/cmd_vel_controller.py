#!/usr/bin/env python3
"""
cmd_vel_controller.py

Wezel ROS2 (Jazzy) subskrybujacy geometry_msgs/msg/Twist na topicu /cmd_vel
oraz geometry_msgs/msg/TwistStamped na /ackermann_steering_controller/reference
i przekladajacy go na sterowanie fizycznym pojazdem RC (model Ackermanna).

Wyznacza odometrie kolowa na podstawie zwrotnych ramek CAN z ODrive (0x009)
oraz publikuje ja na temat /odometry/filtered wraz z transformacja TF (odom -> base_link).

Zawiera wbudowane zatrzymanie awaryjne (E-Stop) na podstawie skanow z /scan.

Wszystkie wartosci sprzetowe (piny, ID osi, geometria) sa parametrami ROS2
i moga byc nadpisane z pliku config/controller_params.yaml lub z launcha.
"""

import struct
import sys
import math
import time

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


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class OdriveAxis:
    """Sterowanie pojedyncza osia ODrive przez CAN (jedno wspoldzielone polaczenie bus)."""

    def __init__(self, bus: can.BusABC, node_id: int, direction: int,
                 max_speed: float, logger):
        self.bus = bus
        self.node_id = node_id
        self.direction = direction
        self.max_speed = max_speed
        self.logger = logger
        # Stan z heartbeatu (cmd 0x01): aktualizowany w read_can
        self.current_state = 1     # domyslnie IDLE
        self.axis_error = 0
        self.last_commanded_vel = 0.0

    def _send(self, cmd_id, data):
        arbitration_id = (self.node_id << 5) | cmd_id
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanOperationError as e:
            self.logger.warn(f"Blad wysylki CAN (node {self.node_id}): {e}")

    def set_state(self, state_id: int):
        self._send(0x07, struct.pack('<I', state_id))

    def set_controller_mode(self, control_mode=2, input_mode=1):
        self._send(0x0b, struct.pack('<II', control_mode, input_mode))

    def clear_errors(self):
        self._send(0x18, struct.pack('<Q', 0))

    def set_velocity(self, velocity: float, torque_ff: float = 0.0):
        velocity = clamp(velocity, -self.max_speed, self.max_speed)
        self.last_commanded_vel = velocity
        vel_to_send = velocity * self.direction
        self._send(0x0d, struct.pack('<ff', float(vel_to_send), float(torque_ff)))

    def arm_velocity_mode(self):
        # Kolejnosc ma znaczenie: najpierw wyczysc bledy, ustaw tryb sterowania
        # (velocity + vel_ramp), wyslij prledkosc 0 (aby ODrive mial zdefiniowany
        # input i NIE rzucil bledu MISSING_INPUT), a DOPIERO potem wejdz w closed-loop.
        self.clear_errors()
        time.sleep(0.05)
        self.set_controller_mode(control_mode=2, input_mode=2)  # 2 = VELOCITY_CONTROL, 2 = INPUT_MODE_VEL_RAMP
        time.sleep(0.05)
        self.set_velocity(0.0)   # zdefiniuj input PRZED uzbrojeniem -> brak MISSING_INPUT
        time.sleep(0.05)
        self.set_state(8)  # AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.05)

    def ensure_armed(self):
        """Jesli os wypadla z closed-loop (np. do IDLE po bledzie), uzbroj ja ponownie.
        Zwraca True jesli os jest gotowa do jazdy."""
        if self.current_state != 8:  # nie w CLOSED_LOOP_CONTROL
            self.logger.warn(
                f"Os {self.node_id} nie jest w closed-loop (stan={self.current_state}, "
                f"err=0x{self.axis_error:X}). Ponowne uzbrajanie...",
                throttle_duration_sec=2.0
            )
            self.arm_velocity_mode()
            return False
        return True

    def disarm(self):
        self.set_state(1)  # AXIS_STATE_IDLE


class CmdVelController(Node):
    def __init__(self):
        super().__init__('cmd_vel_controller')

        # ---------- DEKLARACJA PARAMETROW ----------
        # Serwo (uklad kierowniczy)
        self.declare_parameter('servo_pin', 12)
        self.declare_parameter('servo_min_pulse', 0.0007)
        self.declare_parameter('servo_max_pulse', 0.0023)
        self.declare_parameter('steer_limit', 0.698)      # 40 st = 0.698 rad (max kat skretu kol)

        # ODrive / CAN
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('axis0_node_id', 0)
        self.declare_parameter('axis1_node_id', 1)
        self.declare_parameter('axis0_direction', -1)
        self.declare_parameter('axis1_direction', 1)
        self.declare_parameter('max_odrive_speed', 6.0)   # turns/s

        # Ograniczenia predkosci logicznej (wejscie cmd_vel)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)

        # Geometria pojazdu (z pomiarow)
        self.declare_parameter('wheel_radius', 0.03125)   # 62.5 mm / 2 = 31.25 mm
        self.declare_parameter('wheelbase', 0.34)         # rozstaw osi 34 cm
        self.declare_parameter('track_width', 0.21)       # rozstaw kol 21 cm

        # Ramy TF / odometrii
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # Bezpieczenstwo
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('control_loop_hz', 20.0)
        self.declare_parameter('can_feed_timeout', 0.5)

        # E-Stop (LiDAR)
        self.declare_parameter('estop_enabled', True)
        self.declare_parameter('estop_distance', 0.30)    # dystans zatrzymania
        self.declare_parameter('estop_min_range', 0.08)   # ignoruj odbicia od zderzaka
        self.declare_parameter('estop_fov', 0.52)         # polkat wycinka z przodu (rad); 0.52 ~ +/-30 st

        # ---------- ODCZYT PARAMETROW ----------
        self.servo_pin = self.get_parameter('servo_pin').value
        self.servo_min_pulse = self.get_parameter('servo_min_pulse').value
        self.servo_max_pulse = self.get_parameter('servo_max_pulse').value
        self.steer_limit = self.get_parameter('steer_limit').value

        can_interface = self.get_parameter('can_interface').value
        axis0_id = self.get_parameter('axis0_node_id').value
        axis1_id = self.get_parameter('axis1_node_id').value
        axis0_dir = self.get_parameter('axis0_direction').value
        axis1_dir = self.get_parameter('axis1_direction').value
        self.max_odrive_speed = self.get_parameter('max_odrive_speed').value

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        control_hz = self.get_parameter('control_loop_hz').value
        self.can_feed_timeout = self.get_parameter('can_feed_timeout').value

        self.estop_enabled = self.get_parameter('estop_enabled').value
        self.estop_distance = self.get_parameter('estop_distance').value
        self.estop_min_range = self.get_parameter('estop_min_range').value
        self.estop_fov = self.get_parameter('estop_fov').value

        self.axis0_direction = axis0_dir
        self.axis1_direction = axis1_dir
        self.axis0_node_id = axis0_id
        self.axis1_node_id = axis1_id

        # ---------- INICJALIZACJA SPRZETU ----------
        try:
            lgpio_factory = LGPIOFactory()
            self.servo = Servo(
                self.servo_pin,
                min_pulse_width=self.servo_min_pulse,
                max_pulse_width=self.servo_max_pulse,
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

        self.axis0 = OdriveAxis(self.bus, axis0_id, axis0_dir,
                                self.max_odrive_speed, self.get_logger())
        self.axis1 = OdriveAxis(self.bus, axis1_id, axis1_dir,
                                self.max_odrive_speed, self.get_logger())

        self.get_logger().info("Czyszczenie bledow i uzbrajanie obu osi ODrive (velocity mode)...")
        self.axis0.arm_velocity_mode()
        self.axis1.arm_velocity_mode()

        # ---------- ZMIENNE STANU ----------
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = self.get_clock().now()

        self.vel_axis0 = 0.0
        self.vel_axis1 = 0.0
        self.last_axis0_feed_time = self.get_clock().now()
        self.last_axis1_feed_time = self.get_clock().now()

        self.estop_active = False

        # ---------- ROS2 PUB/SUB ----------
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subskrypcja 1: Manualna klawiatura (Twist)
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)

        # Subskrypcja 2: Autonomiczny planer (TwistStamped)
        self.sub_stamped = self.create_subscription(
            TwistStamped, '/ackermann_steering_controller/reference',
            self.cmd_vel_stamped_callback, qos)

        # Subskrypcja 3: Skaner laserowy (E-Stop)
        if self.estop_enabled:
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)

        self.control_timer = self.create_timer(1.0 / control_hz, self.control_loop)

        self.get_logger().info(
            f"cmd_vel_controller gotowy. "
            f"E-Stop: {'ON' if self.estop_enabled else 'OFF'} "
            f"(dystans: {self.estop_distance}m). "
            f"Geometria: wheelbase={self.wheelbase}m, wheel_r={self.wheel_radius}m, "
            f"steer_limit={math.degrees(self.steer_limit):.0f}st. "
            f"Nasluchuje na /cmd_vel, /ackermann_steering_controller/reference oraz /scan..."
        )

    # ---------- CALLBACKI ----------
    def cmd_vel_callback(self, msg: Twist):
        self.target_linear = clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.target_angular = clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_time = self.get_clock().now()

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        self.target_linear = clamp(msg.twist.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.target_angular = clamp(msg.twist.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        """Analiza skanow LiDAR-u pod katem ochrony przed zderzeniem."""
        estop_triggered = False
        min_detected_dist = self.estop_distance

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < self.estop_min_range:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if -self.estop_fov <= angle <= self.estop_fov:
                if r < self.estop_distance:
                    estop_triggered = True
                    min_detected_dist = min(min_detected_dist, r)

        self.estop_active = estop_triggered
        if self.estop_active:
            self.get_logger().warn(
                f"[ESTOP] Wykryto przeszkode z przodu: {min_detected_dist:.2f}m! ZATRZYMANIE.",
                throttle_duration_sec=1.0
            )

    def read_can(self):
        """Nieblokujacy odczyt ramek CAN i aktualizacja pomiarow predkosci."""
        while True:
            try:
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    break

                node_id = msg.arbitration_id >> 5
                cmd_id = msg.arbitration_id & 0x1f

                if cmd_id == 0x01:  # Heartbeat: Axis_Error (u32) + Axis_State (u8)
                    axis_error, axis_state = struct.unpack('<IB', msg.data[:5])
                    if node_id == self.axis0_node_id:
                        self.axis0.axis_error = axis_error
                        self.axis0.current_state = axis_state
                    elif node_id == self.axis1_node_id:
                        self.axis1.axis_error = axis_error
                        self.axis1.current_state = axis_state

                elif cmd_id == 0x09:  # Get_Encoder_Estimates
                    pos_est, vel_est = struct.unpack('<ff', msg.data)
                    now = self.get_clock().now()
                    if node_id == self.axis0_node_id:
                        self.vel_axis0 = vel_est
                        self.last_axis0_feed_time = now
                    elif node_id == self.axis1_node_id:
                        self.vel_axis1 = vel_est
                        self.last_axis1_feed_time = now
            except Exception:
                break

    # ---------- GLOWNA PETLA ----------
    def control_loop(self):
        # 1. Odczyt danych z magistrali CAN
        self.read_can()

        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        # Watchdog czasowy LUB aktywny E-Stop
        if elapsed > self.watchdog_timeout or self.estop_active:
            linear = 0.0
            angular = 0.0
            # Diagnostyka: rozroznij czy to watchdog (brak cmd_vel) czy E-Stop
            if not self.estop_active and self.target_linear != 0.0:
                self.get_logger().warn(
                    f"[WATCHDOG] Brak swiezych komend od {elapsed:.2f}s (limit "
                    f"{self.watchdog_timeout}s) - zatrzymanie. Teleop musi publikowac ciagle.",
                    throttle_duration_sec=1.0
                )
        else:
            linear = self.target_linear
            angular = self.target_angular

        # 2. Fizyczna predkosc liniowa pojazdu [m/s]
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        axis0_ok = ((now - self.last_axis0_feed_time).nanoseconds / 1e9) < self.can_feed_timeout
        axis1_ok = ((now - self.last_axis1_feed_time).nanoseconds / 1e9) < self.can_feed_timeout

        if axis0_ok and axis1_ok:
            v0 = self.vel_axis0 * self.axis0_direction * (2.0 * math.pi * self.wheel_radius)
            v1 = self.vel_axis1 * self.axis1_direction * (2.0 * math.pi * self.wheel_radius)
            v = (v0 + v1) / 2.0
        else:
            # Fallback symulacyjny (testy offline / brak odczytow CAN)
            v = linear

        # Kat skretu kol [rad]
        steer_angle_rad = (angular / self.max_angular_speed) * self.steer_limit if self.max_angular_speed else 0.0
        steer_angle_rad = clamp(steer_angle_rad, -self.steer_limit, self.steer_limit)

        # 3. Integracja odometrii (model Ackermanna)
        if abs(v) > 0.001 and self.wheelbase > 0.0:
            self.odom_theta += (v * math.tan(steer_angle_rad) / self.wheelbase) * dt
            self.odom_x += v * math.cos(self.odom_theta) * dt
            self.odom_y += v * math.sin(self.odom_theta) * dt

        # 4. TF odom -> base_link
        half_theta = self.odom_theta / 2.0
        quat_z = math.sin(half_theta)
        quat_w = math.cos(half_theta)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = quat_z
            t.transform.rotation.w = quat_w
            self.tf_broadcaster.sendTransform(t)

        # 5. Odometry na /odometry/filtered
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = quat_z
        odom_msg.pose.pose.orientation.w = quat_w
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = (
            v * math.tan(steer_angle_rad) / self.wheelbase if self.wheelbase > 0.0 else 0.0
        )
        self.odom_pub.publish(odom_msg)

        # 6. Sterowanie ODrive
        # Najpierw upewnij sie, ze osie sa uzbrojone (closed-loop). Jesli
        # wypadly do IDLE (np. MISSING_INPUT / blad), ensure_armed je odzyska.
        a0_ready = self.axis0.ensure_armed()
        a1_ready = self.axis1.ensure_armed()

        velocity_rps = (linear / self.max_linear_speed) * self.max_odrive_speed if self.max_linear_speed else 0.0
        try:
            # Wysylaj prledkosc tylko gdy os gotowa; w przeciwnym razie wysylanie
            # 0 utrzymuje zdefiniowany input (zapobiega ponownemu MISSING_INPUT).
            self.axis0.set_velocity(velocity_rps if a0_ready else 0.0)
            self.axis1.set_velocity(velocity_rps if a1_ready else 0.0)
        except Exception as e:
            self.get_logger().warn(f"Blad wysylania predkosci do ODrive: {e}")

        # Sterowanie serwem: mapowanie kata [rad] na zakres serwa [-1, 1]
        try:
            servo_val = clamp(steer_angle_rad / self.steer_limit, -1.0, 1.0) if self.steer_limit else 0.0
            self.servo.value = servo_val
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


if __name__ == '__main__':
    main()
