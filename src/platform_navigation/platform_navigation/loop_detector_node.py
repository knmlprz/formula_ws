#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy  # <-- DODANE IMPORTY DLA QoS
import math

class LoopDetectorNode(Node):
    def __init__(self):
        super().__init__('loop_detector_node')
        
        # Subskrypcja danych odometrii (z Twojego rosbaga) - zostaje bez zmian na /odom zgodnie z decyzją Tech Leada
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # --- ZMIANA: KONFIGURACJA QoS TRANSIENT LOCAL ---
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # --- ZMIANA: NOWA NAZWA TOPICU I PROFIL QoS ---
        # Zmieniono z '/control/loop_closed' na '/map/loop_closed'
        self.loop_status_pub = self.create_publisher(Bool, '/map/loop_closed', qos_profile)
        # ------------------------------------------------
        
        # Zmienne logiczne algorytmu (bez zmian)
        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0
        self.loop_detected = False
        
        # PARAMETRY KONFIGURACYJNE (wartości poglądowe, zostają bez zmian)
        self.MIN_LOOP_DISTANCE = 40.0  # Odległość w metrach, po której detektor się "uzbraja"
        self.START_RADIUS = 0.5        # Promień bramki startowej w metrach

        self.get_logger().info('Loop Detector Node został uruchomiony i czeka na dane...')
        

    def odom_callback(self, msg):
        if self.loop_detected:
            return  # Jeśli pętla została już zamknięta, nie musimy liczyć dalej. Węzeł działa, ale nic nie robi.

        # Wyciąganie pozycji X i Y z wiadomości nav_msgs/Odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # 1. Obliczanie całkowitego przejechanego dystansu (całkowanie drogi)
        if self.last_x is not None and self.last_y is not None:
            distance_step = math.sqrt((current_x - self.last_x)**2 + (current_y - self.last_y)**2)
            self.total_distance += distance_step

        self.last_x = current_x
        self.last_y = current_y

        # 2. Logika wykrywania pętli
        # WARUNEK 1: Bolid musiał przejechać minimalny dystans
        if self.total_distance > self.MIN_LOOP_DISTANCE:
            
            # WARUNEK 2: Liczenie odległości od punktu startowego (0,0)
            distance_from_start = math.sqrt((current_x - 0.0)**2 + (current_y - 0.0)**2)
            
            # WARUNEK 3: Czy wjechaliśmy z powrotem w strefę startu?
            if distance_from_start < self.START_RADIUS:
                self.loop_detected = True
                
                # Publikacja wiadomości o zamknięciu pętli (pójdzie tylko RAZ)
                status_msg = Bool()
                status_msg.data = True
                self.loop_status_pub.publish(status_msg)
                
                self.get_logger().info(f'!!! PĘTLA ZAMKNIĘTA !!! Przejechany dystans: {self.total_distance:.2f}m. Przełączanie na Pure Pursuit!')

def main(args=None):
    rclpy.init(args=args)
    node = LoopDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()