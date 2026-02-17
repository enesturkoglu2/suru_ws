#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math # Dans için matematik lazım!

class SuruDans(Node):
    def __init__(self):
        super().__init__('suru_dans_node')
        
        # QoS Ayarları (MAVROS için şart)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- UAV0 (Lider) ---
        self.uav0_state = State()
        self.uav0_received = False
        self.create_subscription(State, '/uav0/mavros/state', self.uav0_cb, qos_profile)
        self.uav0_pub = self.create_publisher(PoseStamped, '/uav0/mavros/setpoint_position/local', 10)
        self.uav0_arm = self.create_client(CommandBool, '/uav0/mavros/cmd/arming')
        self.uav0_mode = self.create_client(SetMode, '/uav0/mavros/set_mode')

        # --- UAV1 (Takipçi) ---
        self.uav1_state = State()
        self.uav1_received = False
        self.create_subscription(State, '/uav1/mavros/state', self.uav1_cb, qos_profile)
        self.uav1_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.uav1_arm = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.uav1_mode = self.create_client(SetMode, '/uav1/mavros/set_mode')

        # Dans Parametreleri
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.dance_altitude = 10.0  # Dans yüksekliği (metre)
        self.dance_radius = 5.0     # Figürün genişliği (metre)
        self.dance_speed = 0.5      # Dans hızı
        self.uav1_delay = 1.5       # UAV1'in lideri takip gecikmesi (saniye)

        # Döngü (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("💃 KÖRFEZ DANSI BAŞLATILIYOR! MAVROS Bekleniyor...")

    def uav0_cb(self, msg): self.uav0_state = msg; self.uav0_received = True
    def uav1_cb(self, msg): self.uav1_state = msg; self.uav1_received = True

    def get_figure_eight_pose(self, time_elapsed):
        """Verilen zamana göre 8 figürü üzerindeki konumu hesaplar"""
        pose = PoseStamped()
        # 8 Çizme Matematiği (Lemniscate of Bernoulli benzeri basit bir yaklaşım)
        # X ekseninde sinüs, Y ekseninde sinüs*kosinüs ile 8 şekli çıkar.
        t = time_elapsed * self.dance_speed
        pose.pose.position.x = self.dance_radius * math.sin(t)
        pose.pose.position.y = self.dance_radius * math.sin(t) * math.cos(t)
        pose.pose.position.z = self.dance_altitude
        return pose

    def timer_callback(self):
        if not self.uav0_received or not self.uav1_received: return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        # İlk 15 saniye: Kalkış ve Hazırlık
        if elapsed < 15.0:
            if elapsed % 1.0 < 0.1: # Saniyede bir mod dene
                self.set_mode()
            
            # Lider 10m, Takipçi 8m'ye yükselsin
            p0 = PoseStamped(); p0.pose.position.z = self.dance_altitude
            p1 = PoseStamped(); p1.pose.position.z = self.dance_altitude - 2.0
            self.uav0_pub.publish(p0)
            self.uav1_pub.publish(p1)
            if elapsed > 14.8: self.get_logger().info("🕺 DANS BAŞLIYOR!")

        # 15. Saniyeden Sonra: DANS!
        else:
            dance_time = elapsed - 15.0
            # UAV0 (Lider) anlık zamanı kullanır
            p0 = self.get_figure_eight_pose(dance_time)
            # UAV1 (Takipçi) biraz gecikmeli zamanı kullanır (Takip efekti)
            p1 = self.get_figure_eight_pose(dance_time - self.uav1_delay)
            
            self.uav0_pub.publish(p0)
            self.uav1_pub.publish(p1)

    def set_mode(self):
        # UAV0
        if self.uav0_state.mode != "OFFBOARD": self.uav0_mode.call_async(SetMode.Request(custom_mode="OFFBOARD"))
        elif not self.uav0_state.armed: self.uav0_arm.call_async(CommandBool.Request(value=True))
        # UAV1
        if self.uav1_state.mode != "OFFBOARD": self.uav1_mode.call_async(SetMode.Request(custom_mode="OFFBOARD"))
        elif not self.uav1_state.armed: self.uav1_arm.call_async(CommandBool.Request(value=True))

def main(args=None):
    rclpy.init(args=args)
    node = SuruDans()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()