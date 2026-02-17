#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class SuruKontrol(Node):
    def __init__(self):
        super().__init__('suru_kontrol_node')
        
        # --- QoS Ayarları (KRİTİK: MAVROS ile konuşmak için şart) ---
        # MAVROS durum verisini 'Best Effort' basar. Biz de öyle dinlemeliyiz.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.get_logger().info("--- BAŞLATILIYOR: Sürü Kontrol Düğümü ---")

        # --- UAV0 Tanımları ---
        self.uav0_state = State()
        self.uav0_received = False
        self.create_subscription(State, '/uav0/mavros/state', self.uav0_cb, qos_profile)
        self.uav0_pub = self.create_publisher(PoseStamped, '/uav0/mavros/setpoint_position/local', 10)
        self.uav0_arm = self.create_client(CommandBool, '/uav0/mavros/cmd/arming')
        self.uav0_mode = self.create_client(SetMode, '/uav0/mavros/set_mode')

        # --- UAV1 Tanımları ---
        self.uav1_state = State()
        self.uav1_received = False
        self.create_subscription(State, '/uav1/mavros/state', self.uav1_cb, qos_profile)
        self.uav1_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.uav1_arm = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.uav1_mode = self.create_client(SetMode, '/uav1/mavros/set_mode')

        # Döngü (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.counter = 0
        self.get_logger().info("--- BEKLENİYOR: MAVROS Bağlantısı ---")

    def uav0_cb(self, msg):
        if not self.uav0_received:
            self.get_logger().info(f"✅ UAV0 BAĞLANDI! Mod: {msg.mode}, Armed: {msg.armed}")
            self.uav0_received = True
        self.uav0_state = msg

    def uav1_cb(self, msg):
        if not self.uav1_received:
            self.get_logger().info(f"✅ UAV1 BAĞLANDI! Mod: {msg.mode}, Armed: {msg.armed}")
            self.uav1_received = True
        self.uav1_state = msg

    def timer_callback(self):
        # Eğer veri gelmiyorsa, boşuna komut basma
        if not self.uav0_received or not self.uav1_received:
            if self.counter % 40 == 0: # 2 saniyede bir hatırlat
                self.get_logger().warn("⏳ Veri bekleniyor... (Topic isimlerini veya MAVROS'u kontrol et)")
            self.counter += 1
            return

        p0, p1 = PoseStamped(), PoseStamped()
        
        # Sürekli Konum Bas (OFFBOARD modunun yaşaması için şart)
        if self.counter < 200: # İlk 10 saniye (Hazırlık)
            p0.pose.position.z = 0.0
            p1.pose.position.z = 0.0
            
            # Saniyede bir mod değiştirmeyi dene (spam yapmamak için)
            if self.counter % 20 == 0:
                self.set_mode()
                
        else: # 10. Saniyeden sonra KALKIŞ (3 Metre)
            if self.counter == 201:
                self.get_logger().info("🚀 KALKIŞ BAŞLIYOR! Hedef: 3 Metre")
            p0.pose.position.z = 3.0
            p1.pose.position.z = 3.0
            
        self.uav0_pub.publish(p0)
        self.uav1_pub.publish(p1)
        self.counter += 1

    def set_mode(self):
        # UAV0 Kontrol
        if self.uav0_state.mode != "OFFBOARD":
            self.get_logger().info("UAV0 -> OFFBOARD İsteniyor...")
            self.uav0_mode.call_async(SetMode.Request(custom_mode="OFFBOARD"))
        elif not self.uav0_state.armed:
            self.get_logger().info("UAV0 -> ARM İsteniyor...")
            self.uav0_arm.call_async(CommandBool.Request(value=True))
        
        # UAV1 Kontrol
        if self.uav1_state.mode != "OFFBOARD":
            self.get_logger().info("UAV1 -> OFFBOARD İsteniyor...")
            self.uav1_mode.call_async(SetMode.Request(custom_mode="OFFBOARD"))
        elif not self.uav1_state.armed:
            self.get_logger().info("UAV1 -> ARM İsteniyor...")
            self.uav1_arm.call_async(CommandBool.Request(value=True))

def main(args=None):
    rclpy.init(args=args)
    node = SuruKontrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()