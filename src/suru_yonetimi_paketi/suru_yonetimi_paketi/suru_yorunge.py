#!/usr/bin/env python3
import rclpy
import math  # <-- MATEMATİK KÜTÜPHANESİNİ EKLEDİK
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class YoungeKoruyucusu(Node):
    def __init__(self):
        super().__init__('yorunge_node')
        
        # --- QOS AYARLARI ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- UAV0 (LİDER - GÜNEŞ) ---
        self.uav0_pose = PoseStamped()
        self.liderden_veri_geldi_mi = False
        
        # Liderin Durumunu ve Konumunu Dinle
        self.create_subscription(State, '/uav0/mavros/state', self.uav0_state_cb, qos_profile)
        self.create_subscription(PoseStamped, '/uav0/local_position/pose', self.uav0_pose_cb, qos_profile)
        
        # Lideri Yönet
        self.uav0_pub = self.create_publisher(PoseStamped, '/uav0/mavros/setpoint_position/local', 10)
        self.uav0_arm = self.create_client(CommandBool, '/uav0/mavros/cmd/arming')
        self.uav0_mode = self.create_client(SetMode, '/uav0/mavros/set_mode')

        # --- UAV1 (TAKİPÇİ - UYDU) ---
        self.uav1_state = State()
        self.create_subscription(State, '/uav1/mavros/state', self.uav1_state_cb, qos_profile)
        
        # Takipçiyi Yönet
        self.uav1_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.uav1_arm = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.uav1_mode = self.create_client(SetMode, '/uav1/mavros/set_mode')

        # 20Hz Döngü
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info("🛰️ YÖRÜNGE MODU AKTİF! Lider bekleniyor...")

    # --- CALLBACK FONKSİYONLARI ---
    def uav0_state_cb(self, msg): pass
    def uav1_state_cb(self, msg): self.uav1_state = msg

    def uav0_pose_cb(self, msg):
        self.uav0_pose = msg
        if not self.liderden_veri_geldi_mi:
            self.get_logger().info(f"✅ Lider Tespit Edildi! Yörüngeye giriliyor...")
            self.liderden_veri_geldi_mi = True

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        # --- 1. LİDERİN HAREKETİ (MERKEZ) ---
        # Lider olduğu yerde yavaşça yükselip alçalsın (Nefes alma hareketi)
        p0 = PoseStamped()
        p0.pose.position.x = 0.0 # Olduğu yerde kalsın (Merkez)
        p0.pose.position.y = 0.0
        p0.pose.position.z = 5.0 + math.sin(elapsed * 0.5) * 1.0 # 4m ile 6m arasında dalgalan
        
        self.uav0_pub.publish(p0)
        self.set_mode(self.uav0_mode, self.uav0_arm, "OFFBOARD", True)

        # --- 2. TAKİPÇİNİN HAREKETİ (YÖRÜNGE) ---
        if self.liderden_veri_geldi_mi:
            p1 = PoseStamped()
            
            # --- MATEMATİKSEL BÜYÜ BURADA! ---
            YARICAP = 3.0  # Liderden 3 metre uzakta dön
            HIZ = 1.0      # Dönüş hızı (Radyan/saniye)
            
            # Açı zamanla değişir (Dönme efekti)
            theta = elapsed * HIZ 
            
            # Liderin konumu + Çember Formülü (r*cos, r*sin)
            p1.pose.position.x = self.uav0_pose.pose.position.x + (YARICAP * math.cos(theta))
            p1.pose.position.y = self.uav0_pose.pose.position.y + (YARICAP * math.sin(theta))
            p1.pose.position.z = self.uav0_pose.pose.position.z # Liderle aynı yükseklikte kal

            # Drone'un burnunu (Yaw) lidere çevirmek istersek ekstra kod gerekir 
            # ama şimdilik sadece konumu döndürelim.

            self.uav1_pub.publish(p1)
            self.set_mode(self.uav1_mode, self.uav1_arm, "OFFBOARD", True)

    def set_mode(self, mode_client, arm_client, mode, arm):
        req_mode = SetMode.Request()
        req_mode.custom_mode = mode
        mode_client.call_async(req_mode)
        req_arm = CommandBool.Request()
        req_arm.value = arm
        arm_client.call_async(req_arm)

def main(args=None):
    rclpy.init(args=args)
    node = YoungeKoruyucusu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()