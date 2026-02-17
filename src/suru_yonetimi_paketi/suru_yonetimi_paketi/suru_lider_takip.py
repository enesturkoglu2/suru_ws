#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class LiderTakip(Node):
    def __init__(self):
        super().__init__('lider_takip_node')
        
        # --- QOS AYARLARI (Veri Kaybını Önlemek İçin) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- UAV0 (LİDER) ---
        self.uav0_pose = PoseStamped()
        self.liderden_veri_geldi_mi = False  # Güvenlik Kilidi 🔒
        
        # 1. Durum Verisi (State): Senin sisteminde 'mavros' etiketiyle çalışıyor.
        self.create_subscription(State, '/uav0/mavros/state', self.uav0_state_cb, qos_profile)
        
        # 2. KONUM VERİSİ (GPS): İşte düzelttiğimiz yer! 'mavros' kelimesini kaldırdık.
        # Artık şakır şakır akan o veriyi buradan yakalayacak.
        self.create_subscription(PoseStamped, '/uav0/local_position/pose', self.uav0_pose_cb, qos_profile)
        
        # Lideri Yönet (Kare çizsin)
        self.uav0_pub = self.create_publisher(PoseStamped, '/uav0/mavros/setpoint_position/local', 10)
        self.uav0_arm = self.create_client(CommandBool, '/uav0/mavros/cmd/arming')
        self.uav0_mode = self.create_client(SetMode, '/uav0/mavros/set_mode')

        # --- UAV1 (TAKİPÇİ) ---
        self.uav1_state = State()
        self.create_subscription(State, '/uav1/mavros/state', self.uav1_state_cb, qos_profile)
        
        # Takipçiyi Yönet (Lideri izlesin)
        self.uav1_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.uav1_arm = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.uav1_mode = self.create_client(SetMode, '/uav1/mavros/set_mode')

        # 20Hz Döngü (Saniyede 20 kere güncelle)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info("🦅 LİDER-TAKİPÇİ MODU AKTİF! (Doğru Topic Ayarlandı)")

    # --- CALLBACK FONKSİYONLARI ---
    def uav0_state_cb(self, msg): pass
    def uav1_state_cb(self, msg): self.uav1_state = msg

    def uav0_pose_cb(self, msg):
        # Liderden veri geldiği an kilidi açıyoruz!
        self.uav0_pose = msg
        if not self.liderden_veri_geldi_mi:
            self.get_logger().info(f"✅ Lider Görüldü! Konum: x={msg.pose.position.x:.2f} | Takip Başlıyor...")
            self.liderden_veri_geldi_mi = True

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        # --- 1. LİDERİN HAREKET PLANI (Bağımsız) ---
        p0 = PoseStamped()
        p0.pose.position.z = 5.0
        
        # Basit bir "Git-Gel" hareketi
        if elapsed < 15.0:   # 15 saniye bekle / yüksel
            p0.pose.position.x = 0.0
        elif elapsed < 30.0: # 5 metre ileri git
            p0.pose.position.x = 5.0
        else:                # Başlangıca dön
            p0.pose.position.x = 0.0

        self.uav0_pub.publish(p0)
        self.set_mode(self.uav0_mode, self.uav0_arm, "OFFBOARD", True)

        # --- 2. TAKİPÇİNİN HAREKET PLANI (Bağımlı) ---
        if self.liderden_veri_geldi_mi:
            p1 = PoseStamped()
            
            # MATEMATİK: Lider neredeysen, 2 metre arkasına (X-2) ve 2 metre sağına (Y-2) git.
            # Böylece çarpışmazlar.
            p1.pose.position.x = self.uav0_pose.pose.position.x - 2.0
            p1.pose.position.y = self.uav0_pose.pose.position.y - 2.0
            p1.pose.position.z = self.uav0_pose.pose.position.z # Aynı yükseklik

            self.uav1_pub.publish(p1)
            self.set_mode(self.uav1_mode, self.uav1_arm, "OFFBOARD", True)
        else:
            # Veri yoksa uyar (ama spam yapma)
            if int(elapsed) % 3 == 0: 
                self.get_logger().warn(f"⏳ Lider verisi bekleniyor... (Dinlenen Topic: /uav0/local_position/pose)", throttle_duration_sec=3)

    def set_mode(self, mode_client, arm_client, mode, arm):
        # Mod değiştirme ve Arm etme (Standart Prosedür)
        req_mode = SetMode.Request()
        req_mode.custom_mode = mode
        mode_client.call_async(req_mode)
        
        req_arm = CommandBool.Request()
        req_arm.value = arm
        arm_client.call_async(req_arm)

def main(args=None):
    rclpy.init(args=args)
    node = LiderTakip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()