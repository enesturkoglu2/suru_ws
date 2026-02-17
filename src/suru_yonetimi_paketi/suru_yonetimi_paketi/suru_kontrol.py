import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class AkilliSuru(Node):
    def __init__(self):
        super().__init__('akilli_suru_node')

        # --- AYARLAR (Senin Sistemine Göre Güncellendi) ---
        self.LIDER_NS = "/uav0"    # Lider Drone (UAV0)
        self.TAKIPCI_NS = "/uav1"  # Takipçi Drone (UAV1)
        
        self.YUKSEKLIK = 5.0          # Hedef Uçuş Yüksekliği (metre)
        self.TAKIP_MESAFESI_X = -2.0  # Takipçi, Liderin 2m gerisinde
        self.TAKIP_MESAFESI_Y = -2.0  # Takipçi, Liderin 2m solunda

        # --- LİDER İÇİN ---
        self.lider_konum_sub = self.create_subscription(
            PoseStamped,
            f'{self.LIDER_NS}/mavros/local_position/pose',
            self.lider_konum_callback,
            10)
        
        self.lider_pub = self.create_publisher(
            PoseStamped, 
            f'{self.LIDER_NS}/mavros/setpoint_position/local', 
            10)

        # --- TAKİPÇİ İÇİN ---
        self.takipci_pub = self.create_publisher(
            PoseStamped, 
            f'{self.TAKIPCI_NS}/mavros/setpoint_position/local', 
            10)

        # Değişkenler
        self.lider_son_konum = PoseStamped()
        self.lider_baslangic_konumu = None # İlk açılışta nerede olduğunu saklayacağız
        self.zaman_sayaci = 0.0

        # Döngü (20 Hz)
        self.timer = self.create_timer(0.05, self.kontrol_dongusu)
        self.get_logger().info("Sürü Sistemi Hazır! İlk konum verisi bekleniyor...")

    def lider_konum_callback(self, msg):
        self.lider_son_konum = msg
        
        # Eğer ilk kez veri geliyorsa, bunu başlangıç noktası olarak kaydet
        if self.lider_baslangic_konumu is None:
            self.lider_baslangic_konumu = msg
            self.get_logger().info(f"Lider Başlangıç Konumu Kaydedildi: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")

    def kontrol_dongusu(self):
        # Henüz liderden veri gelmediyse hiçbir şey yapma
        if self.lider_baslangic_konumu is None:
            return

        self.zaman_sayaci += 0.05

        # --- 1. LİDER HAREKETİ (Olduğu Yerde Daire Çizme) ---
        lider_hedef = PoseStamped()
        
        # Merkez noktası olarak "Başlangıç Konumu"nu kullanıyoruz
        merkez_x = self.lider_baslangic_konumu.pose.position.x
        merkez_y = self.lider_baslangic_konumu.pose.position.y
        
        # O merkez etrafında küçük bir daire çizelim (Yarıçap: 3 metre)
        lider_hedef.pose.position.x = merkez_x + (3.0 * math.sin(self.zaman_sayaci * 0.2))
        lider_hedef.pose.position.y = merkez_y + (3.0 * math.cos(self.zaman_sayaci * 0.2))
        lider_hedef.pose.position.z = self.YUKSEKLIK
        
        self.lider_pub.publish(lider_hedef)

        # --- 2. TAKİPÇİ HAREKETİ (Lidere Göre Konumlanma) ---
        takipci_hedef = PoseStamped()
        
        # Liderin ŞU ANKİ (hareketli) konumunu referans al
        takipci_hedef.pose.position.x = self.lider_son_konum.pose.position.x + self.TAKIP_MESAFESI_X
        takipci_hedef.pose.position.y = self.lider_son_konum.pose.position.y + self.TAKIP_MESAFESI_Y
        takipci_hedef.pose.position.z = self.YUKSEKLIK

        self.takipci_pub.publish(takipci_hedef)

def main(args=None):
    rclpy.init(args=args)
    node = AkilliSuru()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()