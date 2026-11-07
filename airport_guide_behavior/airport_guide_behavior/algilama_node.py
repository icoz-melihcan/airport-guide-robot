import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

# OpenCV HSV araliklari: H 0-179, S 0-255, V 0-255
# Saf mavi (0,0,1) -> H=120 | Saf sari (1,1,0) -> H=30
RENKLER = {
    'A': {'ad': 'MAVI', 'alt': (105, 120, 60), 'ust': (135, 255, 255)},
    'B': {'ad': 'SARI', 'alt': (22, 120, 60), 'ust': (38, 255, 255)},
}
MIN_ALAN = 0.0015   # goruntunun en az %0.15'i hedef renkteyse "tabela var" (640x480'de ~460 piksel)


class AlgilamaNode(Node):

    def __init__(self):
        super().__init__('algilama_node')
        self.declare_parameter('hedef', 'A')
        self.hedef = str(self.get_parameter('hedef').value).upper()
        if self.hedef not in RENKLER:
            self.get_logger().error(f'Gecersiz hedef "{self.hedef}", A kullaniliyor')
            self.hedef = 'A'
        self.renk = RENKLER[self.hedef]
        self.alt = np.array(self.renk['alt'], dtype=np.uint8)
        self.ust = np.array(self.renk['ust'], dtype=np.uint8)

        self.bridge = CvBridge()
        self.son_temas = -1e9

        self.create_subscription(Image, 'camera/image_raw', self.kamera_cb, 10)
        self.create_subscription(Contacts, 'bumper', self.bumper_cb, 10)
        self.info_pub = self.create_publisher(Point, 'tabela/info', 10)
        self.debug_pub = self.create_publisher(Image, 'tabela/debug', 10)
        self.marker_pub = self.create_publisher(Marker, 'bumper/marker', 10)
        self.create_timer(0.1, self.marker_yayinla)

        self.get_logger().info(
            f'algilama_node basladi. Hedef: {self.hedef} kapisi -> {self.renk["ad"]} tabela araniyor')

    def simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Tabela algilama (saf fonksiyon: ROS'tan bagimsiz, kolay test edilir)
    # ------------------------------------------------------------------
    def tabela_bul(self, bgr):
        """Donus: ((hata, alan) veya None, maske)."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        maske = cv2.inRange(hsv, self.alt, self.ust)
        maske = cv2.morphologyEx(maske, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # gurultu temizle
        alan = cv2.countNonZero(maske) / float(maske.size)
        if alan < MIN_ALAN:
            return None, maske
        m = cv2.moments(maske, True)
        cx = m['m10'] / m['m00']                     # hedef renkli piksellerin yatay merkezi
        yari_genislik = bgr.shape[1] / 2.0
        hata = (cx - yari_genislik) / yari_genislik  # -1 en sol ... 0 orta ... +1 en sag
        return (hata, alan), maske

    def kamera_cb(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        sonuc, maske = self.tabela_bul(bgr)

        info = Point()
        debug = bgr.copy()
        if sonuc is not None:
            hata, alan = sonuc
            info.x, info.y, info.z = float(hata), float(alan), 1.0
            x, y, w, h = cv2.boundingRect(maske)
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cx = int((hata + 1.0) * bgr.shape[1] / 2.0)
            cv2.line(debug, (cx, 0), (cx, bgr.shape[0] - 1), (0, 255, 0), 2)
            yazi = f'{self.renk["ad"]}  hata={hata:+.2f}  alan={alan:.3f}'
        else:
            info.z = 0.0
            yazi = f'{self.renk["ad"]} tabela yok'
        # Not: OpenCV yazi tipi Turkce karakter cizemez -> sadece ASCII
        cv2.putText(debug, yazi, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        kucuk = cv2.resize(cv2.cvtColor(maske, cv2.COLOR_GRAY2BGR),
                           (bgr.shape[1] // 4, bgr.shape[0] // 4))
        debug[-kucuk.shape[0]:, -kucuk.shape[1]:] = kucuk   # sag alt koseye maske

        self.info_pub.publish(info)
        cikis = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        cikis.header = msg.header
        self.debug_pub.publish(cikis)

    # ------------------------------------------------------------------
    # Bumper gorsellestirme
    # ------------------------------------------------------------------
    def bumper_cb(self, msg):
        if len(msg.contacts) > 0:
            if self.simdi() - self.son_temas > 1.0:
                self.get_logger().warning('BUMPER: temas algilandi!')
            self.son_temas = self.simdi()

    def marker_yayinla(self):
        m = Marker()
        m.header.frame_id = 'bumper_link'     # stamp bos (0) = RViz en guncel TF'i kullanir
        m.ns = 'bumper'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        temas = (self.simdi() - self.son_temas) < 0.5
        m.color.r, m.color.g, m.color.b = (1.0, 0.0, 0.0) if temas else (0.0, 1.0, 0.0)
        m.color.a = 1.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AlgilamaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()