import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

ADAY_YONLER = [math.radians(d) for d in range(-75, 76, 5)]   # denenecek yonler (robota gore)


def sinirla(deger, alt, ust):
    return max(alt, min(ust, deger))


def aci_farki(a, b):
    """a - b farkini [-pi, pi] araligina sarar."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def eksene_yuvarla(aci):
    """Aciyi en yakin 90 derecenin katina yuvarlar (terminal koridorlari x/y eksenlerine paralel)."""
    return round(aci / (math.pi / 2)) * (math.pi / 2)


class GuideNode(Node):

    def __init__(self):
        super().__init__('guide_node')

        # ---------------- Parametreler ----------------
        self.hiz = self._prm('hiz', 0.35)                   # normal ileri hiz (m/s)
        self.yavas_hiz = self._prm('yavas_hiz', 0.15)       # engel/duvar yakinken hiz
        self.donus_hizi = self._prm('donus_hizi', 0.6)      # en buyuk donus hizi (rad/s)
        self.yari_genislik = self._prm('yari_genislik', 0.32)   # robot yari genisligi + pay (m)
        self.bosluk_hedef = self._prm('bosluk_hedef', 1.5)  # secilen yonde istenen bos yol (m)
        self.engel_esik = self._prm('engel_esik', 0.5)      # hicbir yonde bu kadar yol yoksa yerinde don
        self.yavas_esik = self._prm('yavas_esik', 1.2)      # secilen yonde bundan az yol varsa yavasla
        self.kavsak_on_esik = self._prm('kavsak_on_esik', 1.4)
        self.kavsak_genislik = self._prm('kavsak_genislik', 4.0)   # sol+sag bundan buyukse kavsaktayiz
        self.k_merkez = self._prm('k_merkez', 0.15)         # koridor ortasina cekme (rad / m)
        self.k_don = self._prm('k_don', 1.2)                # secilen yone donme kazanci

        # ---------------- Durum degiskenleri ----------------
        self.durum = 'KORIDOR_TAKIP'
        self.scan_geldi = False
        self.noktalar = []                     # LiDAR noktalari (x ileri, y sol) - yakin olanlar
        self.on = self.sol = self.sag = 10.0   # on / sol yan / sag yan en yakin mesafeler
        self.yaw = 0.0
        self.ana_eksen = None                  # ana koridorun yonu (ilk odom'dan)

        # ---------------- ROS baglantilari ----------------
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.kontrol_dongusu)   # 10 Hz karar dongusu

        self.get_logger().info('guide_node v1 basladi (sadece LiDAR)')

    # ======================================================================
    # Yardimcilar
    # ======================================================================
    def _prm(self, ad, varsayilan):
        self.declare_parameter(ad, varsayilan)
        return self.get_parameter(ad).value

    def durum_degistir(self, yeni):
        if yeni != self.durum:
            self.get_logger().info(f'DURUM: {self.durum} -> {yeni}')
            self.durum = yeni

    # ======================================================================
    # Sensor geri cagirimlari
    # ======================================================================
    def scan_cb(self, msg):
        noktalar = []
        bolgeler = {'on': (-30, 30), 'sol': (60, 100), 'sag': (-100, -60)}   # derece
        en_yakin = {ad: msg.range_max for ad in bolgeler}
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            aci = msg.angle_min + i * msg.angle_increment
            derece = math.degrees(aci)
            for ad, (a1, a2) in bolgeler.items():       # ROS: pozitif aci = SOL
                if a1 <= derece <= a2 and r < en_yakin[ad]:
                    en_yakin[ad] = r
            if r < 3.0 and abs(derece) <= 110:
                noktalar.append((r * math.cos(aci), r * math.sin(aci)))
        self.on, self.sol, self.sag = en_yakin['on'], en_yakin['sol'], en_yakin['sag']
        self.noktalar = noktalar
        self.scan_geldi = True

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.ana_eksen is None:
            self.ana_eksen = eksene_yuvarla(self.yaw)

    # ======================================================================
    # LiDAR ile engelden kacarak ilerleme ("bosluk secme")
    # ======================================================================
    def bosluk(self, aci):
        """Robot 'aci' yonunde (robota gore) duz giderse kac metre bos yol var?
        Robotun genisligi kadar bir serit dusunulur; seridin icine giren en yakin LiDAR noktasi yolu keser."""
        ux, uy = math.cos(aci), math.sin(aci)
        en_yakin = 3.0
        for px, py in self.noktalar:
            ileri = px * ux + py * uy               # nokta bu yonde ne kadar ileride
            yanal = abs(py * ux - px * uy)          # seridin ortasina uzakligi
            if ileri > 0.0 and yanal < self.yari_genislik and ileri < en_yakin:
                en_yakin = ileri
        return en_yakin

    def merkez_payi(self):
        """Iki yanda da duvar/engel varsa ortaya dogru kucuk bir yon duzeltmesi (rad)."""
        if self.sol + self.sag < 3.5:          # koridor 3 m: toplam bundan buyukse bir yan acik (kavsak)
            return sinirla(self.k_merkez * (self.sol - self.sag), -0.3, 0.3)
        return 0.0

    def serbest_surus(self, cmd, istenen):
        """istenen: gitmek istedigimiz yon (robota gore, rad). Engel yoksa oraya, varsa en yakin bos yone gider."""
        istenen = sinirla(istenen, ADAY_YONLER[0], ADAY_YONLER[-1])
        mesafe = {a: self.bosluk(a) for a in ADAY_YONLER}
        en_uzak = max(mesafe.values())

        if en_uzak < self.engel_esik:
            # Hicbir yon acik degil: yerinde, toplamda daha acik olan tarafa don
            sol_toplam = sum(d for a, d in mesafe.items() if a > 0)
            sag_toplam = sum(d for a, d in mesafe.items() if a < 0)
            cmd.angular.z = self.donus_hizi if sol_toplam >= sag_toplam else -self.donus_hizi
            self.get_logger().info('Her yon kapali -> yerinde donuluyor', throttle_duration_sec=1.0)
            return

        esik = min(self.bosluk_hedef, 0.6 * en_uzak)   # duvara yaklasirken esik de kuculur
        secilen = min((a for a in ADAY_YONLER if mesafe[a] >= esik),
                      key=lambda a: abs(a - istenen))
        if abs(secilen - istenen) > math.radians(10):
            yon = 'sola' if secilen > istenen else 'saga'
            self.get_logger().info(
                f'Engel: istenen yon kapali ({self.bosluk(istenen):.2f} m) -> '
                f'{abs(math.degrees(secilen - istenen)):.0f} derece {yon} sapiliyor',
                throttle_duration_sec=1.0)

        if abs(secilen) > math.radians(45):
            cmd.linear.x = 0.0                                   # cok donmek gerekiyor: once yerinde don
        else:
            v = self.hiz if mesafe[secilen] > self.yavas_esik else self.yavas_hiz
            cmd.linear.x = v * math.cos(secilen)
        cmd.angular.z = sinirla(self.k_don * secilen, -self.donus_hizi, self.donus_hizi)

    # ======================================================================
    # Ana karar dongusu (10 Hz)
    # ======================================================================
    def kontrol_dongusu(self):
        cmd = Twist()
        if not self.scan_geldi:
            self.cmd_pub.publish(cmd)
            return
        ana_eksen = self.ana_eksen if self.ana_eksen is not None else 0.0

        if self.durum == 'KORIDOR_TAKIP':
            # Koridor 3 m genisliginde (sol + sag ~ 3 m). Toplam 4 m'yi asarsa yanlar acildi: kavsak
            hizali = abs(aci_farki(ana_eksen, self.yaw)) < math.radians(20)
            if hizali and self.sol + self.sag > self.kavsak_genislik:
                self.durum_degistir('KAVSAK')
            else:
                self.serbest_surus(cmd, aci_farki(ana_eksen, self.yaw) + self.merkez_payi())

        elif self.durum == 'KAVSAK':
            if self.on > self.kavsak_on_esik:      # karsi duvara kadar yonu koruyarak yavasca duz git
                cmd.linear.x = self.yavas_hiz
                cmd.angular.z = sinirla(1.5 * aci_farki(ana_eksen, self.yaw), -0.3, 0.3)
            else:
                self.durum_degistir('KAVSAKTA_DUR')   # v1: burada durur (kamera karari C7'de)

        # KAVSAKTA_DUR: cmd sifir kalir

        self.get_logger().info(
            f'[{self.durum}] on={self.on:.2f} sol={self.sol:.2f} sag={self.sag:.2f} '
            f'v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f}',
            throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())   # cikarken robotu durdur
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()