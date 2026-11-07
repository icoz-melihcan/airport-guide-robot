import math

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
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

        # ---------------- Parametreler (ros2 run ... --ros-args -p ad:=deger) ----------------
        self.hedef = self._prm('hedef', 'A')
        self.hiz = self._prm('hiz', 0.27)                   # normal ileri hiz (m/s)
        self.yavas_hiz = self._prm('yavas_hiz', 0.15)       # engel/duvar yakinken hiz
        self.donus_hizi = self._prm('donus_hizi', 0.6)      # en buyuk donus hizi (rad/s)
        self.yari_genislik = self._prm('yari_genislik', 0.36)   # robot yari genisligi + pay (m)
        self.bosluk_hedef = self._prm('bosluk_hedef', 1.5)  # secilen yonde istenen bos yol (m)
        self.engel_esik = self._prm('engel_esik', 0.5)      # hicbir yonde bu kadar yol yoksa yerinde don
        self.yavas_esik = self._prm('yavas_esik', 1.2)      # secilen yonde bundan az yol varsa yavasla
        self.kavsak_on_esik = self._prm('kavsak_on_esik', 1.4)
        self.kavsak_genislik = self._prm('kavsak_genislik', 4.0)   # sol+sag bundan buyukse kavsaktayiz
        self.varis_esik = self._prm('varis_esik', 0.8)      # kapi paneline bu kadar yaklasinca dur
        self.varis_alan = self._prm('varis_alan', 0.4)      # panel goruntunun en az %40'i olmali
        self.yaklasma_alan = self._prm('yaklasma_alan', 0.15)   # panel bu kadar buyukse dogrudan ona git
        self.min_kol_mesafe = self._prm('min_kol_mesafe', 3.0)  # kapi, kavsaktan en az bu kadar ileride
        self.k_merkez = self._prm('k_merkez', 0.15)         # koridor ortasina cekme (rad / m)
        self.k_don = self._prm('k_don', 1.2)                # secilen yone donme kazanci
        self.kamera_yari_fov = self._prm('kamera_yari_fov', 0.698)   # kamera yatay gorus acisinin yarisi

        # ---------------- Durum degiskenleri ----------------
        self.durum = 'KORIDOR_TAKIP'
        self.onceki_durum = 'KORIDOR_TAKIP'
        self.scan_geldi = False
        self.noktalar = []               # LiDAR noktalari (x ileri, y sol) - yakin olanlar
        self.on = self.sol = self.sag = 10.0   # on / sol yan / sag yan en yakin mesafeler
        self.x = self.y = self.yaw = 0.0
        self.ana_eksen = None            # ana koridorun yonu (ilk odom'dan)
        self.tabela = None               # (hata, alan) veya None
        self.tabela_zamani = -1e9
        self.bumper_zamani = -1e9
        self.bump_yonu = 1               # bumper sonrasi donus yonu
        self.manevra_bas = 0.0
        self.donus_hedefi = 0.0          # kapi koridorunun yonu
        self.kavsak_xy = (0.0, 0.0)      # donusun bittigi nokta
        self.kavsak_bas = 0.0
        self.kavsak_yonu = 0             # kavsakta donulecek yon: +1 sol, -1 sag, 0 henuz karar yok

        # ---------------- ROS baglantilari ----------------
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Point, 'tabela/info', self.tabela_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(Contacts, 'bumper', self.bumper_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.kontrol_dongusu)   # 10 Hz karar dongusu

        self.get_logger().info(f'guide_node basladi. Hedef: {self.hedef} kapisi')

    # ======================================================================
    # Yardimcilar
    # ======================================================================
    def _prm(self, ad, varsayilan):
        self.declare_parameter(ad, varsayilan)
        return self.get_parameter(ad).value

    def simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def durum_degistir(self, yeni):
        if yeni != self.durum:
            self.get_logger().info(f'DURUM: {self.durum} -> {yeni}')
            self.durum = yeni

    def tabela_gorunuyor(self):
        """Son 0.5 s icinde hedef tabela gorulduyse (hata, alan), yoksa None."""
        if self.tabela is not None and self.simdi() - self.tabela_zamani < 0.5:
            return self.tabela
        return None

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

    def tabela_cb(self, msg):
        if msg.z > 0.5:
            self.tabela = (msg.x, msg.y)
            self.tabela_zamani = self.simdi()
        else:
            self.tabela = None

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.ana_eksen is None:
            self.ana_eksen = eksene_yuvarla(self.yaw)

    def bumper_cb(self, msg):
        if len(msg.contacts) > 0:
            self.bumper_zamani = self.simdi()

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

        esik = min(self.bosluk_hedef, 0.6 * en_uzak)   # duvara/kapiya yaklasirken esik de kuculur
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
        simdi = self.simdi()
        ana_eksen = self.ana_eksen if self.ana_eksen is not None else 0.0

        # ---- GUVENLIK: bumper her seyden once gelir ----
        if simdi - self.bumper_zamani < 0.3 and self.durum != 'VARDI':
            ilerleme_fazinda = (self.durum == 'GERI_CEKIL' and simdi - self.manevra_bas > 3.25)
            if self.durum != 'GERI_CEKIL' or ilerleme_fazinda:
                self.get_logger().warning('BUMPER tetiklendi! Acil stop + geri cekilme')
                if self.durum != 'GERI_CEKIL':
                    self.onceki_durum = self.durum
                self.bump_yonu = 1 if self.sol >= self.sag - 0.2 else -1   # esitse sola
                self.manevra_bas = simdi
                self.durum_degistir('GERI_CEKIL')

        if self.durum == 'GERI_CEKIL':
            t = simdi - self.manevra_bas
            if t < 1.5:
                cmd.linear.x = -0.15                               # 1) ~20 cm geri
            elif t < 3.25:
                cmd.angular.z = self.bump_yonu * self.donus_hizi   # 2) ~60 derece don
            elif t < 5.25 and self.bosluk(0.0) > 0.6:
                cmd.linear.x = 0.3                                 # 3) engelin yanindan ~60 cm ilerle
            else:
                self.durum_degistir(self.onceki_durum)

        elif self.durum == 'KORIDOR_TAKIP':
            # Koridor 3 m genisliginde (sol + sag ~ 3 m). Toplam 4 m'yi asarsa yanlar acildi: kavsak
            # (sadece robot koridor yonune bakarken; yan donukken yan sektorler koridor boyunu olcer)
            hizali = abs(aci_farki(ana_eksen, self.yaw)) < math.radians(20)
            if hizali and self.sol + self.sag > self.kavsak_genislik:
                self.kavsak_bas = simdi
                self.kavsak_yonu = 0
                self.durum_degistir('KAVSAK')
            else:
                self.serbest_surus(cmd, aci_farki(ana_eksen, self.yaw) + self.merkez_payi())

        elif self.durum == 'KAVSAK':
            # 1) Kamera: hedef tabela goruntunun solunda mi saginda mi? (tabelalar henuz uzaktayken karar ver)
            t = self.tabela_gorunuyor()
            if self.kavsak_yonu == 0 and t is not None:
                hata, _ = t
                self.kavsak_yonu = 1 if hata < 0 else -1          # tabela solda -> sola donulecek
                self.get_logger().info(
                    f'Kavsak: {self.hedef} tabelasi {"SOLDA" if self.kavsak_yonu > 0 else "SAGDA"} '
                    f'(hata={hata:+.2f}) -> {"sola" if self.kavsak_yonu > 0 else "saga"} donulecek')
            # 2) Karsi duvara kadar yonu koruyarak yavasca duz git (ortalanacak duvar yok)
            if self.on > self.kavsak_on_esik:
                cmd.linear.x = self.yavas_hiz
                cmd.angular.z = sinirla(1.5 * aci_farki(ana_eksen, self.yaw), -0.3, 0.3)
            elif self.kavsak_yonu != 0:
                self.donus_hedefi = ana_eksen + self.kavsak_yonu * math.pi / 2
                self.durum_degistir('DONUS')                      # 3) kavsagin ortasinda 90 derece don
            elif simdi - self.kavsak_bas > 3.0:
                self.get_logger().warning('Kavsakta hedef tabela gorunmuyor, bekleniyor...',
                                       throttle_duration_sec=2.0)

        elif self.durum == 'DONUS':
            kalan = aci_farki(self.donus_hedefi, self.yaw)
            if abs(kalan) < math.radians(3):
                self.kavsak_xy = (self.x, self.y)
                self.durum_degistir('KAPIYA_YONEL')
            else:
                w = sinirla(2.0 * kalan, -self.donus_hizi, self.donus_hizi)
                if abs(w) < 0.15:
                    w = math.copysign(0.15, kalan)
                cmd.angular.z = w

        elif self.durum == 'KAPIYA_YONEL':
            t = self.tabela_gorunuyor()
            # Kavsaktan kapi koridoru boyunca ne kadar ilerledik? (kavsaktaki tabela ile karismasin)
            ilerleme = ((self.x - self.kavsak_xy[0]) * math.cos(self.donus_hedefi)
                        + (self.y - self.kavsak_xy[1]) * math.sin(self.donus_hedefi))
            kapiya_bakiyor = abs(aci_farki(self.donus_hedefi, self.yaw)) < math.radians(30)
            kapi_bolgesi = kapiya_bakiyor and ilerleme > self.min_kol_mesafe
            if t is not None and kapi_bolgesi and t[1] > self.varis_alan and self.on < self.varis_esik:
                self.get_logger().info(f'*** {self.hedef} KAPISINA ULASILDI! ***')
                self.durum_degistir('VARDI')
            elif t is not None:
                # Tabela goruntunun neresinde? -> robota gore aci (sag = negatif)
                tabela_acisi = -math.atan(t[0] * math.tan(self.kamera_yari_fov))
                if kapi_bolgesi and t[1] > self.yaklasma_alan and self.bosluk(tabela_acisi) > 0.5:
                    # Kapi paneli yakin (son ~2.5 m) ve arada engel yok: duvar artik engel degil hedef
                    cmd.linear.x = self.yavas_hiz
                    cmd.angular.z = sinirla(self.k_don * tabela_acisi, -0.4, 0.4)
                else:
                    self.serbest_surus(cmd, tabela_acisi)   # arada engel var: bosluk secerek dolas
            else:
                self.serbest_surus(cmd, aci_farki(self.donus_hedefi, self.yaw) + self.merkez_payi())

        # VARDI: cmd sifir kalir, robot durur

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