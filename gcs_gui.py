#!/usr/bin/env python3
"""
gcs_gui.py  ·  Sürü İHA Yer İstasyonu — Taktik Ekran v2
=========================================================
ROS 2 + PyQt5  |  Dark/Cyber Military Theme
─────────────────────────────────────────────────────────
Sol Panel  : Formasyon & komut butonları
Orta Panel : 2D Taktik Radar Haritası (QPainter / paintEvent)
Sağ Panel  : Canlı Telemetri Tablosu
Alt Panel  : Hedef Atama (Target Assignment)

Çalıştırmak için:
    python3 gcs_gui.py
"""

import sys, math, time, datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
    QoSDurabilityPolicy, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTableWidget, QTableWidgetItem,
    QFrame, QSizePolicy, QHeaderView, QGroupBox, QSplitter, QStatusBar,
)
from PyQt5.QtCore  import Qt, QThread, pyqtSignal, QObject, QTimer, QPointF, QRectF
from PyQt5.QtGui   import (
    QFont, QColor, QPalette, QPainter, QPen, QBrush,
    QPolygonF, QRadialGradient, QLinearGradient, QPainterPath,
)

# ═══════════════════════════════════════════════════════════════════════════════
#  TEMA SABİTLERİ
# ═══════════════════════════════════════════════════════════════════════════════
C_BG      = "#080D12"
C_PANEL   = "#0C1219"
C_CARD    = "#101820"
C_BORDER  = "#1C2733"
C_TEXT    = "#C8D8E8"
C_DIM     = "#4A5E72"
C_ACCENT  = "#00E5C0"    # Teal neon
C_BLUE    = "#4DAAFF"    # Mavi
C_WARN    = "#FFA040"    # Turuncu
C_DANGER  = "#FF4D6A"    # Kırmızı
C_SUCCESS = "#3DDB7A"    # Yeşil
C_PURPLE  = "#A66EFF"    # Mor

# Radar renkleri
RADAR_BG      = "#04080C"
RADAR_GRID    = "#0A2A1A"
RADAR_RING    = "#0D3326"
RADAR_AX      = "#0F4030"
RADAR_SWEEP   = "#00E5C055"

# UAV renkleri
UAV_COLORS = {
    "uav0": C_ACCENT,    # Lider → Teal
    "uav1": C_BLUE,      # Kanat1 → Mavi
    "uav2": C_PURPLE,    # Kanat2 → Mor
}

STYLESHEET = f"""
* {{ font-family: 'Segoe UI', 'Sans'; color: {C_TEXT}; }}
QMainWindow, QWidget {{ background: {C_BG}; }}
QGroupBox {{
    background: {C_CARD}; border: 1px solid {C_BORDER};
    border-radius: 6px; margin-top: 14px; padding: 10px 8px 8px 8px;
    font-weight: bold; font-size: 11px; color: {C_ACCENT}; letter-spacing: 1.5px;
}}
QGroupBox::title {{
    subcontrol-origin: margin; subcontrol-position: top left;
    padding: 0 6px; left: 10px;
}}
QPushButton {{
    background: {C_CARD}; color: {C_TEXT}; border: 1px solid {C_BORDER};
    border-radius: 5px; padding: 7px 12px; font-size: 12px; font-weight: 500;
}}
QPushButton:hover  {{ border-color: {C_ACCENT}; color: {C_ACCENT}; background: #0A1E18; }}
QPushButton:pressed {{ background: {C_ACCENT}; color: {C_BG}; }}
QPushButton#hold  {{
    background: #200A0F; border-color: {C_DANGER}; color: {C_DANGER}; font-weight: bold;
}}
QPushButton#hold:hover  {{ background: {C_DANGER}; color: {C_BG}; }}
QPushButton#rally {{
    background: #0A200F; border-color: {C_SUCCESS}; color: {C_SUCCESS};
}}
QPushButton#rally:hover {{ background: {C_SUCCESS}; color: {C_BG}; }}
QPushButton#attack {{
    background: #201200; border-color: {C_WARN}; color: {C_WARN};
    font-weight: bold; font-size: 13px; padding: 9px 18px;
}}
QPushButton#attack:hover {{ background: {C_WARN}; color: {C_BG}; }}
QLineEdit {{
    background: {C_CARD}; border: 1px solid {C_BORDER}; border-radius: 5px;
    padding: 7px 10px; font-family: 'Consolas'; font-size: 12px;
}}
QLineEdit:focus {{ border-color: {C_BLUE}; }}
QTableWidget {{
    background: {C_CARD}; alternate-background-color: {C_PANEL};
    gridline-color: {C_BORDER}; border: none;
    font-family: 'Consolas'; font-size: 12px;
}}
QTableWidget::item {{ padding: 4px 8px; }}
QTableWidget::item:selected {{ background: {C_BORDER}; color: {C_ACCENT}; }}
QHeaderView::section {{
    background: {C_PANEL}; color: {C_ACCENT}; border: none;
    border-bottom: 1px solid {C_BORDER}; border-right: 1px solid {C_BORDER};
    padding: 5px 8px; font-size: 10px; font-weight: bold; letter-spacing: 1px;
}}
QStatusBar {{ background: {C_PANEL}; color: {C_DIM}; border-top: 1px solid {C_BORDER}; font-size: 11px; }}
QSplitter::handle {{ background: {C_BORDER}; }}
"""

# ═══════════════════════════════════════════════════════════════════════════════
#  ROS 2 → Qt SİNYAL KÖPRÜSÜ
# ═══════════════════════════════════════════════════════════════════════════════
class RosBridge(QObject):
    telemetry  = pyqtSignal(str, float, float, float, float, float)
    # (uav_id, x, y, z_ned, speed_ms, heading_rad)
    status_msg = pyqtSignal(str)


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS 2 DÜĞÜMÜ
# ═══════════════════════════════════════════════════════════════════════════════
class GcsRosNode(Node):
    _TOPICS = {
        "uav0": "/fmu/out/vehicle_local_position_v1",
        "uav1": "/px4_1/fmu/out/vehicle_local_position_v1",
        "uav2": "/px4_2/fmu/out/vehicle_local_position_v1",
    }

    def __init__(self, bridge: RosBridge):
        super().__init__("gcs_gui_node")
        self.bridge = bridge

        qos_s = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        qos_r = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        )

        self.pub_form   = self.create_publisher(String, "/swarm/formation_cmd",    qos_r)
        self.pub_target = self.create_publisher(String, "/swarm/target_assignments", qos_r)

        for uid, topic in self._TOPICS.items():
            self.create_subscription(VehicleLocalPosition, topic,
                                     self._make_cb(uid), qos_s)
        bridge.status_msg.emit("✅  ROS 2 düğümü bağlandı.")

    def _make_cb(self, uid: str):
        def _cb(msg: VehicleLocalPosition):
            spd = math.sqrt(msg.vx**2 + msg.vy**2)
            hdg = float(getattr(msg, "heading", 0.0))
            self.bridge.telemetry.emit(uid, float(msg.x), float(msg.y),
                                       float(msg.z), spd, hdg)
        return _cb

    def send_formation(self, cmd: str):
        m = String(); m.data = cmd
        self.pub_form.publish(m)
        self.bridge.status_msg.emit(f"📡  Formasyon: {cmd}")

    def send_target(self, payload: str):
        m = String(); m.data = payload
        self.pub_target.publish(m)
        self.bridge.status_msg.emit(f"🎯  Hedef: {payload}")


class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self._n = node
    def run(self):
        rclpy.spin(self._n)


# ═══════════════════════════════════════════════════════════════════════════════
#  TAKTİK RADAR WİDGET'İ
# ═══════════════════════════════════════════════════════════════════════════════
class RadarWidget(QWidget):
    """
    2D Taktik Radar Haritası.
    • Sweep animasyonu (dönen yeşil ışın)
    • Çoklu halka ızgara
    • İHA'lar: yön oku + isim + irtifa etiketi
    • Dünya koordinatları → ekran koordinatları dönüşümü
    """

    SCALE_M     = 300.0    # Radarın kaç metreyi kapsadığı (yarıçap)
    SWEEP_SPEED = 2.0      # derece/frame

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(380, 380)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._uavs: dict[str, dict] = {}   # uid → {x,y,z,spd,hdg,ts}
        self._sweep_angle = 0.0

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick)
        self._anim_timer.start(40)   # ~25 FPS

    # ─── Veri Güncellemesi ──────────────────────────────────────────────────
    def update_uav(self, uid: str, x: float, y: float,
                   z: float, spd: float, hdg: float):
        self._uavs[uid] = {"x": x, "y": y, "z": z,
                            "spd": spd, "hdg": hdg, "ts": time.monotonic()}

    # ─── Sweep Animasyonu ──────────────────────────────────────────────────
    def _tick(self):
        self._sweep_angle = (self._sweep_angle + self.SWEEP_SPEED) % 360.0
        self.update()

    # ─── Koordinat Dönüşümü ────────────────────────────────────────────────
    def _world_to_screen(self, wx: float, wy: float) -> QPointF:
        """NED (x=kuzey, y=doğu) → ekran (x=sağ, y=aşağı)"""
        cx, cy = self.width() / 2, self.height() / 2
        r = min(cx, cy) - 20      # radar yarıçapı (piksel)
        # NED'de x=kuzey → ekranda yukarı (−), y=doğu → ekranda sağ (+)
        sx = cx + (wy / self.SCALE_M) * r
        sy = cy - (wx / self.SCALE_M) * r
        return QPointF(sx, sy)

    def _px_per_meter(self) -> float:
        cx, cy = self.width() / 2, self.height() / 2
        r = min(cx, cy) - 20
        return r / self.SCALE_M

    # ─── paintEvent ────────────────────────────────────────────────────────
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = min(cx, cy) - 20

        # ── Arka Plan ──────────────────────────────────────────────────────
        p.fillRect(0, 0, w, h, QColor(RADAR_BG))

        # ── Radyal Degrade ─────────────────────────────────────────────────
        grad = QRadialGradient(cx, cy, r)
        grad.setColorAt(0.0, QColor("#041410"))
        grad.setColorAt(1.0, QColor(RADAR_BG))
        p.setBrush(QBrush(grad))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(cx, cy), r, r)

        # ── Halka Izgaraları ───────────────────────────────────────────────
        RINGS = 4
        for i in range(1, RINGS + 1):
            ri = r * i / RINGS
            alpha = 80 + i * 20
            col = QColor(RADAR_RING); col.setAlpha(alpha)
            p.setPen(QPen(col, 1))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), ri, ri)
            # Mesafe etiketi
            dist_m = int(self.SCALE_M * i / RINGS)
            p.setPen(QColor(C_DIM))
            p.setFont(QFont("Consolas", 8))
            p.drawText(int(cx + ri + 3), int(cy) - 2, f"{dist_m}m")

        # ── Çapraz Eksen Çizgileri ─────────────────────────────────────────
        ax_col = QColor(RADAR_AX)
        p.setPen(QPen(ax_col, 1, Qt.DashLine))
        p.drawLine(int(cx), int(cy - r), int(cx), int(cy + r))
        p.drawLine(int(cx - r), int(cy), int(cx + r), int(cy))
        # 45° çizgiler
        d = r / math.sqrt(2)
        p.drawLine(int(cx - d), int(cy - d), int(cx + d), int(cy + d))
        p.drawLine(int(cx + d), int(cy - d), int(cx - d), int(cy + d))

        # ── N / E / S / W Etiketleri ───────────────────────────────────────
        lbl_col = QColor(C_ACCENT); lbl_col.setAlpha(130)
        p.setPen(lbl_col)
        p.setFont(QFont("Consolas", 9, QFont.Bold))
        p.drawText(int(cx) - 4, int(cy - r - 6), "N")
        p.drawText(int(cx + r + 4), int(cy) + 4, "E")
        p.drawText(int(cx) - 4, int(cy + r + 14), "S")
        p.drawText(int(cx - r - 14), int(cy) + 4, "W")

        # ── Sweep Işını ────────────────────────────────────────────────────
        # Math: 0°=kuzey=yukarı, sweep saat yönünde
        ang_rad = math.radians(self._sweep_angle - 90)
        sx_end  = cx + r * math.cos(ang_rad)
        sy_end  = cy + r * math.sin(ang_rad)

        sweep_col = QColor(C_ACCENT); sweep_col.setAlpha(160)
        sweep_pen  = QPen(sweep_col, 2)
        p.setPen(sweep_pen)
        p.drawLine(int(cx), int(cy), int(sx_end), int(sy_end))

        # Sweep iz (fading arc)
        arc_col = QColor(C_ACCENT); arc_col.setAlpha(40)
        arc_pen  = QPen(arc_col, r * 2)
        arc_pen.setCapStyle(Qt.FlatCap)
        p.setPen(arc_pen)
        p.setBrush(Qt.NoBrush)
        p.drawArc(
            QRectF(cx - r, cy - r, r * 2, r * 2),
            int(-(self._sweep_angle - 90) * 16),
            -45 * 16,
        )

        # ── Merkez Noktası ─────────────────────────────────────────────────
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C_ACCENT))
        p.drawEllipse(QPointF(cx, cy), 4, 4)

        # ── İHA'ları Çiz ───────────────────────────────────────────────────
        now = time.monotonic()
        for uid, data in self._uavs.items():
            stale = (now - data["ts"]) > 3.0
            col   = QColor(UAV_COLORS.get(uid, C_TEXT))
            if stale:
                col.setAlpha(60)

            sp = self._world_to_screen(data["x"], data["y"])

            # Ekran sınırlarını kırp
            margin = 18.0
            sp = QPointF(
                max(margin, min(w - margin, sp.x())),
                max(margin, min(h - margin, sp.y())),
            )

            self._draw_uav(p, sp, data["hdg"], col, uid, data["z"], stale)

        p.end()

    def _draw_uav(self, p: QPainter, pos: QPointF, hdg: float,
                  col: QColor, uid: str, z_ned: float, stale: bool):
        """İHA'yı yön gösterir üçgen + ışıma noktası + etiket olarak çiz."""
        SIZE      = 10.0   # üçgen boyutu (piksel)
        hdg_deg   = math.degrees(hdg)
        # NED heading: 0=kuzey ekranda yukarı. Ekran açısı = -(hdg-90)
        screen_angle = math.radians(-(hdg_deg - 90))

        # ── İHA Gövde Üçgeni ───────────────────────────────────────────────
        tip  = QPointF(pos.x() + SIZE * 1.6 * math.cos(screen_angle),
                       pos.y() + SIZE * 1.6 * math.sin(screen_angle))
        left_a  = screen_angle + math.radians(140)
        right_a = screen_angle - math.radians(140)
        left  = QPointF(pos.x() + SIZE * math.cos(left_a),
                        pos.y() + SIZE * math.sin(left_a))
        right = QPointF(pos.x() + SIZE * math.cos(right_a),
                        pos.y() + SIZE * math.sin(right_a))

        poly = QPolygonF([tip, left, pos, right])

        fill = QColor(col); fill.setAlpha(50 if not stale else 20)
        p.setBrush(QBrush(fill))
        p.setPen(QPen(col, 1.5))
        p.drawPolygon(poly)

        # ── Merkez Parlama Noktası ─────────────────────────────────────────
        glow = QRadialGradient(pos, 7)
        glow_col = QColor(col); glow_col.setAlpha(180)
        glow_off  = QColor(col); glow_off.setAlpha(0)
        glow.setColorAt(0.0, glow_col)
        glow.setColorAt(1.0, glow_off)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(glow))
        p.drawEllipse(pos, 7, 7)

        p.setBrush(col)
        p.drawEllipse(pos, 3, 3)

        # ── Yön Oku ────────────────────────────────────────────────────────
        arrow_end = QPointF(pos.x() + SIZE * 2.5 * math.cos(screen_angle),
                            pos.y() + SIZE * 2.5 * math.sin(screen_angle))
        p.setPen(QPen(col, 1, Qt.DotLine))
        p.drawLine(pos, arrow_end)

        # ── Etiket ─────────────────────────────────────────────────────────
        alt_m = -z_ned   # NED → metre (pozitif = yukarı)
        lbl   = f" {uid.upper()}\n ↑{alt_m:.0f}m"
        p.setPen(col)
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        p.drawText(QPointF(pos.x() + 12, pos.y() - 4), uid.upper())
        p.setFont(QFont("Consolas", 7))
        dim_col = QColor(col); dim_col.setAlpha(160)
        p.setPen(dim_col)
        p.drawText(QPointF(pos.x() + 12, pos.y() + 9), f"↑{alt_m:.0f}m")

        if stale:
            p.setPen(QColor(C_DANGER))
            p.setFont(QFont("Consolas", 7))
            p.drawText(QPointF(pos.x() + 12, pos.y() + 20), "NO SIG")


# ═══════════════════════════════════════════════════════════════════════════════
#  ANA PENCERE
# ═══════════════════════════════════════════════════════════════════════════════
class GcsMainWindow(QMainWindow):

    _COLS = ["İHA", "DURUM", "X (m)", "Y (m)", "İRTİFA (m)", "HIZ (m/s)", "YAW (°)"]
    _UAVS = [
        ("uav0", "👑  UAV0  Lider"),
        ("uav1", "✈️  UAV1"),
        ("uav2", "✈️  UAV2"),
    ]

    def __init__(self, node: GcsRosNode, bridge: RosBridge):
        super().__init__()
        self._node   = node
        self._bridge = bridge
        self._data: dict[str, dict] = {
            uid: {"x":0,"y":0,"z":0,"spd":0,"hdg":0,"ts":0.0}
            for uid, _ in self._UAVS
        }
        self._row_idx = {uid: i for i, (uid, _) in enumerate(self._UAVS)}

        self._build_ui()
        self._connect()

        self._tbl_timer = QTimer(self)
        self._tbl_timer.timeout.connect(self._refresh_table)
        self._tbl_timer.start(400)

        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start(1000)
        self._tick_clock()

    # ── UI İnşası ──────────────────────────────────────────────────────────
    def _build_ui(self):
        self.setWindowTitle("Sürü İHA Yer İstasyonu — Taktik Ekran")
        self.setMinimumSize(1240, 720)
        self.resize(1440, 860)
        self.setStyleSheet(STYLESHEET)

        root = QWidget(); self.setCentralWidget(root)
        vroot = QVBoxLayout(root)
        vroot.setContentsMargins(10, 8, 10, 4)
        vroot.setSpacing(6)

        vroot.addWidget(self._make_header())
        vroot.addWidget(self._make_hline())

        # ─── Yatay Splitter (Sol | Orta | Sağ) ───────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(4)
        splitter.addWidget(self._make_left_panel())

        # Radar + Tablo dikey splitter
        v_splitter = QSplitter(Qt.Vertical)
        v_splitter.setHandleWidth(4)
        self._radar = RadarWidget()
        v_splitter.addWidget(self._wrap_group("◈  TAKTİK RADAR", self._radar, pad=4))
        v_splitter.addWidget(self._make_table_panel())
        v_splitter.setStretchFactor(0, 3)
        v_splitter.setStretchFactor(1, 2)

        splitter.addWidget(v_splitter)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 4)

        vroot.addWidget(splitter, stretch=1)

        vroot.addWidget(self._make_hline())
        vroot.addWidget(self._make_target_panel())

        # Status bar
        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("🟡  ROS 2 başlatılıyor...")

    # ── Başlık ─────────────────────────────────────────────────────────────
    def _make_header(self) -> QWidget:
        w = QWidget()
        lay = QHBoxLayout(w); lay.setContentsMargins(0,0,0,0)

        titles = QVBoxLayout()
        t = QLabel("◈  SÜRÜ İHA YER İSTASYONU")
        t.setStyleSheet(f"color:{C_ACCENT}; font-size:20px; font-weight:bold; letter-spacing:3px;")
        sub = QLabel("TACTICAL GROUND CONTROL STATION  ·  ROS 2 + PX4  ·  SWARM MULTI-UAV")
        sub.setStyleSheet(f"color:{C_DIM}; font-size:10px; letter-spacing:1px;")
        titles.addWidget(t); titles.addWidget(sub)
        lay.addLayout(titles); lay.addStretch()

        self._clock_lbl = QLabel()
        self._clock_lbl.setStyleSheet(
            f"color:{C_ACCENT}; font-family:'Consolas'; font-size:16px; letter-spacing:3px;"
        )
        lay.addWidget(self._clock_lbl)
        return w

    # ── Sol Panel ──────────────────────────────────────────────────────────
    def _make_left_panel(self) -> QGroupBox:
        grp = QGroupBox("⬡  FORMASYON KOMUTLARI")
        lay = QVBoxLayout(grp); lay.setSpacing(7)

        btns = [
            ("🦅  V_SHAPE",        "V_SHAPE",       None),
            ("📏  LINE",           "LINE",           None),
            ("🏹  ARROW",          "ARROW",          None),
            ("↗  ECHELON RIGHT",   "ECHELON_RIGHT",  None),
            ("↖  ECHELON LEFT",    "ECHELON_LEFT",   None),
            ("🚶  TRAIL (Kolon)",   "TRAIL",          None),
        ]
        for label, cmd, _ in btns:
            b = QPushButton(label)
            b.setMinimumHeight(44)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            b.clicked.connect(lambda _, c=cmd: self._node.send_formation(c))
            lay.addWidget(b)

        lay.addWidget(self._make_hline())

        b_rally = QPushButton("🏠  RALLY — ÜSSE DÖN")
        b_rally.setObjectName("rally")
        b_rally.setMinimumHeight(46)
        b_rally.clicked.connect(lambda: self._node.send_formation("RALLY"))
        lay.addWidget(b_rally)

        b_hold = QPushButton("🚨  HOLD — ACİL DURDUR")
        b_hold.setObjectName("hold")
        b_hold.setMinimumHeight(46)
        b_hold.clicked.connect(lambda: self._node.send_formation("HOLD"))
        lay.addWidget(b_hold)

        lay.addStretch()

        self._active_lbl = QLabel("Aktif komut: —")
        self._active_lbl.setStyleSheet(
            f"color:{C_ACCENT}; font-family:'Consolas'; font-size:11px;"
        )
        self._active_lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._active_lbl)
        return grp

    # ── Tablo Paneli ───────────────────────────────────────────────────────
    def _make_table_panel(self) -> QGroupBox:
        grp = QGroupBox("📡  CANLI TELEMETRİ  —  NED FRAME")
        lay = QVBoxLayout(grp); lay.setContentsMargins(6,6,6,6)

        self._table = QTableWidget(len(self._UAVS), len(self._COLS))
        self._table.setHorizontalHeaderLabels(self._COLS)
        self._table.verticalHeader().setVisible(False)
        self._table.setAlternatingRowColors(True)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.NoSelection)
        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        for i in range(2, len(self._COLS)):
            hdr.setSectionResizeMode(i, QHeaderView.Stretch)
        self._table.verticalHeader().setDefaultSectionSize(38)

        for uid, label in self._UAVS:
            ri = self._row_idx[uid]
            self._table.setItem(ri, 0, self._cell(label, bold=True,
                                                   color=UAV_COLORS.get(uid)))
            for ci in range(1, len(self._COLS)):
                self._table.setItem(ri, ci, self._cell("—"))

        lay.addWidget(self._table)
        note = QLabel("⚡ 2.5 Hz  ·  NED: Negatif Z = Yüksek")
        note.setStyleSheet(f"color:{C_DIM}; font-size:10px; padding:2px 0;")
        lay.addWidget(note)
        return grp

    # ── Hedef Atama ────────────────────────────────────────────────────────
    def _make_target_panel(self) -> QGroupBox:
        grp = QGroupBox("🎯  GÖREV HEDEFİ ATAMA")
        lay = QHBoxLayout(grp); lay.setSpacing(10)

        lay.addWidget(QLabel("Payload:"))
        self._target_edit = QLineEdit()
        self._target_edit.setPlaceholderText(
            "uav1:500,200   veya   uav1:500,200|uav2:-300,400"
        )
        self._target_edit.setMinimumWidth(380)
        self._target_edit.returnPressed.connect(self._send_target)
        lay.addWidget(self._target_edit, stretch=3)

        b = QPushButton("🎯  HEDEF ATA")
        b.setObjectName("attack")
        b.setMinimumHeight(40)
        b.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        b.clicked.connect(self._send_target)
        lay.addWidget(b)

        lay.addStretch()
        self._last_target_lbl = QLabel("Son: —")
        self._last_target_lbl.setStyleSheet(
            f"color:{C_WARN}; font-family:'Consolas'; font-size:11px;"
        )
        lay.addWidget(self._last_target_lbl)
        return grp

    # ── Yardımcı Widget'lar ────────────────────────────────────────────────
    def _make_hline(self) -> QFrame:
        l = QFrame(); l.setFrameShape(QFrame.HLine)
        l.setStyleSheet(f"background:{C_BORDER};")
        l.setFixedHeight(1); return l

    def _wrap_group(self, title: str, widget: QWidget, pad: int = 8) -> QGroupBox:
        g = QGroupBox(title)
        v = QVBoxLayout(g); v.setContentsMargins(pad,pad,pad,pad)
        v.addWidget(widget); return g

    def _cell(self, text: str = "—", bold: bool = False,
              color: str = None, align=Qt.AlignCenter) -> QTableWidgetItem:
        it = QTableWidgetItem(text)
        it.setTextAlignment(align | Qt.AlignVCenter)
        if bold:
            f = it.font(); f.setBold(True); it.setFont(f)
        if color:
            it.setForeground(QColor(color))
        return it

    # ── Sinyal Bağlantıları ─────────────────────────────────────────────────
    def _connect(self):
        self._bridge.telemetry.connect(self._on_telemetry)
        self._bridge.status_msg.connect(self._status.showMessage)

    def _on_telemetry(self, uid: str, x: float, y: float,
                      z: float, spd: float, hdg: float):
        if uid in self._data:
            self._data[uid] = {"x":x,"y":y,"z":z,"spd":spd,"hdg":hdg,"ts":time.monotonic()}
        self._radar.update_uav(uid, x, y, z, spd, hdg)

    # ── Tablo Yenile ─────────────────────────────────────────────────────
    def _refresh_table(self):
        now = time.monotonic()
        for uid, lbl in self._UAVS:
            d  = self._data[uid]
            ri = self._row_idx[uid]
            stale = (now - d["ts"]) > 3.0 if d["ts"] > 0 else True

            if stale:
                self._table.setItem(ri, 1, self._cell("🔴 BAĞLANTI YOK", color=C_DANGER))
                for ci in range(2, len(self._COLS)):
                    self._table.setItem(ri, ci, self._cell("—", color=C_DIM))
                continue

            if d["spd"] > 1.5:
                st, sc = "🟢 UÇUYOR",   C_SUCCESS
            else:
                st, sc = "🟡 BEKLEMEDE", C_WARN
            self._table.setItem(ri, 1, self._cell(st, color=sc))

            alt = -d["z"]
            vals = [
                (f'{d["x"]:+.2f}',        C_TEXT),
                (f'{d["y"]:+.2f}',        C_TEXT),
                (f'{alt:+.2f}',           C_ACCENT if alt > 5 else C_WARN),
                (f'{d["spd"]:.2f}',        C_BLUE),
                (f'{math.degrees(d["hdg"]):.1f}°', C_TEXT),
            ]
            for ci, (v, col) in enumerate(vals, 2):
                self._table.setItem(ri, ci, self._cell(v, color=col))

    # ── Hedef Gönder ───────────────────────────────────────────────────────
    def _send_target(self):
        payload = self._target_edit.text().strip()
        if not payload:
            self._status.showMessage("⚠️  Hedef boş!"); return
        self._node.send_target(payload)
        self._last_target_lbl.setText(f"Son: {payload}")
        self._target_edit.clear()

    # ── Saat ───────────────────────────────────────────────────────────────
    def _tick_clock(self):
        self._clock_lbl.setText("🕐 " + datetime.datetime.now().strftime("%H:%M:%S"))

    def closeEvent(self, e):
        rclpy.shutdown(); e.accept()


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    bridge = RosBridge()
    node   = GcsRosNode(bridge)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QPalette()
    for role, col in [
        (QPalette.Window,        C_BG),
        (QPalette.WindowText,    C_TEXT),
        (QPalette.Base,          C_CARD),
        (QPalette.AlternateBase, C_PANEL),
        (QPalette.Text,          C_TEXT),
        (QPalette.Button,        C_CARD),
        (QPalette.ButtonText,    C_TEXT),
        (QPalette.Highlight,     C_ACCENT),
        (QPalette.HighlightedText, C_BG),
    ]:
        pal.setColor(role, QColor(col))
    app.setPalette(pal)

    win = GcsMainWindow(node, bridge)
    win.show()

    ros_thread = RosThread(node)
    ros_thread.start()

    code = app.exec_()
    ros_thread.quit()
    ros_thread.wait()
    node.destroy_node()
    sys.exit(code)


if __name__ == "__main__":
    main()
