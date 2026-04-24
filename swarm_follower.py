#!/usr/bin/env python3
"""
swarm_follower.py
==================
Sürü İHA V2 - Lider-Takipçi (Leader-Follower) Takipçi Düğümü
Sistem: ROS 2 + PX4 (MicroXRCEAgent DDS)

Mimari: GPS/Telemetri ONLY (Kamera/YOLO yok)
Güdüm: L1 Pure Pursuit + CTRV Öngörü + TECS Enerji Yönetimi

Parametre Olarak Çalıştırma:
    python3 swarm_follower.py --ros-args -p leader_id:=uav0 -p my_id:=uav1

NOTLAR:
  - PX4 v1.14 DDS topic formatı: /px4_{instance}/fmu/...
  - 'uav0' için instance=0 → topic: /fmu/... (prefix'siz)
  - 'uav1' için instance=1 → topic: /px4_1/fmu/...
  - Bu düğüm YALNIZCA kendi komut topiclerine yazar, liderin topiclerine DOKUNMAZ.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import math
import sys
import time

from px4_msgs.msg import (
    VehicleLocalPosition,
    OffboardControlMode,
    VehicleAttitudeSetpoint,
)
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────────────────────
# Yardımcı Fonksiyonlar
# ─────────────────────────────────────────────────────────────────────────────

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Euler açılarını (radyan) quaternion [w, x, y, z] formatına çevirir."""
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]


def normalize_angle(angle: float) -> float:
    """Aci -pi..+pi'ye sarmalar. atan2(sin,cos) ile wrapping hatasi olmaz."""
    return math.atan2(math.sin(angle), math.cos(angle))


def build_topic(uav_id: str, suffix: str) -> str:
    """
    PX4 DDS multi-vehicle topic adını otomatik oluşturur.
      uav0  → /fmu/<suffix>          (instance 0, prefix'siz)
      uav1  → /px4_1/fmu/<suffix>    (instance 1)
      uav2  → /px4_2/fmu/<suffix>    ...
    """
    if uav_id == 'uav0':
        return f'/fmu/{suffix}'
    else:
        instance = int(uav_id.replace('uav', ''))
        return f'/px4_{instance}/fmu/{suffix}'


# ─────────────────────────────────────────────────────────────────────────────
# Swarm Follower Düğümü
# ─────────────────────────────────────────────────────────────────────────────

class SwarmFollowerNode(Node):

    # ── Kontrol Kazançları ────────────────────────────────────────────────────
    KP_ROLL_GPS   = 1.0    # L1 lateral kazanç

    # ── Uçuş Limitleri ───────────────────────────────────────────────────────
    MAX_ROLL      = 0.75   # rad (~43°)
    MAX_PITCH_UP  = 0.35   # rad (~20°)
    MAX_PITCH_DOWN= -0.30  # rad (~-17°)
    MIN_THRUST    = 0.05
    MAX_THRUST    = 0.85

    # ── Sürü Parametreleri ───────────────────────────────────────────────────
    TRAIL_DISTANCE  = 30.0  # metre - Liderden tutunulacak kuyruk mesafesi
    LOOK_AHEAD_TIME = 0.5   # saniye - CTRV ileri tahmin penceresi

    # ── Koordinat Çevirici: Simülasyondaki Spawn/Doğuş Ofsetleri ──────────────────────
    # Her İHA kendi yerel NED (0,0)'1ndan başlar.
    # Bu ofsetler her İHA'nın genç doğuş konumunu ortak global çerçevede belirtir.
    # Formu: { 'uav_id': (global_x, global_y) }
    SPAWN_OFFSETS = {
        'uav0': (   0.0,   0.0),
        'uav1': ( -15.0,  15.0),
        'uav2': ( -15.0, -15.0),
        'uav3': ( -15.0, -30.0),
    }

    # ── Çarpışma Önleyici (APF) ─────────────────────────────────────────────
    CRITICAL_RADIUS = 15.0  # metre - Güvenlik çemberi yarıçapı
    APF_GAIN        = 3.0   # İtici kuvvet kazançı (ne kadar sert kaçılsın)
    APF_WEIGHT      = 0.65  # Kaçış/izleme açı birleştirme ağırlığı (0..1)
    APF_THR_IDLE    = 0.12  # Anti bang-bang: sert kesme yerine rölanti thrust

    # ── Formasyon Geometri Tablosu ───────────────────────────────────────────────
    # Lider gövde eksenine göre (X: ileri/geri, Y: sağ/sol)
    # Anahtar: follower index (1=uav1, 2=uav2, ...)
    # Bilinmeyen index için son satır çarpı index ile genüş yayılır.
    FORMATION_TABLE = {
        'V_SHAPE': {
            1: (-30,  30),
            2: (-30, -30),
            3: (-60,  60),
            4: (-60, -60),
            5: (-90,  90),
        },
        'LINE': {
            1: ( 0,  30),
            2: ( 0, -30),
            3: ( 0,  60),
            4: ( 0, -60),
            5: ( 0,  90),
        },
        'ARROW': {
            1: (-30,  20),
            2: (-30, -20),
            3: (-15,  40),
            4: (-15, -40),
            5: (  0,  60),
        },
        'ECHELON_RIGHT': {
            1: (-30,  30),
            2: (-60,  60),
            3: (-90,  90),
            4: (-120, 120),
            5: (-150, 150),
        },
        'ECHELON_LEFT': {
            1: (-30, -30),
            2: (-60, -60),
            3: (-90, -90),
            4: (-120, -120),
            5: (-150, -150),
        },
        'TRAIL': {
            1: (-30,  0),
            2: (-60,  0),
            3: (-90,  0),
            4: (-120, 0),
            5: (-150, 0),
        },
        'HOLD': {},  # HOLD: mevcut ofset korunur
    }

    # ── Log Periyodu ─────────────────────────────────────────────────────────
    LOG_PERIOD = 0.5        # saniye (2 Hz ekran çıktısı)

    def __init__(self):
        super().__init__('swarm_follower_node')

        # ─────────────────────────────────────────────────────────────────────
        # ROS 2 Parametreleri (leader_id ve my_id dinamik olarak okunur)
        # ─────────────────────────────────────────────────────────────────────
        self.declare_parameter('leader_id',          'uav0')
        self.declare_parameter('my_id',              'uav1')
        # Formasyon ofseti (Lider gövde eksenine göre metre)
        # Varsayılan: V-Formasyonu sağ kanadı  (X: -30 → Arkada, Y: +30 → Sağda)
        self.declare_parameter('formation_offset_x', -30.0)
        self.declare_parameter('formation_offset_y',  30.0)
        # Sürü arkadaşları (APF için – virgillü liste olarak ver)
        # Örn: --ros-args -p swarm_mates:="uav2,uav3"
        self.declare_parameter('swarm_mates', '')

        self.leader_id          = self.get_parameter('leader_id').get_parameter_value().string_value
        self.my_id              = self.get_parameter('my_id').get_parameter_value().string_value
        self.formation_offset_x = self.get_parameter('formation_offset_x').get_parameter_value().double_value
        self.formation_offset_y = self.get_parameter('formation_offset_y').get_parameter_value().double_value

        mates_raw = self.get_parameter('swarm_mates').get_parameter_value().string_value
        # Boş string'i filtrele, yoksa boş liste
        self.swarm_mates = [m.strip() for m in mates_raw.split(',') if m.strip()]

        self.get_logger().info(
            f'=== SwarmFollowerNode Başlatıldı | Ben: {self.my_id} | Lider: {self.leader_id} ==='
        )
        self.get_logger().info(
            f'  Formasyon Ofseti → X: {self.formation_offset_x:.1f}m  Y: {self.formation_offset_y:.1f}m '
            f'(+ Y = Sağ kanat, - Y = Sol kanat)'
        )
        self.get_logger().info(
            f'  Sürü Arkadaşları (APF): {self.swarm_mates if self.swarm_mates else "ÖNCELİKLİ YOK"}'
        )

        # ─────────────────────────────────────────────────────────────────────
        # QoS (Sürü İçi Haberleşme - Best-Effort / Volatile)
        # ─────────────────────────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        # ─────────────────────────────────────────────────────────────────────
        # Durum Değişkenleri - Takipçi (Self)
        # ─────────────────────────────────────────────────────────────────────
        self.my_x    = 0.0
        self.my_y    = 0.0
        self.my_z    = 0.0
        self.my_yaw  = 0.0
        self.my_vx   = 0.0
        self.my_vy   = 0.0

        # ─────────────────────────────────────────────────────────────────────
        # Durum Değişkenleri - Lider
        # ─────────────────────────────────────────────────────────────────────
        self.leader_x   = 0.0
        self.leader_y   = 0.0
        self.leader_z   = 0.0
        self.leader_yaw = 0.0
        self.leader_vx  = 0.0
        self.leader_vy  = 0.0

        # CTRV Öngörü (Yaw Rate Tahmin)
        self.prev_leader_yaw  = 0.0
        self.prev_leader_time = 0.0
        self.leader_yaw_rate  = 0.0
        self.yaw_rate_ema_alpha = 0.25

        # ── V4: Heartbeat Takibi ve Hata Toleransı ──────────────────────────────────
        self.last_leader_msg_time = time.time()  # Başlangıçta "az önce" alındı say
        self.leader_alive  = True                # Lider sağlıklı mı?
        self.is_leader     = False               # Ben yeni lider miyim?
        self.leader_sub    = None               # Aboneliği yeniden bağlamak için referans

        # ── V5: Görev Koordinasyonu (INTERCEPT Modu) ──────────────────────────────
        self.mission_mode        = 'FORMATION'  # 'FORMATION' | 'INTERCEPT'
        self.assigned_target_x   = 0.0
        self.assigned_target_y   = 0.0

        # ─────────────────────────────────────────────────────────────────────
        # Durum Sözlüğü: Diğer Sürü Arkadaşlarının Pozisyonları (APF)
        # { 'uav2': (x, y, z), 'uav3': (x, y, z), ... }
        # ─────────────────────────────────────────────────────────────────────
        self.mates_pos: dict = {}

        # Aktif Formasyon (GCS'den gelir, başlangıçta parametre değeri)
        self.active_formation = 'PARAM_DEFAULT'

        # ─────────────────────────────────────────────────────────────────────
        # Bumpless Transfer (Yumuşak Çıkış EMA)
        # ─────────────────────────────────────────────────────────────────────
        self.current_roll_out  = 0.0
        self.current_pitch_out = 0.0

        self._last_log_time = 0.0

        # ─────────────────────────────────────────────────────────────────────
        # Topic Adlarını Dinamik Oluştur
        # ─────────────────────────────────────────────────────────────────────
        leader_pos_topic = build_topic(self.leader_id, 'out/vehicle_local_position_v1')
        my_pos_topic     = build_topic(self.my_id,     'out/vehicle_local_position_v1')
        my_offboard_topic= build_topic(self.my_id,     'in/offboard_control_mode')
        my_att_sp_topic  = build_topic(self.my_id,     'in/vehicle_attitude_setpoint_v1')

        self.get_logger().info(f'  Lider Pos   : {leader_pos_topic}')
        self.get_logger().info(f'  Benim Pos   : {my_pos_topic}')
        self.get_logger().info(f'  Offboard Out: {my_offboard_topic}')
        self.get_logger().info(f'  Attitude Out: {my_att_sp_topic}')

        # ─────────────────────────────────────────────────────────────────────
        # Abonelikler
        # ─────────────────────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            my_pos_topic,
            self._cb_my_pos,
            qos_sensor,
        )

        self.leader_sub = self.create_subscription(
            VehicleLocalPosition,
            leader_pos_topic,
            self._cb_leader_pos,
            qos_sensor,
        )

        # ─────────────────────────────────────────────────────────────────────
        # APF: Diğer Sürü Arkadaşları için Dinamik Abonelikler
        # ─────────────────────────────────────────────────────────────────────
        for mate_id in self.swarm_mates:
            mate_topic = build_topic(mate_id, 'out/vehicle_local_position_v1')
            self.get_logger().info(f'  APF Abonelik: {mate_topic} ({mate_id})')
            # closure ile mate_id'yi yakala
            self.create_subscription(
                VehicleLocalPosition,
                mate_topic,
                self._make_mate_cb(mate_id),
                qos_sensor,
            )

        # ─────────────────────────────────────────────────────────────────────
        # GCS Komuta Topic Aboneliği (RELIABLE + TRANSIENT_LOCAL)
        # ─────────────────────────────────────────────────────────────────────
        from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy
        qos_gcs = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self.create_subscription(
            String,
            '/swarm/formation_cmd',
            self._cb_formation_cmd,
            qos_gcs,
        )
        self.get_logger().info('  GCS Abonelik: /swarm/formation_cmd [RELIABLE+TRANSIENT_LOCAL]')

        # Hedef Atama Aboneliği (V5 - INTERCEPT Modu)
        self.create_subscription(
            String,
            '/swarm/target_assignments',
            self._cb_target_assignment,
            qos_gcs,
        )
        self.get_logger().info('  GCS Abonelik: /swarm/target_assignments [RELIABLE+TRANSIENT_LOCAL]')

        # ─────────────────────────────────────────────────────────────────────
        # Yayıncılar (Sadece KENDİ komut topiclerine yazılır)
        # ─────────────────────────────────────────────────────────────────────
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            my_offboard_topic,
            qos_sensor,
        )

        self.pub_attitude_sp = self.create_publisher(
            VehicleAttitudeSetpoint,
            my_att_sp_topic,
            qos_sensor,
        )

        # 20 Hz kontrol döngüsü
        self.create_timer(1.0 / 20.0, self._control_loop)

        # 1 Hz lider sağlık kontrolü (V4)
        self.create_timer(1.0, self._check_leader_health)

    # ─────────────────────────────────────────────────────────────────────────
    # Callback: Benim Pozisyonum
    # ─────────────────────────────────────────────────────────────────────────
    def _cb_my_pos(self, msg: VehicleLocalPosition):
        self.my_x   = msg.x
        self.my_y   = msg.y
        self.my_z   = msg.z
        self.my_yaw = msg.heading
        self.my_vx  = msg.vx
        self.my_vy  = msg.vy

    # ─────────────────────────────────────────────────────────────────────────
    # Yardimci: Güvenli Açı Sarmalayıcı (Instance Metodu)
    # atan2(sin, cos) → while-loop'tan çok daha güvenilir,
    # büyük açı değerlerinde Wrap hatasına düşmez.
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _to_my_frame(self, source_id: str, source_x: float, source_y: float) -> tuple:
        """
        Farklı İHA'dan gelen yerel NED koordinatını bizim yerel NED çerçevemize çevirir.

        Algoritma:
          Global = source_local + source_spawn
          My_NED = Global - my_spawn
                 = source_local + source_spawn - my_spawn
        """
        src_spawn = self.SPAWN_OFFSETS.get(source_id, (0.0, 0.0))
        my_spawn  = self.SPAWN_OFFSETS.get(self.my_id,  (0.0, 0.0))
        corrected_x = source_x + src_spawn[0] - my_spawn[0]
        corrected_y = source_y + src_spawn[1] - my_spawn[1]
        return corrected_x, corrected_y

    # ─────────────────────────────────────────────────────────────────────────
    # Callback: GCS Formasyon Emri
    # ─────────────────────────────────────────────────────────────────────────
    def _cb_formation_cmd(self, msg: String):
        formation = msg.data.strip().upper()
        if formation == self.active_formation:
            return  # Aynı komutu tekrar işleme
        self.get_logger().info(f'[GCS] Formasyon Emri Alındı: "{formation}"')
        self._update_formation_offsets(formation)

    # ─────────────────────────────────────────────────────────────────────────
    # V5: Callback - Hedef Atama ( /swarm/target_assignments )
    # Format: "uav1:500.0,200.0|uav2:-300.0,400.0"
    # ─────────────────────────────────────────────────────────────────────────
    def _cb_target_assignment(self, msg: String):
        raw = msg.data.strip()
        for part in raw.split('|'):
            part = part.strip()
            if ':' not in part:
                continue
            uav_id, coords = part.split(':', 1)
            if uav_id.strip() != self.my_id:
                continue  # Bu mesaj bana değil
            try:
                x_str, y_str = coords.split(',')
                self.assigned_target_x = float(x_str.strip())
                self.assigned_target_y = float(y_str.strip())
                self.mission_mode = 'INTERCEPT'
                self.get_logger().warn(
                    f'[{self.my_id}] 🎯 GÖREV ALINDI: INTERCEPT '
                    f'→ Hedef ({self.assigned_target_x:.1f}, {self.assigned_target_y:.1f}) '
                    f'| Formasyon devre dışı!'
                )
            except (ValueError, IndexError) as e:
                self.get_logger().error(f'Hedef koordinatı parse edilemedi: "{coords}" → {e}')

    # ─────────────────────────────────────────────────────────────────────────
    # Dinamik Formasyon Offset Hesaplayıcı
    # my_id numarasına ve formasyon tablosuna göre offset'leri günceller.
    # ─────────────────────────────────────────────────────────────────────────
    def _update_formation_offsets(self, formation: str):
        # V7: RALLY — tüm modları geçersiz kıl, misyon modunu RALLY'e al
        if formation == 'RALLY':
            self.mission_mode    = 'RALLY'
            self.active_formation = 'RALLY'
            self.get_logger().warn(
                f'\033[95m\033[1m'
                f'[{self.my_id}] 🏠 RALLY KOMUTU ALINDI! '
                f'Lider/Formasyon/Hedef UNUTULDU → (0,0) Orbit noktasına uçuluyor.'
                f'\033[0m'
            )
            return

        # RALLY temizliği: RALLY dışı bir komut geldiğinde tekrar FORMATION'a dön.
        if self.mission_mode == 'RALLY':
            self.mission_mode = 'FORMATION'
            self.get_logger().info(
                f'[{self.my_id}] RALLY sonlandırıldı → FORMATION moduna dönüldü.'
            )

        if formation == 'HOLD':
            # Mevcut ofseti koru, sadece formasyon adını güncelle
            self.active_formation = 'HOLD'
            self.get_logger().warn(
                f'[{self.my_id}] HOLD: Ofset şu anki değerde dondu '
                f'(x={self.formation_offset_x:.1f}, y={self.formation_offset_y:.1f})'
            )
            return

        if formation not in self.FORMATION_TABLE:
            self.get_logger().warn(f'Bilinmeyen formasyon: "{formation}", görmezden gelindi.')
            return

        # uav1 → index=1, uav2 → index=2 ...
        try:
            idx = int(self.my_id.replace('uav', ''))
        except ValueError:
            self.get_logger().error(f'my_id parse edilemedi: {self.my_id}')
            return

        table = self.FORMATION_TABLE[formation]

        if idx in table:
            ox, oy = table[idx]
        else:
            # Tabloda tanımlı olmayan index: en son satırın X'ini artır
            max_idx = max(table.keys())
            base_x, base_y = table[max_idx]
            extra  = idx - max_idx
            sign_y = 1 if base_y >= 0 else -1
            ox = base_x - (extra * 30)
            oy = base_y + (extra * 30 * sign_y)

        self.formation_offset_x = float(ox)
        self.formation_offset_y = float(oy)
        self.active_formation   = formation

        self.get_logger().info(
            f'[{self.my_id}] Formasyon Güncellendi: {formation} '
            f'→ offset(x={ox}, y={oy})'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callback Fabrikası: Swarm Arkadaş Pozisyonları (APF için)
    # ─────────────────────────────────────────────────────────────────────────
    def _make_mate_cb(self, mate_id: str):
        """Her arkadaş için ayrı bir callback üretir (closure pattern)."""
        def _cb(msg: VehicleLocalPosition):
            # Koordinat çevirisi: arkadaşın yerel NED'ini bizim frame'imize al
            cx, cy = self._to_my_frame(mate_id, msg.x, msg.y)
            self.mates_pos[mate_id] = (cx, cy, msg.z)
        return _cb

    # ─────────────────────────────────────────────────────────────────────────
    # Callback: Liderin Pozisyonu (CTRV Yaw Rate hesaplaması + Heartbeat damgalama)
    # ─────────────────────────────────────────────────────────────────────────
    def _cb_leader_pos(self, msg: VehicleLocalPosition):
        now = time.monotonic()

        # V4: Heartbeat zaman damgası
        self.last_leader_msg_time = time.time()
        self.leader_alive = True

        # Koordinat çevirisi: liderin yerel NED'ini bizim frame'imize al
        cx, cy = self._to_my_frame(self.leader_id, msg.x, msg.y)

        self.leader_x   = cx
        self.leader_y   = cy
        self.leader_z   = msg.z
        self.leader_vx  = msg.vx
        self.leader_vy  = msg.vy

        yaw = msg.heading

        # Liderin Yaw Rate (Dönüş Hızı) Türevi
        if self.prev_leader_time != 0.0:
            dt = now - self.prev_leader_time
            if dt > 0.0:
                dyaw = normalize_angle(yaw - self.prev_leader_yaw)
                raw_yaw_rate = dyaw / dt
                alpha = self.yaw_rate_ema_alpha
                self.leader_yaw_rate = ((1.0 - alpha) * self.leader_yaw_rate) + (alpha * raw_yaw_rate)

        self.prev_leader_yaw  = yaw
        self.prev_leader_time = now
        self.leader_yaw       = yaw

    # ─────────────────────────────────────────────────────────────────────────
    # V4: Lider Sağlık Kontrolücüsü (1 Hz Timer)
    # ─────────────────────────────────────────────────────────────────────────
    def _check_leader_health(self):
        # Eğer zaten lider seyim veya başlangıç greyasyonu sürmediyse kontrol etme
        if self.is_leader:
            return

        silence = time.time() - self.last_leader_msg_time
        if silence <= 3.0:
            return  # Lider yaşıyor

        # ─ 3 saniye sessizlik: Protokol Başlıyor ─
        if self.leader_alive:  # Sadece bir kez yaz
            self.leader_alive = False
            self.get_logger().warn(
                f'[91m[1m'
                f'\U0001f6a8 LİDER DÜŞTU! ({self.leader_id}) '
                f'HATA TOLERANS PROTOKOLÜ BAŞLA TILIYOR... '
                f'(Sessizlik: {silence:.1f}s)'
                f'\033[0m'
            )

        if self.my_id == 'uav1':
            # ─ uav1 için Terfi (Promotion): Yeni Lider Ben Oluyorum ─
            self.is_leader = True
            # Önceki görev/mode kalıntılarını temizle
            self.mission_mode = 'FORMATION'
            self.assigned_target_x = 0.0
            self.assigned_target_y = 0.0
            self.prev_leader_yaw = 0.0
            self.prev_leader_time = 0.0
            self.leader_yaw_rate = 0.0
            self.get_logger().warn(
                f'[92m[1m'
                f'\U0001f451 YENİ LİDER BENİM! ({self.my_id}) '
                f'L1 Formasyon devre dışı — Sabit Seyir (Loiter) moduna geçildi.'
                f'\033[0m'
            )
        else:
            # ─ Diğer Takipçiler: Yeni Lider = uav1 ─
            new_leader = 'uav1'
            new_topic  = build_topic(new_leader, 'out/vehicle_local_position_v1')

            # Eski lider aboneliğini iptal et
            if self.leader_sub is not None:
                self.destroy_subscription(self.leader_sub)
                self.leader_sub = None

            qos_sensor = QoSProfile(
                reliability = ReliabilityPolicy.BEST_EFFORT,
                durability  = DurabilityPolicy.VOLATILE,
                history     = HistoryPolicy.KEEP_LAST,
                depth       = 1,
            )
            self.leader_sub = self.create_subscription(
                VehicleLocalPosition,
                new_topic,
                self._cb_leader_pos,
                qos_sensor,
            )
            self.leader_id               = new_leader
            self.last_leader_msg_time    = time.time()  # Zaman damgasını sıfırla
            self.leader_alive            = True

            self.get_logger().warn(
                f'[93m[1m'
                f'➡️  YENI LİDERE BİRAKTİM: {new_leader} '
                f'({new_topic})'
                f'\033[0m'
            )

    # ─────────────────────────────────────────────────────────────────────────
    # TECS: Thrust ve Pitch Hesaplama
    # (Lider referanslı, orijinal tracking_control.py mantığı korundu)
    # ─────────────────────────────────────────────────────────────────────────
    def _compute_thrust_and_pitch(self, trail_dist: float, roll: float, actual_dist: float) -> tuple:
        # NED: Z negatifse yüksek, pozitifse alçak
        alt_diff = self.my_z - self.leader_z

        # 1. İrtifa (Pitch) Kontrolcüsü
        kp_pitch   = 0.025
        pitch_alt  = alt_diff * kp_pitch
        pitch_comp = abs(roll) * 0.15  # Roll sırasında lift kaybı kompanzasyonu
        pitch      = pitch_alt + pitch_comp
        pitch      = float(np.clip(pitch, self.MAX_PITCH_DOWN, self.MAX_PITCH_UP))

        # 2. Hız Eşitleme (Cruise Matching) - Trail eşiğine göre nominal thrust
        threshold_dist = self.TRAIL_DISTANCE + 10.0
        if trail_dist <= threshold_dist:
            base_thrust = 0.42
        else:
            kp_dist     = 0.002
            base_thrust = 0.42 + ((trail_dist - threshold_dist) * kp_dist)

        # 3. Basitleştirilmiş TECS (İrtifa → Thrust Enerji Desteği)
        kp_alt_thrust = 0.005
        thrust        = base_thrust + (alt_diff * kp_alt_thrust)

        # 4. Dalış Önleyici (Dive Damper)
        if alt_diff < -5.0 and pitch < 0.0:
            thrust = self.MIN_THRUST

        # 5. Anti-Overshoot Hava Freni (Airbrake)
        my_speed     = math.sqrt(self.my_vx**2 + self.my_vy**2)
        leader_speed = math.sqrt(self.leader_vx**2 + self.leader_vy**2)
        my_speed_kmh = my_speed * 3.6

        if actual_dist < 60.0 and my_speed_kmh > 60.0:
            thrust = self.APF_THR_IDLE               # Yumuşak Hız Kesici (anti bang-bang)
        elif actual_dist < 60.0 and my_speed > leader_speed:
            thrust = max(self.APF_THR_IDLE, self.MIN_THRUST)  # Erken Süzülme (yumuşak)

        # Thrust güvenli dilime al
        thrust = float(np.clip(thrust, self.MIN_THRUST, self.MAX_THRUST))

        return thrust, pitch

    # ─────────────────────────────────────────────────────────────────────────
    # APF: İtici Kuvvet Vektörü (Repulsive Force)
    # Geri dönüş: (rep_angle, rep_magnitude)
    #   rep_angle     → kaçış yönünün dünya açısı (radyan)
    #   rep_magnitude → 0.0 – 1.0 arası normalize edilmiş baskı şiddeti
    # ─────────────────────────────────────────────────────────────────────────
    def _compute_repulsive_force(self) -> tuple:
        rep_fx = 0.0
        rep_fy = 0.0
        triggered = False

        for mate_id, (mx, my, _) in self.mates_pos.items():
            dx  = self.my_x - mx
            dy  = self.my_y - my
            d   = math.sqrt(dx**2 + dy**2)

            if 0.01 < d < self.CRITICAL_RADIUS:
                triggered = True
                # Mesafe azaldıkça kuvvet yükselen üstel itme
                magnitude = self.APF_GAIN * ((1.0 / d) - (1.0 / self.CRITICAL_RADIUS))
                rep_fx += magnitude * (dx / d)
                rep_fy += magnitude * (dy / d)

        if not triggered:
            return 0.0, 0.0

        rep_angle    = math.atan2(rep_fy, rep_fx)
        rep_strength = min(1.0, math.sqrt(rep_fx**2 + rep_fy**2))
        return rep_angle, rep_strength

    # ─────────────────────────────────────────────────────────────────────────
    # Ana Kontrol Döngüsü (20 Hz)
    # ─────────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        now    = time.monotonic()
        do_log = (now - self._last_log_time) >= self.LOG_PERIOD

        self._publish_offboard_mode()

        # Lider önceliği: Lider seçildiyse follower modlarına asla girme.
        if self.is_leader:
            roll   = 0.0
            pitch  = 0.0
            thrust = 0.42
            if do_log:
                self.get_logger().info(
                    f'\033[92m[👑 LİDER/{self.my_id}] Sabit Seyir Modu '
                    f'| Yaw: {math.degrees(self.my_yaw):.1f}° | Thr: {thrust:.2f}\033[0m'
                )
                self._last_log_time = now
            self._publish_attitude_setpoint(roll, pitch, self.my_yaw, thrust)
            return

        # ── V7: RALLY Modu (EN YÜKSEK ÖNCELİK – Her Şeyin Üstünde) ─────────
        # Lider, formasyon, atanmış hedef → TÜMÜ görmezden gelinir.
        # APF kalkanı KESİNLİKLE aktiftir (orbit sırasında çarpışma önleme).
        if self.mission_mode == 'RALLY':
            RALLY_X      = 0.0
            RALLY_Y      = 0.0
            ORBIT_RADIUS = 30.0   # metre – bu mesafenin içinde orbit başlar
            ORBIT_ROLL   = math.radians(25.0)  # ~25° sabit yatış → daire yörüngesi

            rdx    = RALLY_X - self.my_x
            rdy    = RALLY_Y - self.my_y
            r_dist = math.sqrt(rdx**2 + rdy**2)

            # APF: Orbit sırasında diğer İHA'lardan kaç (uydu halkası oluşsun)
            rep_angle, rep_strength = self._compute_repulsive_force()
            in_danger = rep_strength > 0.0

            if r_dist > ORBIT_RADIUS:
                # ── Yaklaşma Fazı: Noktaya Pure Pursuit ───────────────────
                r_course    = math.atan2(rdy, rdx)
                angle_error = self._normalize_angle(r_course - self.my_yaw)

                if in_danger:
                    target_err = angle_error
                    escape_err = self._normalize_angle(rep_angle - self.my_yaw)
                    angle_error = self._normalize_angle(
                        ((1.0 - self.APF_WEIGHT) * target_err) + (self.APF_WEIGHT * escape_err)
                    )

                roll   = float(np.clip(self.KP_ROLL_GPS * angle_error,
                                       -self.MAX_ROLL, self.MAX_ROLL))
                thrust = min(self.MAX_THRUST, 0.42 + (r_dist - ORBIT_RADIUS) * 0.002)
                pitch  = float(np.clip((self.my_z - 0.0) * 0.025,
                                       self.MAX_PITCH_DOWN, self.MAX_PITCH_UP))
                if do_log:
                    self.get_logger().warn(
                        f'\033[95m[{self.my_id}→RALLY-YAKLAŞMA] '
                        f'Mesafe: {r_dist:.1f}m | '
                        f'Roll: {math.degrees(roll):+.1f}° | Thr: {thrust:.2f}\033[0m'
                    )
            else:
                # ── Orbit Fazı: Noktanın etrafında sabit yatışla daire ────
                # APF sürüdeki diğer İHA'ları iterek hepsini farklı yörünge
                # açılarına dağıtır → "kusursuz uydu halkası" oluşur.
                if in_danger:
                    # APF aktifse önce kaç, sonra tekrar orbit'e döner
                    escape_err = self._normalize_angle(rep_angle - self.my_yaw)
                    roll   = float(np.clip(self.KP_ROLL_GPS * escape_err,
                                           -self.MAX_ROLL, self.MAX_ROLL))
                else:
                    roll = ORBIT_ROLL   # Saat yönünde sabit daire

                thrust = 0.42
                pitch  = 0.0
                if do_log:
                    self.get_logger().warn(
                        f'\033[95m[{self.my_id}→RALLY-ORBİT] '
                        f'R: {r_dist:.1f}m | '
                        f'Roll: {math.degrees(roll):+.1f}° | Thr: {thrust:.2f} '
                        f'| APF: {"AKTİF" if in_danger else "pasif"}\033[0m'
                    )

            if do_log:
                self._last_log_time = now
            self._publish_attitude_setpoint(roll, pitch, self.my_yaw, thrust)
            return

        # ── V5: INTERCEPT Modu (en yüksek öncelik – V4'ten önce) ──────────
        if self.mission_mode == 'INTERCEPT':
            itx = self.assigned_target_x
            ity = self.assigned_target_y
            tdx = itx - self.my_x
            tdy = ity - self.my_y
            t_dist = math.sqrt(tdx**2 + tdy**2)

            # Hedefe Pure Pursuit L1
            t_course    = math.atan2(tdy, tdx)
            look_at_err = self._normalize_angle(t_course - self.my_yaw)
            angle_error = look_at_err  # Sadece hedefe bak

            # APF: Çarpışma önleyici hâlâ aktif
            rep_angle, rep_strength = self._compute_repulsive_force()
            if rep_strength > 0.0:
                escape_err  = self._normalize_angle(rep_angle - self.my_yaw)
                angle_error = self._normalize_angle(
                    ((1.0 - self.APF_WEIGHT) * angle_error) + (self.APF_WEIGHT * escape_err)
                )

            roll  = self.KP_ROLL_GPS * angle_error
            roll  = float(np.clip(roll, -self.MAX_ROLL, self.MAX_ROLL))

            # Basit TECS: mesafeye göre gaz – yakınlaşınca süzül
            if t_dist < 20.0:
                self.mission_mode = 'FORMATION'  # Hedefe ulaştı, formasyona dön
                self.get_logger().info(f'[{self.my_id}] ✅ HEDEFE ULAŞILDI! Formasyon moduna dönülüyor.')
                thrust = self.MIN_THRUST
            elif t_dist < 60.0:
                thrust = 0.42
            else:
                thrust = min(self.MAX_THRUST, 0.42 + (t_dist - 60.0) * 0.002)

            pitch = float(np.clip((self.my_z - self.leader_z) * 0.025,
                                  self.MAX_PITCH_DOWN, self.MAX_PITCH_UP))

            if do_log:
                self.get_logger().warn(
                    f'[{self.my_id}→INTERCEPT] '
                    f'Hedefe: {t_dist:.1f}m | '
                    f'Roll: {math.degrees(roll):+.1f}° | Thr: {thrust:.2f}'
                )
                self._last_log_time = now
            self._publish_attitude_setpoint(roll, pitch, self.my_yaw, thrust)
            return

        # ── Gerçek Lider-Takipçi Mesafesi ─────────────────────────────────
        actual_dx   = self.leader_x - self.my_x
        actual_dy   = self.leader_y - self.my_y
        actual_dist = math.sqrt(actual_dx**2 + actual_dy**2)

        # ── CTRV Öngörü: Liderin 0.5 sn Sonraki Konumu ───────────────────
        w  = self.leader_yaw_rate
        v  = math.sqrt(self.leader_vx**2 + self.leader_vy**2)
        yw = self.leader_yaw
        dt = self.LOOK_AHEAD_TIME

        if abs(w) > 0.01:          # Dairesel Yörünge Modeli
            fx = self.leader_x + (v / w) * (math.sin(yw + w * dt) - math.sin(yw))
            fy = self.leader_y + (v / w) * (-math.cos(yw + w * dt) + math.cos(yw))
        else:                      # Doğrusal Yörünge Modeli
            fx = self.leader_x + v * math.cos(yw) * dt
            fy = self.leader_y + v * math.sin(yw) * dt

        # ── Katı Cisim Kinematiği (Rigid Body): Öngörülen Lider Yaw ─────────
        # CTRV ile hesaplanan gelecekteki yaw (pred_leader_yaw) kullanılır.
        # Bu sayede ofset noktası liderin HENÜZ YAPMADIĞI dönüşe göre
        # önceden pozisyon alır → tüm sürü tek bir katı cisim gibi döner.
        pred_leader_yaw = self._normalize_angle(yw + w * dt)   # Gelecekteki Yaw

        # ── Dinamik Formasyon Noktası (2D Rotasyon Matrisi) ───────────────
        # pred_leader_yaw ile ofset vektörünü dünya koordinatlarına döndür.
        # Anlık leader_yaw KULLANILMAZ — katı cisim kinematiği aktif.
        cos_yaw = math.cos(pred_leader_yaw)
        sin_yaw = math.sin(pred_leader_yaw)

        rotated_x = (self.formation_offset_x * cos_yaw) - (self.formation_offset_y * sin_yaw)
        rotated_y = (self.formation_offset_x * sin_yaw) + (self.formation_offset_y * cos_yaw)

        # Formasyon hedef noktası = Lider tahmini konumu + döndürülmüş ofset
        form_x = fx + rotated_x
        form_y = fy + rotated_y

        dx         = form_x - self.my_x
        dy         = form_y - self.my_y
        trail_dist = math.sqrt(dx**2 + dy**2)

        # ── V6: Yumuşatilmiş L1 Güdüm (Swarm Kinematics) ──────────────────────
        # Formasyon noktasına ynelirken burnun %60 noktaya, %40 lider heading'ine bak.
        # Eski '%95 noktaya bak' agrosu çok sert virâj yaratiyordu; yumusatıldı.
        formation_course = math.atan2(dy, dx)
        look_at_err      = self._normalize_angle(formation_course - self.my_yaw)
        heading_err      = self._normalize_angle(self.leader_yaw  - self.my_yaw)
        angle_error      = (look_at_err * 0.60) + (heading_err * 0.40)

        # ── APF: Çarpışma Önleyici İtici Kuvvet ───────────────────────────────────
        rep_angle, rep_strength = self._compute_repulsive_force()
        in_danger = rep_strength > 0.0

        if in_danger:
            escape_err  = self._normalize_angle(rep_angle - self.my_yaw)
            angle_error = self._normalize_angle(
                ((1.0 - self.APF_WEIGHT) * angle_error) + (self.APF_WEIGHT * escape_err)
            )

        # ── V6: İç Kulvar Freni (Inner Track Braking) ───────────────────────────
        # Formasyon noktasına çok yakın ya da geçmiş (önüne geçilmiş):
        # Agresif dönüş (Loiter) YOKTUR. Sadece frenleme (MIN_THRUST) ve
        # Lider'in heading'ine paralel yumuşak düzeltme uygulanir.
        inner_track = (not in_danger) and (trail_dist < 15.0)

        if inner_track:
            # Burnunu lider ile hizala (yumuşak): eski roll komutunu sıfırla
            parallel_err = self._normalize_angle(self.leader_yaw - self.my_yaw)
            roll  = float(np.clip(0.5 * parallel_err, -self.MAX_ROLL, self.MAX_ROLL))
            # Enerji kes: süzulerek formasyon noktasının gerisine düş
            thrust = self.MIN_THRUST
            pitch  = float(np.clip((self.my_z - self.leader_z) * 0.025,
                                   self.MAX_PITCH_DOWN, self.MAX_PITCH_UP))
            if do_log:
                self.get_logger().info(
                    f'[{self.my_id}→İÇ-KULVAR-FRENİ] '
                    f'Form.Mesafe: {trail_dist:.1f}m < 15m | '
                    f'Lider parallel→ Roll: {math.degrees(roll):+.1f}° | Thr: {thrust:.2f}'
                )
        else:
            # Normal L1 Takip (yumuşatilmiş KP)
            roll  = 0.45 * angle_error           # Yumuşatılmış L1 kazancı (Wrap fix)
            roll  = float(np.clip(roll, -self.MAX_ROLL, self.MAX_ROLL))

            thrust, pitch = self._compute_thrust_and_pitch(trail_dist, roll, actual_dist)

            # APF devredeyse ek thrust kesimi
            if in_danger:
                thrust = max(self.MIN_THRUST, thrust * (1.0 - rep_strength * 0.6))
                if do_log:
                    self.get_logger().warn(
                        f'[{self.my_id}→APF-KAÇIŞ] Çarpışma Tehlikesi! '
                        f'Kaçış: {math.degrees(rep_angle):+.1f}° | '
                        f'Baskı: {rep_strength:.2f} | '
                        f'Roll: {math.degrees(roll):+.1f}° | Thr: {thrust:.2f}'
                    )
            elif do_log:
                self.get_logger().info(
                    f'[{self.my_id}→FORMASYON] '
                    f'Lider: {actual_dist:.1f}m | Form.Mesafe: {trail_dist:.1f}m | '
                    f'Ofset(x={self.formation_offset_x:.0f} y={self.formation_offset_y:.0f}) | '
                    f'w: {math.degrees(w):.1f}°/s | '
                    f'AltDiff: {self.my_z - self.leader_z:+.1f}m | '
                    f'Roll: {math.degrees(roll):+.1f}° | '
                    f'Pitch: {math.degrees(pitch):+.1f}° | Thr: {thrust:.2f}'
                )

        if do_log:
            self._last_log_time = now

        self._publish_attitude_setpoint(roll, pitch, self.my_yaw, thrust)

    # ─────────────────────────────────────────────────────────────────────────
    # Yayıncı: OffboardControlMode
    # ─────────────────────────────────────────────────────────────────────────
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp    = self._px4_timestamp()
        msg.position     = False
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = True
        msg.body_rate    = False
        self.pub_offboard_mode.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Yayıncı: VehicleAttitudeSetpoint (Bumpless Transfer EMA dahil)
    # ─────────────────────────────────────────────────────────────────────────
    def _publish_attitude_setpoint(self, target_roll: float, target_pitch: float, yaw: float, thrust: float):
        # Bumpless Transfer: ani komut sıçramalarını EMA ile yumuşat (50/50)
        self.current_roll_out  = (0.50 * self.current_roll_out)  + (0.50 * target_roll)
        self.current_pitch_out = (0.50 * self.current_pitch_out) + (0.50 * target_pitch)

        q = euler_to_quaternion(self.current_roll_out, self.current_pitch_out, yaw)

        msg = VehicleAttitudeSetpoint()
        msg.timestamp      = self._px4_timestamp()
        msg.q_d[0]         = q[0]
        msg.q_d[1]         = q[1]
        msg.q_d[2]         = q[2]
        msg.q_d[3]         = q[3]
        msg.thrust_body[0] = thrust
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = 0.0

        self.pub_attitude_sp.publish(msg)

    @staticmethod
    def _px4_timestamp() -> int:
        return int(time.monotonic() * 1e6)


# ─────────────────────────────────────────────────────────────────────────────
# Giriş Noktası
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SwarmFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SwarmFollower durduruldu (Ctrl+C).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
