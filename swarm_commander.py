#!/usr/bin/env python3
"""
swarm_commander.py
==================
Sürü İHA V5 - Yer İstasyonu (GCS) Komuta Düğümü
Sistem: ROS 2

Görev:
  Operatörden klavye girdisi alarak /swarm/formation_cmd topic'ine
  formasyon emirleri, /swarm/target_assignments topic'ine ise
  görev hedefi atamelari yayınlar.
  TRANSIENT_LOCAL QoS sayesinde sonradan sürüye katılan İHA'lar
  da en son emri hemen görebilir.

Çalıştırmak için:
    python3 swarm_commander.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import String
import threading
import time

# ─────────────────────────────────────────────────────────────────────────────
# Desteklenen Formasyon Komutları
# ─────────────────────────────────────────────────────────────────────────────

FORMATIONS = {
    '1': 'V_SHAPE',       # V-Formasyonu (Kazlar gibi)
    '2': 'LINE',          # Yatay Çizgi (Line-Abreast)
    '3': 'ARROW',         # Ok ucu (Diamond / Arrowhead)
    '4': 'ECHELON_RIGHT', # Sağa kademeli sıra (Echelon Right)
    '5': 'ECHELON_LEFT',  # Sola kademeli sıra (Echelon Left)
    '6': 'TRAIL',         # Tek sütun (Column / Trail)
    '0': 'HOLD',          # Mevcut pozisyonu koru (Emergency Halt)
}

MENU = """
╔══════════════════════════════════════════════════════════╗
║         🚁  SÜRÜ GCS KOMUTA PANELİ  🚁                  ║
╠══════════════════════════════════════════════════════════╣
║  [1]  V_SHAPE        →  V-Formasyonu                    ║
║  [2]  LINE           →  Yatay Çizgi (Line-Abreast)      ║
║  [3]  ARROW          →  Ok Ucu (Diamond)                 ║
║  [4]  ECHELON_RIGHT  →  Sağa Kademeli Sıra              ║
║  [5]  ECHELON_LEFT   →  Sola Kademeli Sıra              ║
║  [6]  TRAIL          →  Tek Sütun (Column)              ║
║  [7]  ATTACK_TARGETS →  🎯 Görev Hedefi Ata (İHA: X, Y) ║
║  [8]  RALLY_POINT    →  🏠 ÜSSE DÖNÜŞ / ORBIT          ║
║  [0]  HOLD           →  🚨 ACİL DURDUR                  ║
║  [q]  Çıkış                                             ║
╚══════════════════════════════════════════════════════════╝
"""

# ─────────────────────────────────────────────────────────────────────────────
# Swarm Commander Düğümü
# ─────────────────────────────────────────────────────────────────────────────

class SwarmCommanderNode(Node):

    TOPIC        = '/swarm/formation_cmd'
    TARGET_TOPIC = '/swarm/target_assignments'

    def __init__(self):
        super().__init__('swarm_commander_node')

        # ── QoS: RELIABLE + TRANSIENT_LOCAL ──────────────────────────────────
        # TRANSIENT_LOCAL: Yeni abone olan İHA'lar, önceki mesajı da alır.
        # RELIABLE       : Paket kayıpları durumunda yeniden iletim garantili.
        qos_gcs = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        self.pub_formation = self.create_publisher(
            String,
            self.TOPIC,
            qos_gcs,
        )

        self.pub_targets = self.create_publisher(
            String,
            self.TARGET_TOPIC,
            qos_gcs,
        )

        self.current_formation = 'HOLD'
        self._running = True

        self.get_logger().info(f'=== SwarmCommander V5 Başlatıldı ===')
        self.get_logger().info(f'  Formasyon Topic : {self.TOPIC}')
        self.get_logger().info(f'  Hedef Topic     : {self.TARGET_TOPIC}')
        self.get_logger().info(f'  QoS             : RELIABLE + TRANSIENT_LOCAL')

        # İlk emri yayınla (HOLD): Sürüye "bekle" komutu gönder
        self._publish(self.current_formation)

        # Klavye girdisi ayrı bir thread'de çalışır
        # (rclpy.spin() ana thread'i bloke etmez)
        self._input_thread = threading.Thread(
            target=self._input_loop,
            daemon=True
        )
        self._input_thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    # Klavye Giriş Döngüsü (Ayrı İş Parçacığı)
    # ─────────────────────────────────────────────────────────────────────────
    def _input_loop(self):
        print(MENU)
        print(f'  ▶ Aktif Formasyon: [ {self.current_formation} ]\n')

        while self._running:
            try:
                key = input('  Tuş > ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if key == 'q':
                self.get_logger().info('Komuta düğümü kapatılıyor...')
                self._running = False
                rclpy.shutdown()
                break

            # ── [7] GÖREV HEDEFİ ATAMA (ATTACK_TARGETS) ─────────────────────
            if key == '7':
                print('\n  🎯 ATTACK TARGETS — Görev Hedefi Ata')
                print('  Format: uav_id:x,y  (boş bırakarak bitir)')
                print('  Örnek : uav1:500,200')
                assignments = []
                while True:
                    try:
                        entry = input('  Atama > ').strip()
                    except (EOFError, KeyboardInterrupt):
                        break
                    if not entry:
                        break
                    assignments.append(entry)
                    more = input('  Başka İHA ata? (E/h) > ').strip().lower()
                    if more not in ('e', 'evet', 'y', 'yes', ''):
                        break

                if assignments:
                    payload = '|'.join(assignments)
                    self._publish_targets(payload)
                    print(f'\n  🟢 Hedef atamalar gönderildi: {payload}\n')
                else:
                    print('  ❌ Atama iptal edildi.\n')
                continue

            # ── [8] RALLY POINT (ACİL ÜSSE DÖNÜŞ) ───────────────────────────
            if key == '8':
                self._publish('RALLY')
                self.current_formation = 'RALLY'
                print(f'\n  🏠 RALLY komutu gönderildi! Tüm sürü (0,0) Orbit noktasına çağrıldı.')
                print(f'  ▶ Aktif Mod: [ RALLY ]\n')
                continue

            # ── Formasyon Komutları ───────────────────────────────────────────
            if key in FORMATIONS:
                formation = FORMATIONS[key]
                self._publish(formation)
                self.current_formation = formation
                status = '🚨 ACİL DURDUR!' if formation == 'HOLD' else f'✅ Formasyon: {formation}'
                print(f'\n  {status}')
                print(f'  ▶ Aktif Formasyon: [ {self.current_formation} ]\n')
            else:
                print(f'  ❌ Geçersiz tuş: "{key}". Menüdeki tuşlamaları kullanın.\n')

    # ─────────────────────────────────────────────────────────────────────────
    # Yayıncı
    # ─────────────────────────────────────────────────────────────────────────
    def _publish(self, formation: str):
        msg = String()
        msg.data = formation
        self.pub_formation.publish(msg)
        self.get_logger().info(f'[GCS → SÜRÜ] Formasyon Emri: "{formation}"')

    def _publish_targets(self, payload: str):
        """Hedef atamalarini yayinlar.
        Format: 'uav1:500.0,200.0|uav2:-300.0,400.0'
        """
        msg = String()
        msg.data = payload
        self.pub_targets.publish(msg)
        self.get_logger().info(f'[GCS → SÜRÜ] Görev Hedefleri: "{payload}"')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Giriş Noktası
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SwarmCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SwarmCommander durduruldu (Ctrl+C).')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
