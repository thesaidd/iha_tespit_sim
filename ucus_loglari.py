#!/usr/bin/env python3
"""
ucus_loglari.py
================
Sabit Kanatlı İHA - Pasif Loglama Düğümü (Telemetri Analizi)
Sistem: ROS 2 + PX4 (MicroXRCEAgent DDS)

Çalıştırmak için:
    python3 ucus_loglari.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
)
from std_msgs.msg import Int32MultiArray

def quaternion_to_euler(q):
    """
    q: [w, x, y, z]
    Dönüş: roll, pitch, yaw (radyan)
    """
    w, x, y, z = q

    # Roll (x ekseni etrafında)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y ekseni etrafında)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z ekseni etrafında)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class FlightLogNode(Node):
    def __init__(self):
        super().__init__('flight_log_node')

        # PX4 QoS Profile (Best-Effort, Volatile, Keep Last 1)
        qos_sensor = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        # Durum Değişkenleri
        self.hunter_x = 0.0
        self.hunter_y = 0.0
        self.hunter_z = 0.0
        self.hunter_vx = 0.0
        self.hunter_vy = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0

        self.hunter_roll = 0.0

        self.cam_cx = 0
        self.cam_cy = 0

        # Abonelikler
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._cb_hunter_pos,
            qos_sensor
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position_v1',
            self._cb_target_pos,
            qos_sensor
        )

        # Gerçek Yatış verisi için
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._cb_hunter_att,
            qos_sensor
        )

        # YOLO Verisi için
        self.create_subscription(
            Int32MultiArray,
            '/target/center',
            self._cb_yolo,
            10
        )

        # Çıktı formatı için Timer (2 Hz = 0.5 sn)
        self.create_timer(0.5, self._log_loop)

        self.get_logger().info('=== Uçuş Log Düğümü Başlatıldı ===')

    def _cb_hunter_pos(self, msg: VehicleLocalPosition):
        self.hunter_x = msg.x
        self.hunter_y = msg.y
        self.hunter_z = msg.z
        self.hunter_vx = msg.vx
        self.hunter_vy = msg.vy

    def _cb_target_pos(self, msg: VehicleLocalPosition):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z
        self.target_vx = msg.vx
        self.target_vy = msg.vy

    def _cb_hunter_att(self, msg: VehicleAttitude):
        # msg.q elemanı PX4'te her zaman [w, x, y, z] quaternion verisidir
        roll, pitch, yaw = quaternion_to_euler(msg.q)
        self.hunter_roll = roll

    def _cb_yolo(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.cam_cx = int(msg.data[0])
            self.cam_cy = int(msg.data[1])

    def _log_loop(self):
        # Yatay Mesafe Hesaplama
        dx = self.hunter_x - self.target_x
        dy = self.hunter_y - self.target_y
        distance = math.sqrt(dx*dx + dy*dy)

        # İrtifa Farkı (Avcı Z - Hedef Z)
        # NED koordinatlarında Z eksi olduğu için yüksekliği ifade eder.
        alt_fark = self.hunter_z - self.target_z

        # Hız Hesaplamaları (m/s -> km/h çevrimi)
        hunter_speed = math.sqrt(self.hunter_vx**2 + self.hunter_vy**2) * 3.6
        target_speed = math.sqrt(self.target_vx**2 + self.target_vy**2) * 3.6

        # Roll açısı (derece)
        roll_deg = math.degrees(self.hunter_roll)

        log_msg = (
            f"[UÇUŞ LOG] Mesafe: {distance:.1f}m | "
            f"Alt_Fark: {alt_fark:+.1f}m | "
            f"Avcı: {int(hunter_speed)}km/h (Roll: {int(roll_deg):+d}°) | "
            f"Hedef: {int(target_speed)}km/h | "
            f"YOLO: ({self.cam_cx}, {self.cam_cy})"
        )

        self.get_logger().info(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightLogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Uçuş Log Düğümü Kapatıldı.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
