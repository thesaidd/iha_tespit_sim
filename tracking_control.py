#!/usr/bin/env python3
"""
tracking_control.py
====================
Sabit Kanatlı İHA - Hibrit Güdüm Kontrol Düğümü (TECS, Offset Pursuit & Predictive CTRV)
Sistem: ROS 2 + PX4 (MicroXRCEAgent DDS)
Mod 1: YOLO Kamera Modu  (yüksek öncelik)
Mod 2: GPS / Telemetri Modu (Predictive Trailing Pursuit - yedek)

Çalıştırmak için:
    python3 tracking_control.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import math
import time

from px4_msgs.msg import (
    VehicleLocalPosition,
    OffboardControlMode,
    VehicleAttitudeSetpoint,
)
from std_msgs.msg import Int32MultiArray

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
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
    while angle >  math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class TrackingControlNode(Node):
    IMAGE_WIDTH  = 640
    IMAGE_HEIGHT = 480
    IMG_CX       = IMAGE_WIDTH  // 2
    IMG_CY       = IMAGE_HEIGHT // 2

    # Kontrol kazançları
    KP_ROLL_CAM   = 0.001
    KD_ROLL_CAM   = 0.001
    KP_PITCH_CAM  = 0.002
    KD_PITCH_CAM  = 0.001
    KP_ROLL_GPS   = 1.0

    # Limitler
    MAX_ROLL       = 0.75  # rad (~43 derece)
    MAX_ROLL_CAM   = 0.75  # rad (~43 derece) Kamera için daraltılmış limit özgürleştirildi
    MAX_PITCH_UP   = 0.35  # rad (~20 derece)
    MAX_PITCH_DOWN = -0.30 # rad (~-17 derece)
    MIN_THRUST     = 0.05
    MAX_THRUST     = 0.85

    # Takip mesafesi ve Tahmin Süresi
    TRAIL_DISTANCE = 50.0  # Güvenli takip mesafesi uzatıldı
    LOOK_AHEAD_TIME = 0.5  # saniye

    LOG_PERIOD = 0.5

    def __init__(self):
        super().__init__('tracking_control_node')

        qos_sensor = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        self.hunter_x    = 0.0
        self.hunter_y    = 0.0
        self.hunter_z    = 0.0
        self.hunter_yaw  = 0.0
        self.hunter_vx   = 0.0
        self.hunter_vy   = 0.0

        self.target_x    = 0.0
        self.target_y    = 0.0
        self.target_z    = 0.0
        self.target_yaw  = 0.0
        self.target_vx   = 0.0
        self.target_vy   = 0.0

        # CTRV Tahmin (Prediction) Değişkenleri
        self.prev_target_yaw = 0.0
        self.prev_target_time = 0.0
        self.target_yaw_rate = 0.0

        self.cam_cx      = 0
        self.cam_cy      = 0
        
        # Kamera Modu Filtre ve PD Değişkenleri
        self.filtered_cx = float(self.IMG_CX)
        self.filtered_cy = float(self.IMG_CY)
        self.prev_err_x  = 0.0
        self.prev_err_y  = 0.0
        self.prev_cam_time = 0.0

        # Çıkış sinyalleri için Bumpless Transfer EMA
        self.current_roll_out = 0.0
        self.current_pitch_out = 0.0

        self._last_log_time = 0.0

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._cb_hunter_pos,
            qos_sensor,
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position_v1',
            self._cb_target_pos,
            qos_sensor,
        )

        self.create_subscription(
            Int32MultiArray,
            '/target/center',
            self._cb_yolo,
            10,
        )

        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_sensor,
        )

        self.pub_attitude_sp = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint_v1',
            qos_sensor,
        )

        self.create_timer(1.0 / 20.0, self._control_loop)

        self.get_logger().info('=== TrackingControlNode başlatıldı (Predictive CTRV) ===')

    def _cb_hunter_pos(self, msg: VehicleLocalPosition):
        self.hunter_x   = msg.x
        self.hunter_y   = msg.y
        self.hunter_z   = msg.z
        self.hunter_yaw = msg.heading
        self.hunter_vx  = msg.vx
        self.hunter_vy  = msg.vy

    def _cb_target_pos(self, msg: VehicleLocalPosition):
        now = time.monotonic()
        
        self.target_x   = msg.x
        self.target_y   = msg.y
        self.target_z   = msg.z
        self.target_vx  = msg.vx
        self.target_vy  = msg.vy
        
        yaw = msg.heading
        
        # Hedefin dönüş hızı (Yaw Rate) hesaplaması
        if self.prev_target_time != 0.0:
            dt = now - self.prev_target_time
            if dt > 0.0:
                dyaw = normalize_angle(yaw - self.prev_target_yaw)
                self.target_yaw_rate = dyaw / dt
                
        self.prev_target_yaw = yaw
        self.prev_target_time = now
        self.target_yaw = yaw

    def _cb_yolo(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.cam_cx = int(msg.data[0])
            self.cam_cy = int(msg.data[1])

    def _compute_thrust_and_pitch(self, trail_dist: float, roll: float, actual_dist: float):
        alt_diff = self.hunter_z - self.target_z  
        
        # 1. İrtifa (Pitch) Kontrolcüsü
        kp_pitch = 0.025
        pitch_alt = alt_diff * kp_pitch 
        
        # Yatış esnasında irtifa kaybını önlemek için ufak burun kaldırma (lift kompanzasyonu)
        pitch_comp = abs(roll) * 0.15
        
        pitch = pitch_alt + pitch_comp
        pitch = float(np.clip(pitch, self.MAX_PITCH_DOWN, self.MAX_PITCH_UP))
        
        # 2. Hız Eşitleme (Speed Matching) ve Uzaklığa Göre Thrust
        threshold_dist = self.TRAIL_DISTANCE + 10.0
        if trail_dist <= threshold_dist:
            base_thrust = 0.42 
        else:
            kp_dist = 0.002
            base_thrust = 0.42 + ((trail_dist - threshold_dist) * kp_dist)
        
        # 3. TECS
        kp_alt_thrust = 0.005
        alt_boost = alt_diff * kp_alt_thrust
        
        thrust = base_thrust + alt_boost
        
        # 4. Dalış Önleyici (Agresif Sönümleme)
        if alt_diff < -5.0 and pitch < 0.0:
            thrust = self.MIN_THRUST

        # 5. Anti-Overshoot (Hava Freni ve Sert Hız Kesici)
        hunter_speed = math.sqrt(self.hunter_vx**2 + self.hunter_vy**2)
        target_speed = math.sqrt(self.target_vx**2 + self.target_vy**2)
        
        hunter_speed_kmh = hunter_speed * 3.6
        
        # Sert Hız Kesici: 60 metreden yakın ve 60 km/h'den hızlıysak motoru tamamen kapat
        if actual_dist < 60.0 and hunter_speed_kmh > 60.0:
            thrust = 0.0
        # Erken Fren (Early Airbrake): 60 metreden yakın ve hızımız hedeften yüksekse süzülmeye (MIN) geç
        elif actual_dist < 60.0 and hunter_speed > target_speed:
            thrust = self.MIN_THRUST
            
        # Eğer hava freni motoru 0.0'a çekmediyse, thrust değerini güvenli dilime al
        if thrust != 0.0:
            thrust = float(np.clip(thrust, self.MIN_THRUST, self.MAX_THRUST))
        
        return thrust, pitch

    def _control_loop(self):
        now = time.monotonic()
        do_log = (now - self._last_log_time) >= self.LOG_PERIOD

        self._publish_offboard_mode()
        
        # Anti-overshoot için ucaklar arasi fiziksel merkez mesafesi
        actual_dx = self.target_x - self.hunter_x
        actual_dy = self.target_y - self.hunter_y
        actual_dist = math.sqrt(actual_dx*actual_dx + actual_dy*actual_dy)

        # Hız hesaplaması (CTRV Modeli için)
        target_speed = math.sqrt(self.target_vx**2 + self.target_vy**2)

        # --- 1. Geleceği Tahmin Etme (Look-ahead Prediction - CTRV) ---
        w = self.target_yaw_rate
        v = target_speed
        yaw = self.target_yaw
        dt_pred = self.LOOK_AHEAD_TIME

        if abs(w) > 0.01: # Dairesel Model
            future_target_x = self.target_x + (v / w) * (math.sin(yaw + w * dt_pred) - math.sin(yaw))
            future_target_y = self.target_y + (v / w) * (-math.cos(yaw + w * dt_pred) + math.cos(yaw))
        else: # Doğrusal Model
            future_target_x = self.target_x + v * math.cos(yaw) * dt_pred
            future_target_y = self.target_y + v * math.sin(yaw) * dt_pred

        future_target_yaw = normalize_angle(yaw + w * dt_pred)

        # --- 2. Gelecekteki Offset (Future Trailing) Noktasının Hesaplanması ---
        trail_x = future_target_x - self.TRAIL_DISTANCE * math.cos(future_target_yaw)
        trail_y = future_target_y - self.TRAIL_DISTANCE * math.sin(future_target_yaw)
        
        dx = trail_x - self.hunter_x
        dy = trail_y - self.hunter_y
        trail_dist = math.sqrt(dx*dx + dy*dy) # Gelecekteki Trailing point'e olan yatay mesafe
        
        camera_active = (self.cam_cx != 0 or self.cam_cy != 0)

        # 3. Yatay (Roll) Kontrolünün Seçimi
        if camera_active:
            if self.prev_cam_time == 0.0:
                self.filtered_cx = float(self.cam_cx)
                self.filtered_cy = float(self.cam_cy)
            else:
                self.filtered_cx = (0.7 * self.filtered_cx) + (0.3 * self.cam_cx)
                self.filtered_cy = (0.7 * self.filtered_cy) + (0.3 * self.cam_cy)

            dt_cam = now - self.prev_cam_time
            if dt_cam <= 0.0:
                dt_cam = 0.05
                
            err_x = self.filtered_cx - self.IMG_CX
            err_y = self.filtered_cy - self.IMG_CY
            
            d_err_x = (err_x - self.prev_err_x) / dt_cam
            d_err_y = (err_y - self.prev_err_y) / dt_cam
            
            pitch_cam = (self.KP_PITCH_CAM * err_y) + (self.KD_PITCH_CAM * d_err_y)
            
            roll  = (self.KP_ROLL_CAM * err_x) + (self.KD_ROLL_CAM * d_err_x)
            roll  = float(np.clip(roll, -self.MAX_ROLL_CAM, self.MAX_ROLL_CAM))
            
            thrust, pitch = self._compute_thrust_and_pitch(trail_dist, roll, actual_dist)
            yaw_out = self.hunter_yaw
            
            self.prev_err_x = err_x
            self.prev_err_y = err_y
            self.prev_cam_time = now
            
            if do_log:
                self.get_logger().info(
                    f'[KAMERA MODU] F-TrailMesafe: {trail_dist:.1f}m '
                    f'px_err=({int(err_x):+d}) Roll: {math.degrees(roll):+.1f}° '
                    f'Pitch: {math.degrees(pitch):+.1f}° Thr: {thrust:.2f}'
                )
        else:
            self.prev_cam_time = 0.0  
            
            # --- 4. Pure Pursuit (Kamerayı Odaklama) ve Heading Senkronizasyonu ---
            # Eski L1 trail sapması yerine, burnunu DOĞRUDAN hedefin GEÇERLİ merkezine dönsün:
            target_course  = math.atan2(actual_dy, actual_dx)           
            look_at_err = normalize_angle(target_course - self.hunter_yaw)
            heading_err = normalize_angle(self.target_yaw - self.hunter_yaw)
            
            angle_error = (look_at_err * 0.95) + (heading_err * 0.05)
            
            if abs(angle_error) > 1.57:  
                roll = self.MAX_ROLL if angle_error > 0 else -self.MAX_ROLL
                
                thrust, pitch = self._compute_thrust_and_pitch(trail_dist, roll, actual_dist)
                thrust = self.MIN_THRUST  
                
                if do_log:
                    self.get_logger().info(
                        f'[LOITER MODU] Öne Geçildi! F-Mesafe: {trail_dist:.1f}m '
                        f'Dönüyor: {math.degrees(roll):+.1f}° Thr: {thrust:.2f}'
                    )
            else:
                roll  = self.KP_ROLL_GPS * angle_error
                roll  = float(np.clip(roll, -self.MAX_ROLL, self.MAX_ROLL))
                
                thrust, pitch = self._compute_thrust_and_pitch(trail_dist, roll, actual_dist)
                
                if do_log:
                    self.get_logger().info(
                        f'[TELEMETRİ L1-PREDICT] Mesafe: {trail_dist:.1f}m (w:{math.degrees(w):.1f}°/s) '
                        f'HedefAlt(Diff): {self.hunter_z - self.target_z:.1f}m '
                        f'Roll: {math.degrees(roll):+.1f}° Pitch: {math.degrees(pitch):+.1f}° Thr: {thrust:.2f}'
                    )

            yaw_out = self.hunter_yaw

        if do_log:
            self._last_log_time = now

        self._publish_attitude_setpoint(roll, pitch, yaw_out, thrust)

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp        = self._px4_timestamp()
        msg.position         = False
        msg.velocity         = False
        msg.acceleration     = False
        msg.attitude         = True
        msg.body_rate        = False
        self.pub_offboard_mode.publish(msg)

    def _publish_attitude_setpoint(self, target_roll: float, target_pitch: float, yaw: float, thrust: float):
        self.current_roll_out = (0.50 * self.current_roll_out) + (0.50 * target_roll)
        self.current_pitch_out = (0.50 * self.current_pitch_out) + (0.50 * target_pitch)

        q = euler_to_quaternion(self.current_roll_out, self.current_pitch_out, yaw)

        msg = VehicleAttitudeSetpoint()
        msg.timestamp        = self._px4_timestamp()
        msg.q_d[0]           = q[0]
        msg.q_d[1]           = q[1]
        msg.q_d[2]           = q[2]
        msg.q_d[3]           = q[3]
        msg.thrust_body[0]   = thrust
        msg.thrust_body[1]   = 0.0
        msg.thrust_body[2]   = 0.0

        self.pub_attitude_sp.publish(msg)

    @staticmethod
    def _px4_timestamp() -> int:
        return int(time.monotonic() * 1e6)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kullanıcı tarafından durduruldu (Ctrl+C).')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
