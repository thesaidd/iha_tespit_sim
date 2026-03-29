#!/usr/bin/env python3
"""
yolo_arayuz.py
==============
Avcı İHA vizöründe hedefin (Bounding Box) görsel olarak işaretlenmesi ve
takip durumunun saniye bazlı renkli OSD ile yansıtılmasını sağlayan ROS 2 OpenCV arayüzü.

Kurulum gereksinimleri (Eğer yoksa):
    sudo apt install ros-humble-cv-bridge
    pip install opencv-python

Çalıştırmak için:
    python3 yolo_arayuz.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import time

class YoloArayuzNode(Node):
    def __init__(self):
        super().__init__('yolo_arayuz_node')
        
        self.bridge = CvBridge()
        
        self.cam_cx = 0
        self.cam_cy = 0
        
        self.target_visible = False
        self.first_detection_time = 0.0
        
        # Kullanıcının kamera topic ismini kolayca değiştirebilmesi için ayar değişkeni
        # (Gazebo veya gerçek uçak donanımı için burayı güncelleyin)
        self.CAMERA_TOPIC = '/uav0/camera'
        
        # Kamera konusuna abone olma
        self.create_subscription(
            Image,
            self.CAMERA_TOPIC,
            self._cb_image,
            rclpy.qos.qos_profile_sensor_data
        )

        # YOLO Center aboneligi
        self.create_subscription(
            Int32MultiArray,
            '/target/center',
            self._cb_yolo,
            10
        )

        self.get_logger().info('=== YOLO Görsel Takip Ekranı Başlatıldı ===')
        self.get_logger().info('Hedef kamerası (OpenCV) dinleniyor...')

    def _cb_yolo(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.cam_cx = int(msg.data[0])
            self.cam_cy = int(msg.data[1])

    def _cb_image(self, msg: Image):
        # ROS 2 Image mesajini OpenCV (BGR) formatina cevir
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Görüntü dönüştürme hatası: {e}")
            return
            
        now = time.monotonic()
        
        # Eger koordinatlar 0,0 dan farkliysa vizorde hedef vardir
        if self.cam_cx != 0 and self.cam_cy != 0:
            if not self.target_visible:
                # Ekrana yeni girdi (veya koptuktan sonra geri dondu)
                self.target_visible = True
                self.first_detection_time = now
            
            duration = now - self.first_detection_time
            
            # Dinamik 50x50 Yesil Bounding Box
            # Kenarlardan tasmamasini guarantiler
            top_left = (max(0, self.cam_cx - 25), max(0, self.cam_cy - 25))
            bottom_right = (min(cv_img.shape[1], self.cam_cx + 25), min(cv_img.shape[0], self.cam_cy + 25))
            cv2.rectangle(cv_img, top_left, bottom_right, (0, 255, 0), 2)
            
            # Tam hedefin ustune (Y-10 pixel) yazi bastir
            text_pos = (top_left[0], max(20, top_left[1] - 10))
            
            if duration < 1.0:
                text = "[ TESPIT EDILDI ]"
                color = (0, 255, 255) # BGR (Sari)
            else:
                text = "[ TAKIP EDILIYOR ]"
                color = (0, 255, 0)   # BGR (Yesil)
                
            cv2.putText(cv_img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Merkez isareti (Artı / Crosshair)
            cv2.drawMarker(cv_img, (self.cam_cx, self.cam_cy), (0, 0, 255), cv2.MARKER_CROSS, 10, 1)

        else:
            self.target_visible = False
            text = "HEDEF ARANIYOR..."
            color = (0, 0, 255) # BGR (Kirmizi)
            cv2.putText(cv_img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Görüntüyü ekrana bas
        cv2.imshow("Avci Ucak Vizoru", cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloArayuzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ekran kapatılıyor (Ctrl+C).')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
