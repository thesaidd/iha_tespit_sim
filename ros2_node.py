"""
ROS2 Entegrasyon Betiği
========================
Gazebo simülasyonunda çalışacak ROS2 node.

Düğüm:
  /camera/image_raw  →  [sensor_msgs/Image]  →  nesne_tespit_sistemi()
                                              ↓
  /target/detection  ←  [custom topic]  ←  (is_detected, cx, cy)

Kullanım (Ubuntu / ROS2 kurulu makine):
    cd ~/ros2_ws/src
    cp -r /path/to/Tübitak_sim uav_detection
    # package.xml ve CMakeLists.txt ekle (ROS2 Python paketi olarak)
    colcon build --packages-select uav_detection
    source install/setup.bash
    ros2 run uav_detection ros2_node
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Int32MultiArray
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[UYARI] ROS2 bulunamadı. Bu betik sadece ROS2 ortamında çalışır.")

from nesne_tespit import nesne_tespit_sistemi, ciz_sonuc


# ──────────────────────────────────────────────────────────────────────
class UAVDetectionNode(Node if ROS_AVAILABLE else object):
    """
    ROS2 Node: Kamera görüntüsünü alır, tespit yapar, sonucu yayınlar.

    Yayınlanan Konular:
    ┌──────────────────────────────┬──────────────────────────────────┐
    │ /target/is_detected          │ std_msgs/Bool                    │
    │ /target/center               │ std_msgs/Int32MultiArray [cx,cy] │
    │ /target/debug_image          │ sensor_msgs/Image (görsel)       │
    └──────────────────────────────┴──────────────────────────────────┘
    """

    def __init__(self):
        super().__init__('uav_detection_node')
        self.bridge = CvBridge()

        # Abone ol
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',      # ← Gazebo kamera konusu
            self.image_callback,
            10
        )

        # Yayınla
        self.pub_detected = self.create_publisher(Bool,            '/target/is_detected', 10)
        self.pub_center   = self.create_publisher(Int32MultiArray, '/target/center',       10)
        self.pub_debug    = self.create_publisher(Image,           '/target/debug_image',  10)

        self.get_logger().info("UAV Detection Node başlatıldı.")
        self.get_logger().info("Abone: /camera/image_raw")
        self.get_logger().info("Yayın: /target/is_detected | /target/center")

    def image_callback(self, msg: 'Image'):
        """Kamera mesajı geldiğinde çağrılır."""
        # ROS Image → OpenCV numpy array (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ─── ANA ÇAĞRI ───────────────────────────────────────────────
        is_detected, cx, cy = nesne_tespit_sistemi(frame)
        # ─────────────────────────────────────────────────────────────

        # is_detected yayınla
        det_msg = Bool()
        det_msg.data = is_detected
        self.pub_detected.publish(det_msg)

        # Merkez koordinatları yayınla
        center_msg = Int32MultiArray()
        center_msg.data = [cx, cy]
        self.pub_center.publish(center_msg)

        # Debug görüntüsü yayınla
        vis = ciz_sonuc(frame, is_detected, cx, cy)
        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        self.pub_debug.publish(debug_msg)

        if is_detected:
            self.get_logger().info(
                f"HEDEF: cx={cx}, cy={cy}  "
                f"hata=({cx - frame.shape[1]//2:+d}, {cy - frame.shape[0]//2:+d})"
            )


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    if not ROS_AVAILABLE:
        print("[HATA] ROS2 / rclpy bulunamadı. Lütfen ROS2 ortamını aktifleştirin.")
        sys.exit(1)

    rclpy.init(args=args)
    node = UAVDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
