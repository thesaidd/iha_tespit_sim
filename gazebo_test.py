"""
Gazebo Standalone Test
======================
Gazebo bağlantısı, cv_bridge veya model ROS paketi kurulmadan önce
sistemi test etmek için basit bir döngü simülatörü.

Senaryolar:
  1. Webcam / video → nesne_tespit_sistemi() → PID çıktısı simüle et
  2. Yapay renk bloğu ile "lider İHA" simüle et
"""

import cv2
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nesne_tespit import nesne_tespit_sistemi, ciz_sonuc


# ──────────────────────────────────────────────────────────────────────
# Basit PID Kontrolcü Simülasyonu
# ──────────────────────────────────────────────────────────────────────

class SimplePID:
    """
    Miniature PID — gerçek uçak kontrolcünüz bunu kullanmaz,
    sadece simülasyonda kontrol sinyalinin nasıl üretildiğini gösterir.
    """
    def __init__(self, kp=0.005, ki=0.0001, kd=0.002):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float) -> float:
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


pid_x = SimplePID()
pid_y = SimplePID()


# ──────────────────────────────────────────────────────────────────────
# Yapay "Lider İHA" framesi oluşturucu (gerçek kamera yoksa)
# ──────────────────────────────────────────────────────────────────────

def create_synthetic_frame(t: float, width=640, height=480) -> np.ndarray:
    """
    Basit bir "hedef İHA" simüle eden sentetik frame.
    Ekranda dairesel hareket yapan bir dikdörtgen çizer.
    """
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    # Arka plan gürültüsü (gökyüzü efekti)
    noise = np.random.randint(30, 80, (height, width, 3), dtype=np.uint8)
    frame = cv2.addWeighted(frame, 0.0, noise, 1.0, 0)

    # Dairesel hareket
    cx = int(width/2  + (width*0.3)  * np.cos(t * 0.5))
    cy = int(height/2 + (height*0.2) * np.sin(t * 0.5))

    # "İHA" olarak küçük gri dikdörtgen
    bw, bh = 40, 20
    cv2.rectangle(frame,
                  (cx - bw//2, cy - bh//2),
                  (cx + bw//2, cy + bh//2),
                  (180, 180, 180), -1)

    # Kanat çizgileri
    cv2.line(frame, (cx - bw, cy), (cx + bw, cy), (150, 150, 150), 3)

    return frame


# ──────────────────────────────────────────────────────────────────────
# Ana Döngü
# ──────────────────────────────────────────────────────────────────────

def run_gazebo_test(source=None):
    """
    Args:
        source: None → sentetik frame kullan
                int  → webcam index
                str  → video dosyası yolu
    """
    if source is not None:
        cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
        use_synthetic = False
    else:
        cap = None
        use_synthetic = True

    print("=" * 55)
    print(" GAZEBO STANDALONE TEST")
    print(" ESC → çıkış")
    print("=" * 55)

    t = 0.0
    dt = 1.0 / 30.0
    frame_num = 0

    while True:
        # ── Frame al ─────────────────────────────────────────────────
        if use_synthetic:
            frame = create_synthetic_frame(t)
            t += dt
        else:
            ret, frame = cap.read()
            if not ret:
                break

        # ── TESPİT ───────────────────────────────────────────────────
        is_detected, cx, cy = nesne_tespit_sistemi(frame)

        # ── KONTROL SİNYALİ (PID Simülasyonu) ───────────────────────
        img_cx = frame.shape[1] // 2
        img_cy = frame.shape[0] // 2

        if is_detected:
            hata_x = cx - img_cx   # + → sağda, - → solda
            hata_y = cy - img_cy   # + → aşağıda, - → yukarıda

            roll_cmd  = pid_x.compute(hata_x)   # sağa/sola
            pitch_cmd = pid_y.compute(hata_y)   # yukarı/aşağı

            print(f"[{frame_num:05d}] BULUNDU  "
                  f"cx={cx:4d} cy={cy:4d}  "
                  f"hata=({hata_x:+4d},{hata_y:+4d})  "
                  f"roll={roll_cmd:+.3f}  pitch={pitch_cmd:+.3f}")
        else:
            print(f"[{frame_num:05d}] YOK — arama modu")

        # ── GÖRSELLEŞTİRME ───────────────────────────────────────────
        vis = ciz_sonuc(frame, is_detected, cx, cy)

        # PID çıktısını göster
        if is_detected:
            cv2.putText(vis,
                        f"roll={roll_cmd:+.3f}  pitch={pitch_cmd:+.3f}",
                        (10, vis.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cv2.imshow("Gazebo Sim Test", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

        frame_num += 1

    if cap:
        cap.release()
    cv2.destroyAllWindows()


# ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--source", default=None,
                   help="None=sentetik, 0=webcam, video.mp4=dosya")
    args = p.parse_args()

    run_gazebo_test(args.source)
