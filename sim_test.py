"""
sim_test.py
===========
Yerel Test Betiği — Simülatör olmadan sistemi test et.

Kullanım:
    # Webcam ile test
    python sim_test.py --source 0

    # Video dosyası ile test
    python sim_test.py --source ornek_video.mp4

    # Belirtilen görüntü boyutu ile test (Gazebo kamera çözünürlüğünü taklit et)
    python sim_test.py --source 0 --width 640 --height 480
"""

import cv2
import time
import argparse
import sys
import os

# Proje kökünü yola ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nesne_tespit import nesne_tespit_sistemi, ciz_sonuc


def parse_args():
    p = argparse.ArgumentParser(description="Simülasyon Tespit Testi")
    p.add_argument("--source", default="0",
                   help="Kaynak: webcam index (0,1,...) veya video dosyası yolu")
    p.add_argument("--width",  type=int, default=None,
                   help="Kamera genişliği (piksel)")
    p.add_argument("--height", type=int, default=None,
                   help="Kamera yüksekliği (piksel)")
    p.add_argument("--no-display", action="store_true",
                   help="Pencere açmadan çalıştır (headless)")
    return p.parse_args()


def main():
    args = parse_args()

    # Kaynak seç
    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print(f"[HATA] Kaynak açılamadı: {args.source}")
        sys.exit(1)

    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("\n[Kontroller]")
    print("  ESC  → Çıkış")
    print("  'r'  → Sistemi sıfırla\n")

    fps_t0 = time.time()
    fps_count = 0
    current_fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame okunamadı — çıkılıyor.")
            break

        # ─── ANA ÇAĞRI ───────────────────────────────────────────────
        is_detected, cx, cy = nesne_tespit_sistemi(frame)
        # ─────────────────────────────────────────────────────────────

        # FPS hesapla
        fps_count += 1
        if fps_count >= 10:
            current_fps = fps_count / (time.time() - fps_t0)
            fps_t0 = time.time()
            fps_count = 0

        # Görsel
        if not args.no_display:
            vis = ciz_sonuc(frame, is_detected, cx, cy)
            cv2.putText(vis, f"FPS: {current_fps:.1f}",
                        (10, vis.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            cv2.imshow("Sim Tespit Testi", vis)

        # Çıktı (simülatör entegrasyonunu simüle eder)
        if is_detected:
            hata_x = cx - frame.shape[1] // 2
            hata_y = cy - frame.shape[0] // 2
            print(f"[BULUNDU] cx={cx}, cy={cy} | hata=({hata_x:+d}, {hata_y:+d})")
        else:
            print("[YOK]")

        # Tuş kontrolü
        if not args.no_display:
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('r'):
                from nesne_tespit import _system
                if _system:
                    _system._reset_tracking()
                print("[SIFIRLA] Sistem sıfırlandı.")

    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()
    print("Test tamamlandı.")


if __name__ == "__main__":
    main()
