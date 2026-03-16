"""
nesne_tespit.py
===============
Simülasyon Entegrasyon Noktası
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Gazebo/ROS veya herhangi bir simülatör bu dosyayı import eder.
Kullanım sadece tek satır:

    from detection_core.nesne_tespit import nesne_tespit_sistemi

    is_detected, cx, cy = nesne_tespit_sistemi(frame)

GİRİŞ  : frame → np.ndarray (BGR, kameradan gelen ham görüntü)
ÇIKIŞ  : (is_detected: bool, cx: int, cy: int)
          is_detected = True  → hedef görüntüde var, cx/cy merkez piksel
          is_detected = False → hedef yok, cx=0, cy=0
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import sys
import os

# detection_core klasörünü yola ekle (farklı CWD durumu için)
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import cv2
import numpy as np
from typing import Tuple

from detection_core.detector    import UAVDetector
from detection_core.tracker     import RobustTracker
from detection_core.kalman_filter import UAVKalmanFilter
import detection_core.config    as cfg


# ──────────────────────────────────────────────────────────────────────
# Singleton Sistemi (bir kez başlat, her çağrıda kullan)
# ──────────────────────────────────────────────────────────────────────

class _DetectionSystem:
    """İç sınıf — doğrudan kullanmayın, nesne_tespit_sistemi() kullanın."""

    def __init__(self):
        print("="*55)
        print(" SİMÜLASYON TESPİT SİSTEMİ BAŞLATILIYOR")
        print("="*55)

        self.detector = UAVDetector(
            model_path     = cfg.MODEL_PATH,
            conf_threshold = cfg.CONF_THRESHOLD,
            iou_threshold  = cfg.IOU_THRESHOLD,
            device         = cfg.DEVICE,
            target_classes = cfg.TARGET_CLASSES,
        )

        self.tracker = RobustTracker(
            tracker_type   = cfg.TRACKER_TYPE,
            trajectory_length = cfg.TRAJECTORY_LENGTH,
            max_bbox_change   = cfg.MAX_BBOX_CHANGE,
            min_bbox_area     = cfg.MIN_BBOX_AREA,
        )

        self.kalman = UAVKalmanFilter(
            process_noise     = cfg.KALMAN_Q,
            measurement_noise = cfg.KALMAN_R,
        ) if cfg.USE_KALMAN else None

        self.frame_count      = 0
        self.in_tracking_mode = False   # False = tespit modu, True = takip modu

        print("="*55)
        print(" SİSTEM HAZIR  →  nesne_tespit_sistemi(frame)")
        print("="*55)

    # ------------------------------------------------------------------
    def process(self, frame: np.ndarray) -> Tuple[bool, int, int]:
        """
        Ana işlem fonksiyonu.

        Returns:
            (is_detected, cx, cy)
        """
        self.frame_count += 1
        reinit_now = (self.frame_count % cfg.REINIT_INTERVAL == 0)

        # ── TAKİP MODU ──────────────────────────────────────────────
        if self.in_tracking_mode and self.tracker.is_tracking and not reinit_now:
            success, bbox = self.tracker.update(frame)

            if success:
                cx, cy = self._bbox_center(bbox)

                # Kalman güncelle
                if self.kalman:
                    cx_k, cy_k = self.kalman.update((cx, cy))
                    cx, cy = int(cx_k), int(cy_k)

                return True, cx, cy
            else:
                # Takip kaybedildi → tespit moduna dön
                self._reset_tracking()

        # ── TESPİT MODU ─────────────────────────────────────────────
        detections = self.detector.detect(frame)

        if not detections:
            self.in_tracking_mode = False
            return False, 0, 0

        best = self.detector.get_best_detection(detections)
        cx, cy = best['center']

        # Tracker ve Kalman'ı başlat
        self.tracker.init(frame, best['bbox'])
        if self.kalman:
            self.kalman.init(float(cx), float(cy))
        self.in_tracking_mode = True

        return True, cx, cy

    # ------------------------------------------------------------------
    def _bbox_center(self, bbox) -> Tuple[int, int]:
        x1, y1, x2, y2 = bbox
        return (x1 + x2) // 2, (y1 + y2) // 2

    def _reset_tracking(self):
        self.tracker.reset()
        if self.kalman:
            self.kalman.reset()
        self.in_tracking_mode = False


# Singleton örneği — ilk çağrıda oluşturulur
_system: _DetectionSystem | None = None


def _get_system() -> _DetectionSystem:
    global _system
    if _system is None:
        _system = _DetectionSystem()
    return _system


# ══════════════════════════════════════════════════════════════════════
#  ★  ANA ENTEGRASYON FONKSİYONU  ★
# ══════════════════════════════════════════════════════════════════════

def nesne_tespit_sistemi(frame: np.ndarray) -> Tuple[bool, int, int]:
    """
    Simülatörden gelen görüntüyü işler.

    Parameters
    ----------
    frame : np.ndarray
        BGR formatında kamera görüntüsü (Gazebo → cv_bridge → numpy array)

    Returns
    -------
    is_detected : bool
        True  → hedef tespit edildi
        False → hedef görüntüde yok
    cx : int
        Hedefin görüntü üzerindeki X merkez koordinatı (piksel)
        (is_detected=False ise = 0)
    cy : int
        Hedefin görüntü üzerindeki Y merkez koordinatı (piksel)
        (is_detected=False ise = 0)

    Example
    -------
    >>> is_detected, cx, cy = nesne_tespit_sistemi(frame)
    >>> if is_detected:
    ...     hata_x = cx - frame.shape[1] // 2   # + sağda, - solda
    ...     hata_y = cy - frame.shape[0] // 2   # + aşağıda, - yukarıda
    ...     # → PID kontrolcüye ver
    """
    return _get_system().process(frame)


# ══════════════════════════════════════════════════════════════════════
#  Opsiyonel: görselleştirme (simülatördeki debug penceresi için)
# ══════════════════════════════════════════════════════════════════════

def ciz_sonuc(frame: np.ndarray,
              is_detected: bool,
              cx: int, cy: int) -> np.ndarray:
    """
    Tespit sonucunu frame üzerine çiz (debug/monitör için).

    Returns
    -------
    Annotated frame (orijinalin kopyası)
    """
    vis = frame.copy()
    h, w = vis.shape[:2]
    cx_img, cy_img = w // 2, h // 2

    if is_detected:
        # Hedef kutusu
        sys_ = _get_system()
        if sys_.tracker.last_bbox:
            x1, y1, x2, y2 = sys_.tracker.last_bbox
            cv2.rectangle(vis, (x1, y1), (x2, y2), cfg.COLOR_BBOX, 2)

        # Merkez noktası
        cv2.circle(vis, (cx, cy), 6, cfg.COLOR_CROSSHAIR, -1)

        # Hata çizgisi (görüntü merkezi → hedef merkezi)
        cv2.line(vis, (cx_img, cy_img), (cx, cy), cfg.COLOR_CROSSHAIR, 1)

        # İz
        traj = sys_.tracker.get_trajectory()
        for i in range(1, len(traj)):
            alpha = i / len(traj)
            col = tuple(int(c * alpha) for c in cfg.COLOR_TRAJECTORY)
            cv2.line(vis, traj[i-1], traj[i], col, 2)

        # Kalman ileriye tahmin
        if cfg.SHOW_PREDICTION and sys_.kalman and sys_.kalman.initialized:
            preds = sys_.kalman.predict_ahead(cfg.PREDICTION_STEPS)
            pts = [(int(p[0]), int(p[1])) for p in preds]
            prev = (cx, cy)
            for pt in pts:
                cv2.line(vis, prev, pt, cfg.COLOR_PREDICTION, 1)
                cv2.circle(vis, pt, 3, cfg.COLOR_PREDICTION, -1)
                prev = pt

        # Hata değerleri
        hata_x = cx - cx_img
        hata_y = cy - cy_img
        cv2.putText(vis,
                    f"HEDEF BULUNDU  hata=({hata_x:+d}, {hata_y:+d}) px",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    cfg.COLOR_BBOX, 2)
    else:
        cv2.putText(vis, "HEDEF YOK — Tarama...",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 0, 255), 2)

    # Görüntü merkezi crosshair
    cv2.line(vis, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (128, 128, 128), 1)
    cv2.line(vis, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (128, 128, 128), 1)

    return vis
