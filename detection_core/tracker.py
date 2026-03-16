"""
OpenCV Tracker Modülü
Simülasyon versiyonu - sadece takip çekirdeği
"""

import cv2
import numpy as np
from collections import deque
from typing import Optional, Tuple, List, Dict


class RobustTracker:
    """
    Stabilite artırıcı özelliklerle OpenCV tracker.
    MOSSE (hızlı), KCF (dengeli), CSRT (hassas) destekler.
    """

    TRACKER_TYPES = {
        'CSRT' : cv2.TrackerCSRT_create,
        'KCF'  : cv2.TrackerKCF_create,
        'MOSSE': cv2.legacy.TrackerMOSSE_create,
        'MIL'  : cv2.TrackerMIL_create,
    }

    def __init__(self,
                 tracker_type: str = 'MOSSE',
                 trajectory_length: int = 15,
                 max_bbox_change: float = 0.5,
                 min_bbox_area: int = 50):
        """
        Args:
            tracker_type    : 'MOSSE' | 'KCF' | 'CSRT' | 'MIL'
            trajectory_length: İz uzunluğu (frame)
            max_bbox_change : Maksimum bbox değişim oranı (0-1)
            min_bbox_area   : Minimum bbox alanı (piksel²)
        """
        if tracker_type not in self.TRACKER_TYPES:
            raise ValueError(f"Geçersiz tracker: {tracker_type}")

        self.tracker_type = tracker_type
        self.tracker = None
        self.is_tracking = False
        self.trajectory = deque(maxlen=trajectory_length)
        self.max_bbox_change = max_bbox_change
        self.min_bbox_area = min_bbox_area

        self.last_bbox = None
        self.last_center = None
        self.frames_tracked = 0
        self.frames_lost = 0
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3
        self.bbox_history: List[Tuple] = []

        print(f"[Tracker] {tracker_type} hazır")

    # ------------------------------------------------------------------
    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """Tracker'ı başlat. bbox = (x1, y1, x2, y2)"""
        self.tracker = self.TRACKER_TYPES[self.tracker_type]()

        x1, y1, x2, y2 = bbox
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)
        x, y, w, h = x1, y1, x2 - x1, y2 - y1

        if w < 5 or h < 5:
            return False

        # Frame sınırı düzeltmesi
        fh, fw = frame.shape[:2]
        x = max(0, min(x, fw - 1))
        y = max(0, min(y, fh - 1))
        w = min(w, fw - x)
        h = min(h, fh - y)

        try:
            success = self.tracker.init(frame, (x, y, w, h))
        except Exception as e:
            print(f"[Tracker] init hatası: {e}")
            return False

        if success:
            self.is_tracking = True
            self.last_bbox = (x, y, x + w, y + h)
            self.last_center = (x + w // 2, y + h // 2)
            self.trajectory.clear()
            self.trajectory.append(self.last_center)
            self.bbox_history = [self.last_bbox]
            self.consecutive_failures = 0

        return success

    # ------------------------------------------------------------------
    def update(self, frame: np.ndarray) -> Tuple[bool, Optional[Tuple]]:
        """
        Tracker'ı güncelle.

        Returns:
            (success: bool, bbox: (x1,y1,x2,y2) | None)
        """
        if not self.is_tracking or self.tracker is None:
            return False, None

        success, bbox_xywh = self.tracker.update(frame)

        if success:
            x, y, w, h = [int(v) for v in bbox_xywh]
            bbox = (x, y, x + w, y + h)

            # Validasyon
            if not self._is_valid(bbox, frame.shape):
                return self._fail()

            if not self._change_ok(bbox):
                return self._fail()

            # Temporal smoothing
            bbox = self._smooth(bbox)

            center = ((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2)
            self.last_bbox = bbox
            self.last_center = center
            self.trajectory.append(center)
            self.bbox_history.append(bbox)
            if len(self.bbox_history) > 10:
                self.bbox_history.pop(0)
            self.frames_tracked += 1
            self.consecutive_failures = 0
            return True, bbox

        return self._fail()

    # ------------------------------------------------------------------
    def reset(self):
        """Tracker'ı sıfırla"""
        self.tracker = None
        self.is_tracking = False
        self.trajectory.clear()
        self.last_bbox = None
        self.last_center = None
        self.bbox_history = []
        self.consecutive_failures = 0

    def get_trajectory(self) -> List[Tuple]:
        return list(self.trajectory)

    def get_velocity(self) -> Optional[Tuple[float, float]]:
        if len(self.trajectory) < 2:
            return None
        p1, p2 = self.trajectory[-2], self.trajectory[-1]
        return (p2[0] - p1[0], p2[1] - p1[1])

    def get_stats(self) -> Dict:
        return {
            'tracker_type'  : self.tracker_type,
            'is_tracking'   : self.is_tracking,
            'frames_tracked': self.frames_tracked,
            'frames_lost'   : self.frames_lost,
            'last_center'   : self.last_center,
            'velocity'      : self.get_velocity(),
        }

    # ------------------------------------------------------------------  (private)
    def _fail(self) -> Tuple[bool, None]:
        self.consecutive_failures += 1
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.is_tracking = False
            self.frames_lost += 1
        return False, None

    def _is_valid(self, bbox, shape) -> bool:
        x1, y1, x2, y2 = bbox
        h, w = shape[:2]
        bw, bh = x2 - x1, y2 - y1
        if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
            return False
        if bw <= 0 or bh <= 0 or bw * bh < self.min_bbox_area:
            return False
        if bw * bh > w * h * 0.8:
            return False
        return True

    def _change_ok(self, new_bbox) -> bool:
        if self.last_bbox is None:
            return True
        x1o, y1o, x2o, y2o = self.last_bbox
        wo, ho = x2o - x1o, y2o - y1o
        x1n, y1n, x2n, y2n = new_bbox
        wn, hn = x2n - x1n, y2n - y1n
        if wo > 0 and abs(wn - wo) / wo > self.max_bbox_change:
            return False
        if ho > 0 and abs(hn - ho) / ho > self.max_bbox_change:
            return False
        cx_diff = abs((x1n + x2n) / 2 - (x1o + x2o) / 2)
        if cx_diff > max(wo, ho) * self.max_bbox_change:
            return False
        return True

    def _smooth(self, new_bbox) -> Tuple:
        recent = self.bbox_history[-3:] + [new_bbox]
        return (
            int(np.mean([b[0] for b in recent])),
            int(np.mean([b[1] for b in recent])),
            int(np.mean([b[2] for b in recent])),
            int(np.mean([b[3] for b in recent])),
        )
