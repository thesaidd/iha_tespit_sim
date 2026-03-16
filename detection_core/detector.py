"""
YOLOv8 / YOLO11 Tespit Modülü
Simülasyon versiyonu - sadece tespit çekirdeği
"""

from ultralytics import YOLO
import cv2
import numpy as np
from typing import List, Optional, Dict


class UAVDetector:
    """YOLOv8/YOLO11 tabanlı İHA tespit sınıfı"""

    def __init__(self,
                 model_path: str = 'yolo11m-uav.pt',
                 conf_threshold: float = 0.3,
                 iou_threshold: float = 0.45,
                 device: str = 'cpu',
                 target_classes: Optional[List[int]] = None):
        """
        Args:
            model_path     : Model dosya yolu (.pt)
            conf_threshold : Güven eşiği
            iou_threshold  : NMS IoU eşiği
            device         : 'cpu' veya 'cuda'
            target_classes : Tespit edilecek sınıf ID'leri (None = hepsi)
        """
        print(f"[Detector] Model yükleniyor: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        self.target_classes = target_classes
        self.class_names = self.model.names
        print(f"[Detector] Hazır | device={device} | conf={conf_threshold}")

    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Frame'de nesne tespiti yap.

        Args:
            frame: BGR formatında numpy dizisi

        Returns:
            Liste[dict] → her biri:
              {
                'bbox'  : (x1, y1, x2, y2),
                'conf'  : float,
                'class' : int,
                'name'  : str,
                'center': (cx, cy)
              }
        """
        results = self.model(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )

        detections = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                if self.target_classes and class_id not in self.target_classes:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                detections.append({
                    'bbox'  : (int(x1), int(y1), int(x2), int(y2)),
                    'conf'  : confidence,
                    'class' : class_id,
                    'name'  : self.class_names[class_id],
                    'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                })

        return detections

    def get_best_detection(self, detections: List[Dict]) -> Optional[Dict]:
        """En yüksek güven skoruna sahip tespiti döndür."""
        if not detections:
            return None
        return max(detections, key=lambda x: x['conf'])
