"""
Kalman Filter Modülü
Simülasyon versiyonu - konum yumuşatma ve ileriye tahmin
"""

import numpy as np
from filterpy.kalman import KalmanFilter
from typing import Tuple, Optional, List


class UAVKalmanFilter:
    """İHA takibi için Kalman Filter — durum: [x, y, vx, vy]"""

    def __init__(self,
                 dt: float = 1.0,
                 process_noise: float = 1e-3,
                 measurement_noise: float = 1e-1):
        """
        Args:
            dt                : Zaman adımı
            process_noise     : Süreç gürültüsü (model belirsizliği)
            measurement_noise : Ölçüm gürültüsü (tespit belirsizliği)
        """
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # Durum geçiş matrisi: x_k+1 = F * x_k
        self.kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])

        # Ölçüm matrisi: z = H * x  (sadece konum)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self.kf.Q = np.eye(4) * process_noise
        self.kf.R = np.eye(2) * measurement_noise
        self.kf.P = np.eye(4) * 1000

        self.initialized = False

    # ------------------------------------------------------------------
    def init(self, x: float, y: float, vx: float = 0.0, vy: float = 0.0):
        """Filter'ı verilen konumla başlat."""
        self.kf.x = np.array([x, y, vx, vy])
        self.initialized = True

    def update(self, measurement: Tuple[float, float]) -> Tuple[float, float]:
        """
        Yeni ölçümü işle ve filtrelenmiş konumu döndür.

        Args:
            measurement: (x, y) ölçümü

        Returns:
            (x_filtered, y_filtered)
        """
        if not self.initialized:
            self.init(measurement[0], measurement[1])
            return measurement

        self.kf.predict()
        self.kf.update(np.array(measurement))
        return float(self.kf.x[0]), float(self.kf.x[1])

    def predict_ahead(self, steps: int = 5) -> List[Tuple[float, float]]:
        """
        N adım ilerisi için tahminler üret (state'i bozmadan).

        Returns:
            [(x, y), ...] listesi
        """
        if not self.initialized:
            return []

        saved_x = self.kf.x.copy()
        saved_P = self.kf.P.copy()

        predictions = []
        for _ in range(steps):
            self.kf.predict()
            predictions.append((float(self.kf.x[0]), float(self.kf.x[1])))

        self.kf.x = saved_x
        self.kf.P = saved_P
        return predictions

    def get_position(self) -> Tuple[float, float]:
        if not self.initialized:
            return 0.0, 0.0
        return float(self.kf.x[0]), float(self.kf.x[1])

    def get_velocity(self) -> Tuple[float, float]:
        if not self.initialized:
            return 0.0, 0.0
        return float(self.kf.x[2]), float(self.kf.x[3])

    def get_speed(self) -> float:
        vx, vy = self.get_velocity()
        return float(np.sqrt(vx**2 + vy**2))

    def reset(self):
        self.kf.P = np.eye(4) * 1000
        self.initialized = False
