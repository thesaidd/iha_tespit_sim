"""
Konfigurasyon - Simülasyon Versiyonu
Tüm ayarlar tek yerde.
"""

# ─── Model ───────────────────────────────────────────────────────────
MODEL_PATH      = 'yolo11m-uav.pt'   # .pt dosyasının yolu
CONF_THRESHOLD  = 0.30               # Güven eşiği (0-1)
IOU_THRESHOLD   = 0.45               # NMS IoU eşiği
DEVICE          = 'cpu'              # 'cpu' veya 'cuda'
TARGET_CLASSES  = [0]                # 0 = uav (özel model); None = hepsi

# ─── Tracker ─────────────────────────────────────────────────────────
TRACKER_TYPE      = 'MOSSE'   # MOSSE (hızlı) | KCF (dengeli) | CSRT (hassas)
REINIT_INTERVAL   = 30        # Kaç frame'de bir YOLO ile yeniden doğrulama
MAX_BBOX_CHANGE   = 0.5       # Tracker bbox stabilite eşiği
MIN_BBOX_AREA     = 50        # Minimum tespit alanı (piksel²)

# ─── Kalman Filter ───────────────────────────────────────────────────
USE_KALMAN        = True
KALMAN_Q          = 1e-3     # Süreç gürültüsü
KALMAN_R          = 1e-1     # Ölçüm gürültüsü
PREDICTION_STEPS  = 5        # Kaç adım ilerisi tahmin

# ─── Görselleştirme ──────────────────────────────────────────────────
TRAJECTORY_LENGTH = 15       # İz uzunluğu (frame sayısı)
SHOW_PREDICTION   = True     # Kalman tahminini göster?
SHOW_FPS          = True

# ─── Renk tanımları (BGR) ────────────────────────────────────────────
COLOR_BBOX        = (0,   255,  0)    # yeşil
COLOR_TRAJECTORY  = (255,  0,   0)   # mavi
COLOR_PREDICTION  = (0,    0, 255)   # kırmızı
COLOR_TEXT        = (255, 255, 255)  # beyaz
COLOR_CROSSHAIR   = (0,  255, 255)   # sarı
