# Tübitak — Simülasyon Versiyonu

> YOLO11 + OpenCV Tracker + Kalman Filter  
> Gazebo / ROS2 entegrasyonuna hazır tek-fonksiyon arayüzü

---

## 📁 Klasör Yapısı

```
Tübitak_sim/
│
├── nesne_tespit.py          ★ ANA ENTEGRASYON NOKTASI
│
├── detection_core/          (iç modüller — doğrudan dokunmayın)
│   ├── detector.py          YOLO11 tespit motoru
│   ├── tracker.py           OpenCV RobustTracker (MOSSE/KCF/CSRT)
│   ├── kalman_filter.py     Konum yumuşatma + ileriye tahmin
│   └── config.py            ★ Tüm ayarlar buradan değiştirilir
│
├── sim_test.py              Webcam/video ile yerel test
├── gazebo_test.py           Gazebo bağlantısız standalone test (PID simülasyonu)
├── ros2_node.py             ROS2 Node (Gazebo entegrasyonu)
│
├── yolo11m-uav.pt           Model dosyası (setup.bat ile kopyalanır)
├── requirements.txt
└── setup.bat
```

---

## ⚡ Hızlı Başlangıç

### 1. Kurulum

```bash
# Windows
setup.bat

# Linux/Mac
pip install -r requirements.txt
cp ../yolo11m-uav.pt .
```

### 2. Test (Gazebo olmadan)

```bash
# Webcam ile test
python sim_test.py --source 0

# Sentetik "lider İHA" simülasyonu + PID kontrolü
python gazebo_test.py

# Video dosyası ile
python sim_test.py --source ornek.mp4
```

---

## 🔗 Simülatöre Entegrasyon

Simülatör kodunuzdan sadece bu iki satır yeterli:

```python
from nesne_tespit import nesne_tespit_sistemi

is_detected, cx, cy = nesne_tespit_sistemi(frame)
```

| Parametre    | Tip        | Açıklama |
|-------------|-----------|----------|
| `frame`     | `np.ndarray` | Kameradan gelen BGR görüntüsü |
| `is_detected` | `bool`   | `True` = hedef var |
| `cx`        | `int`     | Hedefin X piksel koordinatı (0 = sol kenar) |
| `cy`        | `int`     | Hedefin Y piksel koordinatı (0 = üst kenar) |

### Hata Hesaplama (PID için)

```python
is_detected, cx, cy = nesne_tespit_sistemi(frame)

if is_detected:
    # Görüntü merkezinden sapma
    hata_x = cx - frame.shape[1] // 2   # + sağda, - solda
    hata_y = cy - frame.shape[0] // 2   # + aşağıda, - yukarıda

    # PID kontrolcünüze verin:
    roll_komutu  = pid_x.compute(hata_x)
    pitch_komutu = pid_y.compute(hata_y)
```

---

## 🤖 ROS2 / Gazebo Entegrasyonu

### Yayınlanan Konular

| Konu | Mesaj Tipi | Açıklama |
|------|-----------|----------|
| `/target/is_detected` | `std_msgs/Bool` | Hedef var mı? |
| `/target/center` | `std_msgs/Int32MultiArray` | `[cx, cy]` |
| `/target/debug_image` | `sensor_msgs/Image` | Görsel çıktı |

### Node Başlatma

```bash
# ROS2 ortamında
ros2 run uav_detection ros2_node

# veya doğrudan
python ros2_node.py
```

Node otomatik olarak `/camera/image_raw` konusuna abone olur.

---

## ⚙️ Ayarlar

`detection_core/config.py` dosyasındaki değerleri değiştirerek sistemi ayarlayın:

```python
# Model
MODEL_PATH     = 'yolo11m-uav.pt'
CONF_THRESHOLD = 0.30        # Güven eşiği (düşürünce daha fazla tespit)

# Tracker hızı
TRACKER_TYPE   = 'MOSSE'     # 'MOSSE'=hızlı | 'KCF'=dengeli | 'CSRT'=hassas

# Kalman
USE_KALMAN     = True        # Konum yumuşatma açık/kapalı

# Yeniden doğrulama
REINIT_INTERVAL = 30         # Her 30 frame'de bir YOLO ile kontrol
```

---

## 🔬 Sistem Mimarisi

```
frame (BGR)
    │
    ▼
[UAVDetector]  ─── YOLO11 inference ───► tespitler listesi
    │
    ▼
[RobustTracker]  ─── OpenCV MOSSE ──►  bbox (x1,y1,x2,y2) 
    │                (her frame)
    ▼
[UAVKalmanFilter]  ── gürültü süz ──►  cx_filtered, cy_filtered
    │
    ▼
return (is_detected: bool, cx: int, cy: int)
```

**Akıllı Mod Geçişi:**
- İlk tespitte → YOLO çalışır, Tracker başlatılır
- Sonraki framelerde → sadece Tracker çalışır (hızlı!)
- Her `REINIT_INTERVAL` frame'de → YOLO tekrar doğrular
- Tracker kaybederse → otomatik tespit moduna döner

---

## 📋 Gereksinimler

```
ultralytics >= 8.0.0   (YOLO)
opencv-python >= 4.8.0
numpy >= 1.24.0
filterpy >= 1.4.5      (Kalman)
```

ROS2 entegrasyonu için ek olarak:
```
rclpy
cv_bridge
sensor_msgs
std_msgs
```

---

## ℹ️ Orijinal Proje

Bu klasör `Tübitak/` projesinin simülasyon entegrasyonuna özel alt kümesidir.  
Orijinal proje: test logger, ekran yakalama, gelişmiş görselleştirme ve  
arşivleme özellikleri içerir. Bu versiyonda sadece tespit çekirdeği vardır.
