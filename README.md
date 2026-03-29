<div align="center">
  <h1>🚀 Sabit Kanatlı İHA Otonom Takip Sistemi</h1>
  <p><b>Gelişmiş Taktiksel Güdüm (Predictive Tracking), TECS ve YOLO Tabanlı Görsel Savaş Vizörü (OSD) İçeren ROS 2 - PX4 Arayüzü</b></p>

  [![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg?logo=ros)](https://docs.ros.org/en/humble/index.html)
  [![PX4](https://img.shields.io/badge/PX4-Autopilot-154df0.svg?logo=px4)](https://px4.io/)
  [![OpenCV](https://img.shields.io/badge/OpenCV-Python-5C3EE8.svg?logo=opencv)](https://opencv.org/)
  [![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange.svg?logo=gazebo)](https://gazebosim.org/home)

</div>

## 📖 Proje Özeti
Bu proje, ROS 2 (Humble) ve PX4 DDS (MicroXRCEAgent) mimarisi üzerine kurulu, askeri standartlarda otonom bir **Sabit Kanatlı İHA İt Dalaşı (Dogfight) ve Gözlem** sistemidir. 

**Reaktif** (kaçıp kovalayan) sistemlerin hantallığını aşmak için; bu mimaride havacılık literatüründeki **CTRV (Constant Turn Rate and Velocity)** öngörü modelleri ve **L1 Vektör Güdümü** uygulanmıştır. Sistem, hedef kameradan kopsa bile GPS üzerinden hedefin o anki durumuna göre 2 saniye sonra nerede olacağını hesaplayıp o noktaya kilitlenir.

---

## 🌟 Öne Çıkan Özellikler

### 🎯 1. Predictive Trajectory Tracking (Kestirimci Güdüm)
Sistem hedefin *şu anki* değil, hızını ve dönüş oranını (Yaw Rate) analiz ederek **gelecekteki** rotasını tahmin eder. Hedef manevra yaptığında avcı İHA virajı dışarıdan almak yerine, merminin gideceği yeri hesaplarcasına doğrudan hedefin önüne/merkezine uçar.

### 🦅 2. Pure Pursuit & L1 Vektör Rotalaması
Kol uçuşunu (yan yana uçmayı) engelleyen ve burnu doğrudan hedefin merkezine yönelten "Odaklı" (Look-At) ağırlıklandırma katsayısı (%95) kullanılmıştır. Uçak, Hedefin rotasına sızarken keskin savrulmaları önleyen L1 algoritması ile pamuk gibi yumuşak bir giriş yapar (Bumpless Transfer).

### 🕹️ 3. TECS (Total Energy Control System)
Avcı İHA'nın yüksek irtifadan dalarken yerçekimi yüzünden kontrolden çıkmasını önleyen basitleştirilmiş bir enerji yönetim sistemidir. Uçak hedefin altında/üstünde kalarak sadece "Elevatör" (burun) hareketleri değil, gazı (Thrust) irtifa ve çekime göre **dinamik olarak (%0.05 - %0.85)** ayarlar.

### 🛑 4. Aktif Hava Freni (Airbrake) & Tailgating Koruması
Overshoot (Öne geçme) probleminin son noktası! Uçak hedefe 60 metreden fazla yaklaşıp hızı 60 km/h sınırını aşarsa sistem gazı doğrudan **0.0 (Tamamen Kapalı)** konumuna çeker. Uçak hava sürtünmesiyle güvenli 50 metre Trail (Kuyruk) mesafesine oturana dek süzülür. Eğer 90 derecelik açısal taşma olursa hedefi beklemek için tam kanat kırıp pürüzsüz **Loiter (Bekleme Dairesi)** çizer.

### 🖥️ 5. YOLO & Savaş Vizörü (HUD / OSD)
Otonomi yapısının ne düşündüğünü kameradan interaktif görmek için OpenCV tabanlı `yolo_arayuz.py` düğümü geliştirildi.
* 🔻 **[ Kırmızı ]**: Hedef kameradan (FOV) kopuksa uyarı verir.
* 🟨 **[ TESPİT EDİLDİ ]**: Hedef vizöre ilk 1 saniye girdiğinde sarı renkli dinamik kutu atar.
* 🟩 **[ TAKİP EDİLİYOR ]**: 1 saniyeden uzun kesintisiz takiplerde kilitlenir yeşil vizöre geçer.

### 📊 6. Telemetri Analizi
Uçağa müdahale etmeyen pasif dinleyici `ucus_loglari.py`, DDS QoS profillerini (Best-Effort) sömürerek Quaternion matematiklerinden Euler Rolllarına, uçağın NED (North-East-Down) hız asimptotlarına kadar tüm anlık telemetriyi **2 Hz frekans ile temiz HUD formatında yansıtır.**

---

## 🛠️ Kurulum

```bash
# 1. Projeyi klonlayın
git clone https://github.com/thesaidd/iha_tespit_sim.git
cd iha_tespit_sim

# 2. Gerekli kütüphaneleri yükleyin (Eğer yoksa)
sudo apt install ros-humble-cv-bridge
pip install opencv-python numpy
```

*(Sistemin PX4 Sitl, MicroXRCEAgent ve ROS 2 ortamlarına uygun olarak derlendiği varsayılmıştır.)*

---

## 🚀 Sistemi Çalıştırma (Adım Adım Gece Uçuşu)

Sistemi ateşlemek için terminalleri sırasıyla açın:

**Terminal 1:** (DDS Köprüsü)
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2:** (Avcı İHA - Hunter)
```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=default PX4_GZ_POSE="0,0,0.5,0,0,0" PX4_SYS_AUTOSTART=4008 make px4_sitl gz_advanced_plane
# (Açıldıktan sonra DDS köprüsünü çalıştırın)
# pxh> uxrce_dds_client start -t udp -p 8888
```

**Terminal 3:** (Hedef İHA - 50m İleride)
```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=default PX4_GZ_POSE="50,0,0.5,0,0,0" PX4_SYS_AUTOSTART=4008 instance=1 ./build/px4_sitl_default/bin/px4 -i 1
# pxh> uxrce_dds_client start -t udp -p 8888
```

**Terminal 4:** (Kamera Bridge İletişimi)
```bash
source ~/ros2_ws/install/setup.bash
# Konfigürasyonu kendi sisteminizin Gazebo yapısına göre ayarlayın.
ros2 run ros_gz_bridge parameter_bridge "/world/default/model/advanced_plane_0/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" --ros-args -r /world/default/model/advanced_plane_0/link/base_link/sensor/camera/image:=/uav0/camera
```

**Terminal 5:** (Otonom Güdüm Zekası)
```bash
cd ~/iha_tespit_sim
source ~/ros2_ws/install/setup.bash
python3 tracking_control.py
```

**Terminal 6:** (Savaş Vizörü OSD)
```bash
cd ~/iha_tespit_sim
source ~/ros2_ws/install/setup.bash
python3 yolo_arayuz.py
```

**Terminal 7:** (Telemetri Analiz - İsteğe Bağlı)
```bash
cd ~/iha_tespit_sim
source ~/ros2_ws/install/setup.bash
python3 ucus_loglari.py
```

<div align="center">
  <br>
  <i>Yusuf | Geliştirici Tarafından ROS 2 ile Hayata Geçirilmiştir.</i>
</div>
