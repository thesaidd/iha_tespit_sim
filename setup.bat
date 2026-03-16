@echo off
echo ===================================================
echo  Tübitak Sim - Kurulum
echo ===================================================

REM Model dosyasını kopyala
if exist "..\yolo11m-uav.pt" (
    copy "..\yolo11m-uav.pt" "yolo11m-uav.pt"
    echo [OK] Model kopyalandi: yolo11m-uav.pt
) else (
    echo [UYARI] yolo11m-uav.pt bulunamadi!
    echo         detection_core\config.py dosyasindaki MODEL_PATH'i guncelleyin.
)

REM Bağımlılıkları yükle
echo.
echo Bagimliliklar yukleniyor...
pip install -r requirements.txt

echo.
echo ===================================================
echo  KURULUM TAMAMLANDI
echo ===================================================
echo.
echo Test calistirmak icin:
echo   python sim_test.py --source 0        ^(webcam^)
echo   python gazebo_test.py                ^(sentetik frame^)
echo   python gazebo_test.py --source 0     ^(webcam ile PID gosterimi^)
echo.
pause
