# Raspberry Pi 5 — Atış Kontrol

Üst katmandan TCP JSON alır → PID / menzil kararı → STM32’ye UART binary gönderir.

## Kurulum

```bash
cd rpi5
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

UART: `raspi-config` → Serial Port açık, login shell kapalı.

Kablo:
- RPi TXD → STM32 PA10 (RX)
- RPi RXD → STM32 PA9 (TX)
- GND ortak
- LiDAR UART4 (`/dev/ttyAMA4`, GPIO33)

## Çalıştırma

```bash
python -m fire_control.main \
  --stm-port /dev/ttyAMA0 \
  --lidar-port /dev/ttyAMA4 \
  --tcp-port 5005 \
  --video-host 192.168.137.147 \
  --frame-w 640 \
  --frame-h 480
```

Varsayılan PID preset: **`iyi_yatay`**
- pan: `P=0.034 I=0 D=0.010`
- tilt: `P=0.018 I=0 D=0.022` (daha yumuşak + frenli; dikey kaçırmayı kesmek için)

```bash
# Preset açık (varsayılan)
python -m fire_control.main --pid-preset iyi_yatay ...

# Tilt’i daha da yumuşat
python -m fire_control.main --kp-tilt 0.014 --kd-tilt 0.028 ...

# FF (genelde kapalı; droop için)
python -m fire_control.main --tilt-gravity-kg 1.2 --tilt-gravity-mode cos ...
python -m fire_control.main --tilt-gravity-kg 0 ...

# Preset’siz
python -m fire_control.main --pid-preset none --kp 0.034 --kd 0.010 --kp-tilt 0.018 --kd-tilt 0.022 ...
```

Yerçekimi FF, PID state’ine birikmez; STM’ye giden `tilt_cmd = tilt + Kg·cos(elev)` (droop telafisi). Aksi halde her döngüde Δ’ya eklenen Kg ramp yapardı.

`--video-host` = **PC IP**. Kamera fire_control ile birlikte sürekli UDP:5000’e gider.
Arayüz Başlat = sadece dinler; Durdur = PC tarafını kapatır, Pi yayına devam eder.

Yatay servo tersliği tercihen PC `SERVO_INVERT_PAN` / `SERVO_INVERT_PAN_AUTO` ile.
RPi `--invert-x` yalnız PC invert yoksa.

Örnek giriş mesajları: `../PROTOCOL.md`
