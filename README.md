# Atış Kontrol (RPi5 + STM32F411)

Bu depo yalnızca **atış kontrol** yazılımını içerir:

| Birim | Görev |
|--------|--------|
| **Raspberry Pi 5** | TCP komut alma, PID, LiDAR menzil, UART binary |
| **STM32F411** | Servo PWM + MOSFET tetik, failsafe |

Görüntü işleme / arayüz bu repoda yoktur.

## Pinler

- **PA6** → X servo (pan)
- **PA7** → Y servo (tilt)
- **PB1** → Tetik (IRLZ44N)
- **PA9 / PA10** → USART1 ↔ RPi UART

## Klasörler

- `rpi5/` — Python atış kontrol servisi
- `stm32f411/` — C firmware
- `PROTOCOL.md` — UART + TCP mesaj formatı

## Çalıştırma (RPi)

```bash
cd rpi5
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m fire_control.main --tcp-port 5005 --stm-port /dev/ttyAMA0 --lidar-port /dev/ttyAMA4
```
