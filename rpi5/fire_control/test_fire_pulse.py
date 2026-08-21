#!/usr/bin/env python3
"""Pi-only ateş testi — KEY-style: tek FIRE=1 kenarı, STM 180ms pulse.

fire_control KAPALI iken:
  python3 -m rpi5.fire_control.test_fire_pulse
"""
from __future__ import annotations

import argparse
import sys
import time

try:
    from .protocol import DownlinkCommand
    from .uart_bridge import Stm32Bridge
except ImportError:
    from protocol import DownlinkCommand
    from uart_bridge import Stm32Bridge


def main() -> int:
    ap = argparse.ArgumentParser(description="UART KEY-style ateş testi")
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print(f"[TEST] UART {args.port} — FIRE 0→1 kenarı (STM ~180ms)")
    bridge = Stm32Bridge(port=args.port, baud=args.baud)
    try:
        off_cmd = DownlinkCommand(
            pan_deg=90.0,
            tilt_deg=80.0,
            fire=False,
            arm=True,
            heartbeat=True,
            home=False,
            safe=False,
            enable=True,
            stage=1,
        )
        on_cmd = DownlinkCommand(
            pan_deg=90.0,
            tilt_deg=80.0,
            fire=True,
            arm=True,
            heartbeat=True,
            home=False,
            safe=False,
            enable=True,
            stage=1,
        )

        for _ in range(5):
            bridge.send(off_cmd, min_period_s=0.0)
            bridge.poll()
            time.sleep(0.02)

        print("[TEST] FIRE=1")
        bridge.send(on_cmd, min_period_s=0.0)
        time.sleep(0.25)
        for _ in range(3):
            bridge.send(off_cmd, min_period_s=0.0)
        print("[TEST] bitti — röle ~180ms tıkladı mı?")
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    sys.exit(main())
