"""PID — KTR Şekil 4.9 (RPi5).

Çıkış: bu adımda eklenecek açı (°).
output_limit: max hız (°/s) → adım tavanı = limit * dt.
D: filtrelenmiş türev (ham Δe/dt tekmesi yok).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDGains:
    # P/I/D varsayılan 0 — ekrandan gelir; output_limit sadece hız tavanı (°/s)
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 50.0
    output_limit: float = 80.0  # °/s güvenlik tavanı (panel P değil)
    derivative_filter: float = 0.7


class PID:
    def __init__(self, gains: PIDGains | None = None) -> None:
        self.gains = gains or PIDGains()
        self._i = 0.0
        self._prev_err = 0.0
        self._d_filtered = 0.0
        self._has_prev = False

    def set_gains(
        self,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        output_limit: float | None = None,
        reset: bool = True,
    ) -> None:
        if kp is not None:
            self.gains.kp = float(kp)
        if ki is not None:
            self.gains.ki = float(ki)
        if kd is not None:
            self.gains.kd = float(kd)
        if output_limit is not None:
            self.gains.output_limit = float(output_limit)
        if reset:
            self.reset()

    def reset(self) -> None:
        self._i = 0.0
        self._prev_err = 0.0
        self._d_filtered = 0.0
        self._has_prev = False

    def step(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        g = self.gains
        self._i += error * dt
        self._i = max(-g.integral_limit, min(g.integral_limit, self._i))

        d = 0.0
        if self._has_prev:
            raw_d = (error - self._prev_err) / dt
            alpha = max(0.0, min(0.95, g.derivative_filter))
            self._d_filtered = (
                alpha * self._d_filtered + (1.0 - alpha) * raw_d
            )
            d = self._d_filtered
        self._prev_err = error
        self._has_prev = True

        out = g.kp * error + g.ki * self._i + g.kd * d
        # output_limit = °/s → bu döngüdeki max adım
        max_step = g.output_limit * dt
        return max(-max_step, min(max_step, out))
