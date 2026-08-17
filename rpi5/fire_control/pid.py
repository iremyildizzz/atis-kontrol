"""PID controller — KTR Şekil 4.9 (RPi5).

Çıkış: bu kontrol adımında uygulanacak açı artımı (°).
İç model: hız (°/s); `output_limit` fiziksel max hızdır.

Panel Kp/Kd (eski “derece/adım @50 Hz”) → NOMINAL_DT ile °/s'e çevrilir,
böylece döngü 40–60 Hz salınsa bile aynı P sayısı aynı fiziksel davranışı verir.
"""
from __future__ import annotations

from dataclasses import dataclass

# Kazançların tanımlandığı referans periyot (UART / kontrol ≈ 50 Hz)
NOMINAL_DT = 0.02


@dataclass
class PIDGains:
    kp: float = 0.25
    ki: float = 0.0
    kd: float = 0.05
    integral_limit: float = 200.0
    output_limit: float = 100.0  # °/s — fiziksel hız tavanı
    derivative_filter: float = 0.7  # 1'e yakın = daha yumuşak D


class PID:
    def __init__(self, gains: PIDGains | None = None) -> None:
        self.gains = gains or PIDGains()
        self._i = 0.0
        self._prev_err = 0.0
        self._has_prev = False
        self._d_filtered = 0.0

    def set_gains(
        self,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        output_limit: float | None = None,
        reset: bool = True,
    ) -> None:
        """Arayüzden gelen P/I/D güncellemesi."""
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
        self._has_prev = False
        self._d_filtered = 0.0

    def step(self, error: float, dt: float) -> float:
        """Hata (°) → bu adımda eklenecek açı (°)."""
        if dt <= 0.0:
            return 0.0

        g = self.gains
        self._i += error * dt
        self._i = max(-g.integral_limit, min(g.integral_limit, self._i))

        d = 0.0
        if self._has_prev:
            raw_d = (error - self._prev_err) / dt
            alpha = max(0.0, min(0.95, g.derivative_filter))
            self._d_filtered = alpha * self._d_filtered + (1.0 - alpha) * raw_d
            d = self._d_filtered
        self._prev_err = error
        self._has_prev = True

        # Eski panel birimleri (derece/adım @50 Hz) → °/s
        cmd = g.kp * error + g.ki * self._i + g.kd * d
        rate = cmd / NOMINAL_DT
        rate = max(-g.output_limit, min(g.output_limit, rate))
        return rate * dt
