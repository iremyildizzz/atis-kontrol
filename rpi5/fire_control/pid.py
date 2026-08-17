"""PID controller — KTR Şekil 4.9 (RPi5 üzerinde çalışır).

Yön aynı kalır; merkeze yakınken çıkış kısılır ki balonu geçmeden
kilit bandında (±70 px) birkaç kare dursun.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDGains:
    kp: float = 0.035
    ki: float = 0.002
    kd: float = 0.008
    integral_limit: float = 200.0
    output_limit: float = 4.0  # derece / adım — aşırı geçmeyi kes
    d_limit: float = 30.0  # ham türev tavanı (°/s)
    near_err_deg: float = 1.5  # bu altında soft-land
    near_scale: float = 0.45


class PID:
    def __init__(self, gains: PIDGains | None = None) -> None:
        self.gains = gains or PIDGains()
        self._i = 0.0
        self._prev_err = 0.0
        self._has_prev = False

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

    def step(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        g = self.gains
        self._i += error * dt
        self._i = max(-g.integral_limit, min(g.integral_limit, self._i))

        d = 0.0
        if self._has_prev:
            d = (error - self._prev_err) / dt
            d = max(-g.d_limit, min(g.d_limit, d))
        self._prev_err = error
        self._has_prev = True

        out = g.kp * error + g.ki * self._i + g.kd * d

        # Merkeze yakın: geçmeyi azalt → kilit için ortada kal
        lim = g.output_limit
        if abs(error) <= g.near_err_deg:
            out *= g.near_scale
            lim = max(0.4, g.output_limit * 0.5)

        return max(-lim, min(lim, out))
