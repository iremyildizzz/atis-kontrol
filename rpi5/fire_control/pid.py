"""PID + yumuşak takip — KTR Şekil 4.9 (RPi5).

Çıkış: açı hızı (°/s). Uygulayan `out * dt` ile pan/tilt günceller.

Uygulananlar (checklist):
  1) Hata LPF (EMA)
  2) Türev LPF (Δe filtresi) — ham D sıçraması yok
  3) Integral windup clamp (I genelde 0)
  4) Slew / ivme tavanı — komut hızı bir anda ters dönmez
  5) Velocity feedforward (Kv) — hareketli balonu gecikmeden takip
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDGains:
    # Daha hızlı ama yumuşak: P orta, D çok küçük+filtreli, I=0, Kv feedforward
    kp: float = 0.18
    ki: float = 0.0
    kd: float = 0.004
    kv: float = 0.45          # feedforward: balon açısal hızı (°/s hatası)
    integral_limit: float = 12.0
    output_limit: float = 14.0  # °/s — üst hız
    error_limit: float = 5.0
    deadband: float = 0.25
    d_limit: float = 20.0       # ham türev tavanı (°/s)
    err_alpha: float = 0.28     # hata LPF
    d_alpha: float = 0.25       # türev LPF (α≈0.2–0.3)
    accel_limit: float = 45.0   # °/s² — yumuşak ama çevik


class PID:
    def __init__(self, gains: PIDGains | None = None) -> None:
        self.gains = gains or PIDGains()
        self._i = 0.0
        self._prev_err = 0.0
        self._has_prev = False
        self._err_f = 0.0
        self._d_f = 0.0
        self._rate_cmd = 0.0

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
        self._has_prev = False
        self._err_f = 0.0
        self._d_f = 0.0
        self._rate_cmd = 0.0

    def step(self, error: float, dt: float) -> float:
        """Hata (°) → yumuşak + hızlı komut hızı (°/s)."""
        if dt <= 0.0:
            return 0.0

        g = self.gains
        raw = max(-g.error_limit, min(g.error_limit, float(error)))

        # --- 1) Hata LPF ---
        ae = max(0.05, min(1.0, g.err_alpha))
        self._err_f = (1.0 - ae) * self._err_f + ae * raw
        err = self._err_f

        if abs(err) < g.deadband:
            err = 0.0
            self._i *= 0.85
            self._rate_cmd *= 0.88
            self._d_f *= 0.85

        # --- I (windup clamp); hareketli hedefte genelde 0 ---
        self._i += err * dt
        self._i = max(-g.integral_limit, min(g.integral_limit, self._i))

        # --- 2) Türev + LPF (ham Δe değil) ---
        d_raw = 0.0
        if self._has_prev:
            d_raw = (err - self._prev_err) / dt
            d_raw = max(-g.d_limit, min(g.d_limit, d_raw))
        ad = max(0.05, min(1.0, g.d_alpha))
        self._d_f = (1.0 - ad) * self._d_f + ad * d_raw
        self._prev_err = err
        self._has_prev = True

        # --- P + I + D_filt ---
        # D burada "fren": hatanın değişimine karşı (klasik)
        # --- 5) Velocity feedforward: hata büyüme yönünde balonu yakala ---
        # d_f > 0 → hedef merkezden uzaklaşıyor → aynı yönde hız ekle
        ff = g.kv * self._d_f

        desired = (
            g.kp * err
            + g.ki * self._i
            + g.kd * self._d_f
            + ff
        )
        desired = max(-g.output_limit, min(g.output_limit, desired))

        # --- 3/4) Slew / ivme limiti ---
        max_delta = g.accel_limit * dt
        delta = max(-max_delta, min(max_delta, desired - self._rate_cmd))
        self._rate_cmd += delta
        self._rate_cmd = max(-g.output_limit, min(g.output_limit, self._rate_cmd))
        return self._rate_cmd
