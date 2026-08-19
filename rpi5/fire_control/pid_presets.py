"""Kayıtlı PID ayarları.

iyi_yatay — yatay kilit iyi (2026-08):
  pan  P=0.034  I=0  D=0.010
  tilt P daha düşük, D daha yüksek → dikey overshoot / kaçırmayı kes
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PidPreset:
    name: str
    kp: float
    ki: float
    kd: float
    # Dikey ayrı kazanç (None → pan ile aynı)
    kp_tilt: float | None = None
    ki_tilt: float | None = None
    kd_tilt: float | None = None
    # STM tilt komutuna droop ofseti (derece); overshoot için 0 tercih
    tilt_gravity_kg: float = 0.0
    tilt_gravity_mode: str = "cos"


# Ad: iyi_yatay — pan sert, tilt yumuşak+frenli
IYI_YATAY = PidPreset(
    name="iyi_yatay",
    kp=0.034,
    ki=0.0,
    kd=0.010,
    kp_tilt=0.018,
    ki_tilt=0.0,
    kd_tilt=0.022,
    tilt_gravity_kg=0.0,
    tilt_gravity_mode="cos",
)

PRESETS: dict[str, PidPreset] = {
    IYI_YATAY.name: IYI_YATAY,
}


def resolve_tilt_gains(
    kp: float,
    ki: float,
    kd: float,
    kp_tilt: float | None,
    ki_tilt: float | None,
    kd_tilt: float | None,
) -> tuple[float, float, float]:
    """Pan kazancından tilt kazancını çöz."""
    return (
        float(kp if kp_tilt is None else kp_tilt),
        float(ki if ki_tilt is None else ki_tilt),
        float(kd if kd_tilt is None else kd_tilt),
    )


def tilt_gravity_ff(tilt_deg: float, kg: float, mode: str = "cos") -> float:
    """STM komutuna yerçekimi ofseti (state'e birikmez)."""
    if kg == 0.0:
        return tilt_deg
    mode_l = (mode or "cos").lower()
    if mode_l == "const":
        return tilt_deg + kg
    elev_rad = math.radians(tilt_deg - 90.0)
    return tilt_deg + kg * math.cos(elev_rad)
