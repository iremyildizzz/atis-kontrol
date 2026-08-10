"""Aşama-3 hedef sınıfına göre puanlı imha mesafeleri."""
from __future__ import annotations

from typing import Optional

# (min_m, max_m) — inclusive. Daha yakında imha puan sayılmaz.
ENGAGE_RANGE_M: dict[str, tuple[float, float]] = {
    "f16": (10.0, 15.0),
    "jet": (10.0, 15.0),
    "ucak": (10.0, 15.0),
    "helikopter": (5.0, 15.0),
    "heli": (5.0, 15.0),
    "helicopter": (5.0, 15.0),
    "balistik": (5.0, 15.0),
    "fuze": (5.0, 15.0),
    "füze": (5.0, 15.0),
    "missile": (5.0, 15.0),
    "roket": (5.0, 15.0),
    "mini": (0.0, 15.0),
    "micro": (0.0, 15.0),
    "iha": (0.0, 15.0),
    "uav": (0.0, 15.0),
    "mini_iha": (0.0, 15.0),
    "micro_iha": (0.0, 15.0),
    "mini-micro-iha": (0.0, 15.0),
}

# gokhisar config.CLASS_NAMES ile aynı
CLASS_ID_ALIASES: dict[int, str] = {
    0: "fuze",        # balistik füze → 5–15 m
    1: "helikopter",  # 5–15 m
    2: "iha",         # mini/micro → 0–15 m
    3: "ucak",        # F16 → 10–15 m
    4: "balon",       # doğrudan imha hedefi değil
}


def normalize_class_name(name: str) -> str:
    s = name.strip().lower().replace("ı", "i").replace("ü", "u").replace("ö", "o")
    s = s.replace(" ", "_").replace("-", "_")
    return s


def resolve_target_class(class_name: str = "", class_id: int = -1) -> str:
    if class_name:
        key = normalize_class_name(class_name)
        # alt string eşleşmeleri
        for alias in ENGAGE_RANGE_M:
            if alias in key or key in alias:
                return alias
        if key in ENGAGE_RANGE_M:
            return key
        return key
    if class_id in CLASS_ID_ALIASES:
        return CLASS_ID_ALIASES[class_id]
    return ""


def engage_range_for(class_name: str = "", class_id: int = -1) -> Optional[tuple[float, float]]:
    """Sınıfa özel (min,max) metre; bilinmiyorsa None."""
    key = resolve_target_class(class_name, class_id)
    if key in ENGAGE_RANGE_M:
        return ENGAGE_RANGE_M[key]
    # kısmi eşleşme
    for alias, rng in ENGAGE_RANGE_M.items():
        if alias in key or key in alias:
            return rng
    return None


def distance_allows_fire(
    stage: int,
    distance_m: Optional[float],
    class_name: str = "",
    class_id: int = -1,
    *,
    require_lidar_stage3: bool = True,
) -> tuple[bool, str]:
    """
    Aşama-3: sınıf menzili zorunlu.
    Dönüş: (izin, neden)
    """
    if stage < 3:
        # Aşama-1/2: sınıf menzil kuralı yok
        return True, "stage_lt_3"

    if distance_m is None:
        if require_lidar_stage3:
            return False, "no_lidar"
        return True, "lidar_optional"

    rng = engage_range_for(class_name, class_id)
    if rng is None:
        # Bilinmeyen sınıf: güvenli üst sınır 0-15
        lo, hi = 0.0, 15.0
        reason_cls = "unknown_class_default_0_15"
    else:
        lo, hi = rng
        reason_cls = resolve_target_class(class_name, class_id)

    if lo <= distance_m <= hi:
        return True, f"in_range:{reason_cls}:{lo}-{hi}"
    return False, f"out_of_range:{reason_cls}:{distance_m:.2f} not in {lo}-{hi}"
