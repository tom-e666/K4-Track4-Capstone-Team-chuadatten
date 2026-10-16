"""Part B: physics check. TODO 3."""
from __future__ import annotations

import math

G = 9.81


def is_feasible(p: dict) -> bool:
    mu = p["mu"]
    max_accel = mu * G
    a_lat = (math.pi / p["cutDur"]) ** 2 * 3.5 / 2.0
    if a_lat > max_accel:
        return False
    if p["cutBrake"] > max_accel:
        return False
    if p["pedV"] > 2.5:
        return False
    if not (0.0 < p["cutV"] <= 22.0):
        return False
    return True
