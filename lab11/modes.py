"""Part D: name the failure. TODO 8."""
from __future__ import annotations

from avsim import core


def label_mode(R: core.Rollout, k: int, event: dict, contact_type) -> str:
    other = event["b"] if event["a"] == "ego" else event["a"]
    if other == "P1":
        return "crosswalk_block"
    if contact_type(R, k, event) == "rear":
        return "rear_ended"
    if other == "C2":
        f = event["f"]
        c2 = R.F[k, f, core.IX["C2"]]
        if c2[0] < 112.0 and c2[4] < 6.0:
            return "bus_bypass"
        return "cut_in"
    return "other"
