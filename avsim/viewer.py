"""avsim.viewer: write the file the 3D viewer's chapter 9 ("Phòng lab") opens.

The viewer re-simulates every scenario with its own copy of the core (bit-exact with Python), so the file
only carries parameters, never trajectories. A 5 000-point search cloud is a few hundred kB.
"""
from __future__ import annotations

import json
import math

import numpy as np

from . import space

FORMAT = "avsim-viewer/1"
CLOUD_NAMES = [c[0] for c in space.CONT if c[0] != "cutBrakeV"] + ["mu", "pedLook", "cutBrake"]


def _num(v, nd=None):
    """None/nan -> None. Runs keep full precision (the viewer must replay the exact scenario: a 4-decimal cut-in time
    can turn a crash into a miss); only the search cloud, which is just dots on a plot, is rounded (nd=4)."""
    if v is None:
        return None
    v = float(v)
    return None if math.isnan(v) else (v if nd is None else round(v, nd))


def export(path, runs, cloud=None, policy=None, baseline_policy=None, title=""):
    """
    runs:   list of dict(params=..., label="bus_bypass", id="B-03", outcome="crash"|"near"|"safe", others="reactive",
            hazards=[...] for a chapter-10 hazard scene)
    cloud:  optional list of dict(params=..., out="crash"|"near"|"safe"|"infeasible", it=search round, mode=label or None)
    policy: the ego policy knobs the runs should be replayed with (your fix); baseline_policy: what the ghost car uses.
    """
    doc = dict(format=FORMAT, title=title, policy=policy or {}, baseline_policy=baseline_policy or {}, runs=[])
    for r in runs:
        doc["runs"].append(dict(id=r.get("id", ""), label=r.get("label"), outcome=r.get("outcome", "crash"),
                                others=r.get("others", "reactive"), policy=r.get("policy"),
                                params={k: (_num(v) if not isinstance(v, bool) else v) for k, v in r["params"].items()},
                                **({"hazards": r["hazards"]} if r.get("hazards") else {})))
    if cloud:
        doc["cloud"] = dict(names=CLOUD_NAMES, x="cutT0", y="cutGap", fixed=dict(civil=True),
                            rows=[[_num(c["params"].get(n, 0.0), 4) for n in CLOUD_NAMES] for c in cloud],
                            out=[c["out"] for c in cloud], it=[int(c.get("it", 0)) for c in cloud],
                            mode=[c.get("mode") for c in cloud])
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, separators=(",", ":"))
    return doc
