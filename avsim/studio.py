"""avsim.studio: read the file the 3D viewer's chapter 10 ("xưởng kịch bản") saves, and test a hazard at scale.

A studio file (format "avsim-studio/1") holds one concrete scene: world params, policy, hazard cards, and, if the
student drove, the pedal input. To ask "how often does this hurt?", the student adds two things to it by hand:

    "space":    {"h0_x": [120, 260], "h0_trig": [8, 40]}   ranges the hazard's numbers vary over (uniform), and
    "exposure": 0.002                                       how often this situation occurs per trip, with a reason in "why".

Everything here is given code, not a TODO.
"""
from __future__ import annotations

import json
import re

import numpy as np

from . import core

FORMAT = "avsim-studio/1"
# What a hazard's numbers may be, physically or by the sim's map. Outside these, the scene is not one the world allows.
LIMITS = {
    "ped": dict(x=(90, 320), v=(0.3, 3.5), t0=(0, 24), trig=(0, 120)),
    "stalled": dict(x=(-200, 320)),
    "brake": dict(decel=(0.5, 9.0), dur=(0.1, 10), t0=(0, 24), trig=(0, 120)),
    "bike": dict(x=(-200, 320), d=(-1.6, 1.6), v=(0.5, 12), swerve=(0, 4.5), swDur=(0.3, 5), t0=(0, 24), trig=(0, 120)),
}
# default ranges when the file has no "space": a neighbourhood of the scene the student built
SPREAD = dict(x=40.0, v=0.3, t0=3.0, trig=0.5, decel=2.0, dur=1.0, d=0.4, swerve=0.4, swDur=0.4)
RELATIVE = {"v", "trig", "swerve", "swDur"}


def load(path):
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("format") != FORMAT:
        raise ValueError(f"{path}: not an {FORMAT} file (save it from the viewer's chapter 10)")
    raw = doc.get("hazards", [])
    if not raw:
        raise ValueError(f"{path}: no hazards; add at least one in the studio")
    if len(raw) > 4:
        raise ValueError(f"{path}: {len(raw)} hazards; the studio allows at most 4")
    for i, h in enumerate(raw):          # the viewer and Python must read the file the same way: no guessing
        if h.get("type") not in LIMITS:
            raise ValueError(f"{path}: hazard {i} has unknown type {h.get('type')!r}")
        for f in LIMITS[h["type"]]:
            v = h.get(f)
            if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))):
                raise ValueError(f"{path}: hazard {i} field {f!r} must be a number, got {v!r}")
        if h["type"] == "ped" and h.get("t0", 0) is None:
            raise ValueError(f"{path}: hazard {i} t0 must be a number")
    if doc.get("ego") == "script":
        ea = doc.get("egoAcc")
        if not ea or any(isinstance(a, bool) or not isinstance(a, (int, float)) for a in ea):
            raise ValueError(f"{path}: egoAcc must be a non-empty list of numbers")
    doc["hazards"] = [core.hazard_full(h) for h in raw]
    return doc


BIKE_LAT_MAX = 3.0     # m/s^2: peak sideways acceleration of a hard emergency swerve on a bicycle


def bike_lat_acc(swerve, swDur):
    """Peak lateral acceleration of the cosine swerve d(u) = d0 - swerve*(1 - cos(pi u))/2, u = t/swDur."""
    return np.pi ** 2 * abs(swerve) / (2 * swDur ** 2)


def hazard_ok(hazards, params=None):
    """Is every hazard inside LIMITS (and, if params give per-scenario values, those too)? A brake beyond mu*g is not,
    nor is a bike swerve sharper than BIKE_LAT_MAX."""
    params = params or {}
    mu = params.get("mu", 0.8)
    for i, h in enumerate(hazards):
        val = lambda f: params.get(f"h{i}_{f}", h.get(f))          # noqa: E731
        for f, (lo, hi) in LIMITS[h["type"]].items():
            v = val(f)
            if v is None:
                continue
            if not lo <= v <= hi:
                return False
        if h["type"] == "brake" and val("decel") > mu * core.G + 1e-9:
            return False
        if h["type"] == "bike" and bike_lat_acc(val("swerve"), val("swDur")) > BIKE_LAT_MAX + 1e-9:
            return False
    return True


def check_space(doc):
    """`space` may only vary numeric fields of hazards that exist: keys "h{i}_{field}" with field in LIMITS for that
    hazard's type, each range [lo, hi] with lo <= hi inside the limits. Returns a list of problems (empty = fine)."""
    bad = []
    n = len(doc["hazards"])
    for key, rng in (doc.get("space") or {}).items():
        m = re.fullmatch(r"h(\d+)_(\w+)", key)
        if not m or int(m.group(1)) >= n:
            bad.append(f"{key}: not h<i>_<field> for one of your {n} hazards")
            continue
        h = doc["hazards"][int(m.group(1))]
        lim = LIMITS[h["type"]].get(m.group(2))
        if lim is None:
            bad.append(f"{key}: a {h['type']} has no numeric field {m.group(2)!r} (allowed: {', '.join(LIMITS[h['type']])})")
        elif not (isinstance(rng, list) and len(rng) == 2 and lim[0] <= rng[0] <= rng[1] <= lim[1]):
            bad.append(f"{key}: range {rng} must be [lo, hi] inside {list(lim)}")
    return bad


def default_space(doc):
    out = {}
    for i, h in enumerate(doc["hazards"]):
        for f, (lo, hi) in LIMITS[h["type"]].items():
            v = h.get(f)
            if v is None or f not in SPREAD:
                continue
            w = SPREAD[f] * (abs(v) if f in RELATIVE else 1.0)
            if w > 0:
                out[f"h{i}_{f}"] = [max(lo, v - w), min(hi, v + w)]
    return out


def sample_hazard_values(space, n, rng):
    """n dicts of per-scenario hazard overrides, uniform over each range."""
    keys = sorted(space)
    cols = {k: rng.uniform(space[k][0], space[k][1], n) for k in keys}
    return [{k: float(cols[k][j]) for k in keys} for j in range(n)]


def _first_seen(R: core.Rollout, j, start=None):
    """(K,) first frame (at or after `start`, default 0) the ego's roof sensor has a clear line of sight to actor j
    within 120 m; NF-1 = never."""
    F, ids = R.F, R.ids
    acts = R.actors or core.ACTORS
    veh = np.array([a.kind != "ped" for a in acts])
    out = np.full(R.K, core.NF - 1)
    L, W = np.array([a.dims[0] for a in acts]) / 2, np.array([a.dims[1] for a in acts]) / 2
    for f in range(core.NF):
        e, t = F[:, f, core.EGO], F[:, f, j]
        near = np.hypot(t[:, 0] - e[:, 0], t[:, 1] - e[:, 1]) <= 120
        hit = core.seg_hits_box(e[:, 0, None], e[:, 1, None], t[:, 0, None], t[:, 1, None],
                                F[:, f, :, 0], F[:, f, :, 1], F[:, f, :, 2], F[:, f, :, 3], L[None, :], W[None, :])
        hit[:, [core.EGO, j]] = False
        seen = near & ~np.any(hit & veh[None, :], axis=1)
        if start is not None:
            seen &= f >= start
        out = np.where((out == core.NF - 1) & seen, f, out)
        if (out < core.NF - 1).all():
            break
    return out


NEVER = -1     # fire frame of a hazard that never acted in that scenario


def fire_frames(R: core.Rollout, params=None):
    """(K, n_hazards) frame each hazard started acting, NEVER (-1) if it did not. Pedestrians, bikes and braking cars
    report it from the sim itself (R.fire); a stalled car has no start, so its frame is when the ego first could see it."""
    out = np.array(R.fire, copy=True) if R.fire is not None else np.full((R.K, len(R.hazards or [])), NEVER)
    for i, h in enumerate(R.hazards or []):
        if h["type"] == "stalled":
            fs = _first_seen(R, R.ids.index(f"H{i}"))
            out[:, i] = np.where(fs < core.NF - 1, fs, NEVER)
    return out


def blame(R: core.Rollout, k, ff=None):
    """Which hazard a scenario's first ego collision belongs to.

    i      = hazard i: it had started acting before the contact AND the ego hit it (or hit the car it made brake);
    -1     = unattributed: the ego hit something else, or the hazard had not acted yet (an ordinary crash of this
             town, e.g. a bus bypass while your bike rode 200 m away), so the crash says nothing about the hazard;
    None   = no ego collision.
    """
    ev = [e for e in R.events[k] if "ego" in (e["a"], e["b"])]
    if not ev:
        return None
    ff = fire_frames(R) if ff is None else ff
    f_hit, other = ev[0]["f"], (ev[0]["b"] if ev[0]["a"] == "ego" else ev[0]["a"])
    acted = [i for i in range(ff.shape[1]) if NEVER < ff[k, i] <= f_hit]
    for i in acted:
        h = R.hazards[i]
        if other == f"H{i}" or (h["type"] == "brake" and other == h["who"]):
            return i
    return -1


def oracle_avoids(params, R: core.Rollout, hazards, policy=None):
    """For each blamed crash: would the same planner have avoided every collision by braking at the tyre limit from
    the first frame it could know about the hazard?

    That frame is when the blamed hazard has started acting AND the ego's sensor has line of sight to it (a pedestrian
    behind a parked bus is not yet knowable). The oracle rollout runs the planner unchanged up to that frame
    (core.rollout(brake_from=...)), so the world is bit-identical until then.

    Returns (avoid, f0, avoid_foreknown): avoid = "tránh được bằng phanh hết cỡ ngay khi thấy"; avoid_foreknown =
    the same braking from the hazard's start even if unseen (an upper bound: the gap between the two is what
    occlusion costs). Unattributed crashes and crash-free runs get True with f0 = -1; report them separately.
    """
    ff = fire_frames(R, params)
    f_fire = np.full(R.K, NEVER)
    blamed = np.full(R.K, -1)
    for k in range(R.K):
        b = blame(R, k, ff)
        if b is not None and b >= 0:
            f_fire[k], blamed[k] = ff[k, b], b
    f_vis = f_fire.copy()
    for i, h in enumerate(R.hazards or []):
        m = (blamed == i) & (f_fire >= 0)
        if not m.any() or h["type"] == "stalled":         # a stalled car's frame is already its first sighting
            continue
        j = R.ids.index(h["who"] if h["type"] == "brake" else f"H{i}")
        fs = _first_seen(R, j, start=np.where(m, f_fire, core.NF))
        f_vis = np.where(m, np.where(fs < core.NF - 1, fs, core.NF), f_vis)    # never seen = brakes never
    def run(f0):
        O = core.rollout(params, policy=policy, hazards=hazards, brake_from=np.where(f0 >= core.NF, -1, f0))
        return np.array([f0[k] < 0 or (f0[k] < core.NF and not any("ego" in (e["a"], e["b"]) for e in O.events[k]))
                         for k in range(R.K)])
    return run(f_vis), np.where(f_vis >= core.NF, NEVER, f_vis), run(f_fire)
