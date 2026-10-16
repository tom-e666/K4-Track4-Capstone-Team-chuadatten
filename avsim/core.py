"""avsim.core: a batched numpy port of sim3d/sim-core.js.

One call simulates K scenarios at once. Every frame is a fixed number of numpy operations on
(K, N) arrays, so a batch of 1 000 scenarios costs roughly what one scenario costs.
The JS file drives the 3D viewer; tests/test_parity.py checks that both give the same rollouts.

Units: metres, seconds. World frame: x east, z south (three.js convention). Right-hand traffic.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

DT = 0.05
T_END = 24.0
NF = int(round(T_END / DT)) + 1
G = 9.81

# ---------------------------------------------------------------- map (same polylines as the JS)


def _seg(x0, z0, x1, z1, step=1.0):
    n = max(1, math.ceil(math.hypot(x1 - x0, z1 - z0) / step))
    return [(x0 + (x1 - x0) * i / n, z0 + (z1 - z0) * i / n) for i in range(n + 1)]


def _arc(cx, cz, r, a0, a1, step=0.5):
    n = max(2, math.ceil(abs(a1 - a0) * r / step))
    out = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        out.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    return out


class Lane:
    def __init__(self, *parts):
        pts = []
        for q in parts:
            for p in q:
                if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
                    pts.append(p)
        self.pts = np.array(pts, dtype=float)
        s = [0.0]
        for j in range(1, len(pts)):
            s.append(s[-1] + math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1]))
        self.s = np.array(s)
        self.len = s[-1]

    def at(self, s, d):
        """Pose at arc length s with lateral offset d (right positive). Arrays in, arrays out."""
        s = np.clip(s, 0.0, self.len - 1e-6)
        lo = np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.s) - 2)
        a, b = self.pts[lo], self.pts[lo + 1]
        L = self.s[lo + 1] - self.s[lo]
        L = np.where(L == 0, 1.0, L)
        u = (s - self.s[lo]) / L
        hx = (b[..., 0] - a[..., 0]) / L
        hz = (b[..., 1] - a[..., 1]) / L
        x = a[..., 0] + (b[..., 0] - a[..., 0]) * u - hz * d
        z = a[..., 1] + (b[..., 1] - a[..., 1]) * u + hx * d
        return x, z, hx, hz


ROAD = dict(xMin=-220, xMax=330, half=7, crossX=80, crossHalf=7, walkX=122)
WALK_X = 122.0
LANES = {
    "E1": Lane(_seg(-220, 1.75, 330, 1.75)),
    "E2": Lane(_seg(-220, 5.25, 330, 5.25)),
    "W1": Lane(_seg(330, -1.75, -220, -1.75)),
    "W2": Lane(_seg(330, -5.25, -220, -5.25)),
    "XR": Lane(_seg(81.75, 70, 81.75, 10), _arc(86.5, 10, 4.75, math.pi, 1.5 * math.pi), _seg(86.5, 5.25, 330, 5.25)),
    "NB": Lane(_seg(81.75, 120, 81.75, -120)),
    "SB": Lane(_seg(78.25, -120, 78.25, 120)),
    "WALK": Lane(_seg(WALK_X, 9.2, WALK_X, -9.2, 0.25)),
    "SWS": Lane(_seg(-80, 8.6, 300, 8.6)),
    "SWN": Lane(_seg(300, -8.6, -80, -8.6)),
}

CAR = (4.6, 1.9)
BUS = (12.0, 2.55)
PED = (0.6, 0.6)
PED_LOOK = 0.5      # a walking pedestrian stops when a vehicle body is within this many metres ahead of its centre
BIKE = (1.8, 0.7)

# ---------------------------------------------------------------- scenario parameters and the ego policy

DEFAULTS = dict(cutT0=5.0, cutDur=2.4, cutV=15.0, pedT0=9.6, pedV=1.45, busGo=17.5, cutDs=-4.0, tgT=0.9, mu=0.8,
                cutGap=None, cutBrake=0.0, pedGap=None, pedLook=0, civil=False)
POLICY = dict(v0=12.5, T=1.6, a=1.4, b=2.0, s0=2.5, cautionV=7.5, cautionRange=55.0, pedBuffer=2.5, cutinLat=2.15)
HUMAN = dict(v0=12.5, T=1.1, a=1.6, b=3.5, s0=2.0)


def _idm(**kw):
    p = dict(v0=13.0, T=1.4, a=1.6, b=2.2, s0=2.0)
    p.update(kw)
    return p


@dataclass
class Actor:
    id: str
    name: str
    kind: str
    dims: tuple
    lane: str
    s: object          # float or per-scenario key
    v: object
    ctl: str
    p: dict = field(default_factory=dict)
    d: float = 0.0
    color: str = "c1"
    dark: bool = False
    until: object = None
    t0: object = 0.0
    vw: object = 0.0


# per-scenario values are given as strings naming a parameter; "180+cutDs" style offsets via tuples
ACTORS = [
    Actor("ego", "xe ta", "ego", CAR, "E1", 180.0, 12.0, "ego", color="ego"),
    Actor("L1", "xe dẫn", "car", CAR, "E1", 214.0, 12.4, "idm", _idm(v0=12.6), color="c1"),
    Actor("C2", "xe cắt làn", "car", CAR, "E2", ("add", 180.0, "cutDs"), "cutV", "cutin", _idm(v0="cutV", T=0.9, b=3.0), color="adv"),
    Actor("TG", "xe bám đuôi", "car", CAR, "E1", 163.0, 12.2, "idm", _idm(v0=13.5, T="tgT", a=1.8, b=3.0, s0=1.5), color="c3", dark=True),
    Actor("E5", "xe", "car", CAR, "E2", 120.0, 13.0, "idm", _idm(v0=13.0), color="c2"),
    Actor("BUS", "xe buýt", "bus", BUS, "E2", 326.0, 0.0, "hold", _idm(v0=9.0, a=1.0), color="bus", until="busGo"),
    Actor("X1", "xe rẽ phải", "car", CAR, "XR", 58.0, 0.0, "hold", _idm(v0=9.0, a=1.4), color="c4", until=18.6),
    Actor("W1a", "xe", "car", CAR, "W1", 150.0, 12.0, "idm", _idm(v0=12.5), color="c2"),
    Actor("W1b", "xe", "car", CAR, "W1", 120.0, 12.0, "idm", _idm(v0=12.5), color="c1", dark=True),
    Actor("W2a", "xe", "car", CAR, "W2", 190.0, 11.0, "idm", _idm(v0=11.5), color="c4"),
    Actor("W2b", "xe", "car", CAR, "W2", 60.0, 13.0, "idm", _idm(v0=13.0), color="c3"),
    Actor("W1c", "xe", "car", CAR, "W1", 268.0, 12.0, "idm", _idm(v0=12.5), color="c2"),
    Actor("P1", "người đi bộ", "ped", PED, "WALK", 0.0, 0.0, "walk", color="ped", t0="pedT0", vw="pedV"),
    Actor("P2", "người đi bộ", "ped", PED, "SWS", 120.0, 1.3, "walk", color="ped", t0=0.0, vw=1.3),
    Actor("P3", "người đi bộ", "ped", PED, "SWN", 90.0, 1.2, "walk", color="ped", t0=0.0, vw=1.2),
    Actor("P4", "người đi bộ", "ped", PED, "SWS", 260.0, 1.1, "walk", d=0.4, color="ped", t0=0.0, vw=1.1),
]
IDS = [a.id for a in ACTORS]
N = len(ACTORS)
IX = {a.id: i for i, a in enumerate(ACTORS)}
EGO, C2, TG, BUSI, X1, P1 = IX["ego"], IX["C2"], IX["TG"], IX["BUS"], IX["X1"], IX["P1"]
PEDS = np.array([a.kind == "ped" for a in ACTORS])
LEN = np.array([a.dims[0] for a in ACTORS])
WID = np.array([a.dims[1] for a in ACTORS])
PARKED = [dict(x=89.6, z=22, hx=0, hz=1, dark=False, color="c2"), dict(x=89.6, z=30, hx=0, hz=1, dark=True, color="c1"),
          dict(x=70.4, z=-24, hx=0, hz=-1, dark=False, color="c4"), dict(x=70.4, z=16, hx=0, hz=-1, dark=False, color="c3"),
          dict(x=89.6, z=-18, hx=0, hz=1, dark=False, color="c3")]


def _val(v, P, K):
    """Resolve an actor field: a number, a per-scenario array, a parameter name, or ("add", base, name)."""
    if isinstance(v, np.ndarray):
        return v.astype(float)
    if isinstance(v, tuple):
        return v[1] + P[v[2]]
    if isinstance(v, str):
        return P[v].astype(float)
    if v is None:
        return np.full(K, np.inf)
    return np.full(K, float(v))


def as_batch(params, defaults=DEFAULTS):
    """list of dicts (or one dict) -> dict of arrays, with defaults filled in. None gates become nan."""
    if isinstance(params, dict):
        params = [params]
    K = len(params)
    out = {}
    for k, dv in defaults.items():
        col = []
        for p in params:
            v = p.get(k, dv)
            col.append(np.nan if v is None else float(v))
        out[k] = np.array(col, dtype=float)
    return out, K


# ---------------------------------------------------------------- hazard cards (same as HAZARD_DEFAULTS in sim-core.js)

HAZARD_DEFAULTS = {
    "ped": dict(x=200.0, side="S", v=1.6, t0=0.0, trig=30.0),
    "stalled": dict(x=260.0, lane="E2"),
    "brake": dict(who="L1", decel=6.0, dur=2.5, t0=0.0, trig=None),
    "bike": dict(x=200.0, lane="E2", d=1.2, v=5.0, swerve=1.6, swDur=2.0, t0=0.0, trig=25.0),
}
HAZARD_STRUCT = {"type", "side", "lane", "who"}     # fixed for a whole batch; every other field may vary per scenario
# a crossing at x = 0; the hazard's x is added afterwards, which reproduces the JS polyline exactly
_XWALK = {"S": Lane(_seg(0, 9.2, 0, -9.2, 0.25)), "N": Lane(_seg(0, -9.2, 0, 9.2, 0.25))}


def hazard_full(h):
    out = dict(HAZARD_DEFAULTS.get(h["type"], {}))
    out.update(h)
    return out


def _hazard_actors(hazards, plist, K):
    """Extra actors and per-scenario hazard values. Scenario k may override field f of hazard i with params key "h{i}_{f}"."""
    hz = [hazard_full(h) for h in (hazards or [])]

    def hv(i, f):
        col = [p.get(f"h{i}_{f}", hz[i].get(f)) for p in plist]
        return np.array([np.nan if c is None else float(c) for c in col], dtype=float)
    extra, xwalk, walk_trig, bikes, brakes = [], {}, {}, {}, []
    n0 = len(ACTORS)
    for i, h in enumerate(hz):
        aid = f"H{i}"
        if h["type"] == "ped":
            j = n0 + len(extra)
            extra.append(Actor(aid, "người băng ngang", "ped", PED, aid, 0.0, 0.0, "walk", color="ped", t0=hv(i, "t0"), vw=hv(i, "v")))
            X = hv(i, "x")
            xwalk[j] = (X, _XWALK["N" if h["side"] == "N" else "S"])
            walk_trig[j] = (hv(i, "trig"), X, i)
        elif h["type"] == "stalled":
            extra.append(Actor(aid, "xe chết máy", "car", CAR, h["lane"], hv(i, "x") + 220, 0.0, "hold", _idm(), color="c4", dark=True, until=None))
        elif h["type"] == "bike":
            j = n0 + len(extra)
            v = hv(i, "v")
            extra.append(Actor(aid, "xe đạp", "bike", BIKE, h["lane"], hv(i, "x") + 220, v, "bike", _idm(v0=v, T=1.0, a=1.0, b=2.5, s0=1.5),
                               d=hv(i, "d"), color="c1"))
            bikes[j] = dict(t0=hv(i, "t0"), trig=hv(i, "trig"), dd=-hv(i, "swerve"), dur=hv(i, "swDur"), d0=hv(i, "d"), start=np.full(K, np.nan), hz=i)
        elif h["type"] == "brake":
            if h["who"] in IX and ACTORS[IX[h["who"]]].ctl not in ("hold", "walk", "ego"):
                brakes.append(dict(i=IX[h["who"]], t0=hv(i, "t0"), trig=hv(i, "trig"), decel=hv(i, "decel"), dur=hv(i, "dur"), start=np.full(K, np.nan), hz=i))
            else:
                raise ValueError(f"brake hazard: {h['who']!r} is not a moving car (use L1, C2, TG, E5, W1a, ...)")
        else:
            raise ValueError(f"unknown hazard type {h['type']!r}")
    return hz, extra, xwalk, walk_trig, bikes, brakes


# ---------------------------------------------------------------- geometry (vectorised)


def seg_hits_box(x0, z0, x1, z1, bx, bz, bhx, bhz, bl, bw):
    """Does segment p0->p1 cross the oriented box? Broadcasts over any shape."""
    rx, rz, qx, qz = x0 - bx, z0 - bz, x1 - bx, z1 - bz
    p0 = rx * bhx + rz * bhz
    p1 = -rx * bhz + rz * bhx
    q0 = qx * bhx + qz * bhz
    q1 = -qx * bhz + qz * bhx
    t0 = np.zeros(np.broadcast(p0, bl).shape)
    t1 = np.ones_like(t0)
    ok = np.ones_like(t0, dtype=bool)
    for p, q, e in ((p0, q0, bl), (p1, q1, bw)):
        d = q - p
        flat = np.abs(d) < 1e-9
        ok &= ~(flat & (np.abs(p) > e))
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.where(flat, -np.inf, (-e - p) / np.where(flat, 1, d))
            v = np.where(flat, np.inf, (e - p) / np.where(flat, 1, d))
        lo, hi = np.minimum(u, v), np.maximum(u, v)
        t0 = np.maximum(t0, lo)
        t1 = np.minimum(t1, hi)
    return ok & (t0 <= t1)


def obb_overlap(ax, az, ahx, ahz, al, aw, bx, bz, bhx, bhz, bl, bw):
    """2-D separating-axis test for oriented boxes (broadcasting)."""
    dx, dz = bx - ax, bz - az
    sep = np.zeros(np.broadcast(ax, bx).shape, dtype=bool)
    for nx, nz in ((ahx, ahz), (-ahz, ahx), (bhx, bhz), (-bhz, bhx)):
        ra = al * np.abs(nx * ahx + nz * ahz) + aw * np.abs(-nx * ahz + nz * ahx)
        rb = bl * np.abs(nx * bhx + nz * bhz) + bw * np.abs(-nx * bhz + nz * bhx)
        sep |= np.abs(nx * dx + nz * dz) > ra + rb
    return ~sep


# ---------------------------------------------------------------- rollout


@dataclass
class Rollout:
    F: np.ndarray            # (K, NF, N, 7): x, z, hx, hz, v, a, d
    EGO: np.ndarray          # (K, NF, 4): mode, v0, ped-visible, leader index
    events: list             # per scenario: list of dicts {t, f, a, b, x, z, fake}
    params: dict             # dict of arrays
    policy: dict             # dict of arrays
    ego: str
    others: str
    actors: list = None      # ACTORS plus any hazard actors (None = ACTORS)
    hazards: list = None     # the hazard cards, defaults filled in
    fire: np.ndarray = None  # (K, n_hazards) frame each hazard started acting (walk / swerve / brake), -1 = never.
                             # A stalled car never "acts": it is just there (see studio.fire_frames).

    @property
    def ids(self):
        return [a.id for a in (self.actors or ACTORS)]

    @property
    def K(self):
        return self.F.shape[0]

    def scenario(self, k):
        """Scenario k as the dict the 3D viewer reads (same layout as SimCore.rollout)."""
        return dict(F=self.F[k].reshape(-1).tolist(), EGO=self.EGO[k].reshape(-1).tolist(), events=self.events[k],
                    actors=[dict(id=a.id, name=a.name, kind=a.kind, dims=dict(len=a.dims[0], wid=a.dims[1]), color=a.color,
                                 dark=a.dark, lane=a.lane) for a in (self.actors or ACTORS)],
                    hazards=self.hazards or [],
                    params={key: (None if np.isnan(v[k]) else float(v[k])) for key, v in self.params.items()},
                    policy={key: float(v[k]) for key, v in self.policy.items()})


def _idm_acc(v0, T, a, b, s0, v, gap, lv, has_lead, amin):
    ss = s0 + np.maximum(0.0, v * T + v * (v - lv) / (2 * np.sqrt(a * b)))
    free = 1 - (v / v0) ** 4
    inter = np.where(has_lead, (ss / np.maximum(gap, 0.2)) ** 2, 0.0)
    return np.maximum(amin, np.minimum(a, a * (free - inter)))


def rollout(params=None, policy=None, ego="planner", others="reactive", log: Rollout | None = None,
            hazards=None, ego_acc=None, brake_from=None) -> Rollout:
    """Simulate a batch. params / policy: a dict or a list of dicts (same length), missing keys take the defaults.

    ego: "planner" (the policy), "human" (the logged driver) or "script" (ego_acc: pedal input per frame, (NF,) or (K, NF)).
    others: "reactive" or "replay" (needs log, a Rollout of the same batch: everyone but the ego follows it).
    hazards: hazard cards (see HAZARD_DEFAULTS), the same structure for the whole batch; params key "h{i}_{field}"
    overrides a numeric field of hazard i for one scenario.
    brake_from: (K,) frame per scenario from which the ego ignores its planner and brakes at the tyre limit (-1 or
    >= NF = never). Used by the full-brake oracle (studio.oracle_avoids): before that frame the run is bit-identical.
    """
    P, K = as_batch(params if params is not None else {})
    POL, K2 = as_batch(policy if policy is not None else ([{}] * K), POLICY)
    if K2 == 1 and K > 1:
        POL = {k: np.repeat(v, K) for k, v in POL.items()}
    elif K == 1 and K2 > 1:
        P = {k: np.repeat(v, K2) for k, v in P.items()}
        K = K2
    replay = others == "replay"
    if replay and (log is None or log.K != K):
        raise ValueError("replay needs log=Rollout of the same batch size")
    civil = P["civil"] > 0.5
    amin = -np.minimum(9.0, P["mu"] * G)
    human = ego == "human"
    script = ego == "script"
    plist = params if isinstance(params, list) else [params or {}]
    if len(plist) != K:
        plist = [plist[0]] * K
    hz_cards, extra, xwalk, walk_trig, bikes, brakes = _hazard_actors(hazards, plist, K)
    ACT = ACTORS + extra
    N = len(ACT)
    IDS = [a.id for a in ACT]
    PEDS = np.array([a.kind == "ped" for a in ACT])
    LEN = np.array([a.dims[0] for a in ACT])
    WID = np.array([a.dims[1] for a in ACT])
    bf = np.full(K, NF) if brake_from is None else np.where(np.asarray(brake_from) < 0, NF, np.asarray(brake_from))
    fire = np.full((K, len(hz_cards)), -1, dtype=int)
    if script:
        ea = np.asarray(ego_acc, dtype=float)
        ea = np.broadcast_to(ea if ea.ndim == 2 else ea[None, :], (K, ea.shape[-1]))

    s = np.stack([_val(a.s, P, K) for a in ACT], 1)
    v = np.stack([_val(a.v, P, K) for a in ACT], 1)
    d = np.stack([_val(a.d, P, K) for a in ACT], 1)
    idmp = {key: np.stack([_val(a.p.get(key, 0.0), P, K) for a in ACT], 1) for key in ("v0", "T", "a", "b", "s0")}
    until = np.stack([_val(a.until, P, K) if a.ctl == "hold" else np.full(K, -np.inf) for a in ACT], 1)
    t0w = np.stack([_val(a.t0, P, K) for a in ACT], 1)
    vw = np.stack([_val(a.vw, P, K) for a in ACT], 1)
    started = np.full((K, N), np.nan)              # walkers: time they started
    going = np.zeros((K, N), dtype=bool)           # hold actors that have given way and left
    c_start = np.full(K, np.nan)
    c_u = np.zeros(K)
    c_done = np.full(K, np.nan)
    gated_ped = ~np.isnan(P["pedGap"])
    gated_cut = ~np.isnan(P["cutGap"])

    F = np.zeros((K, NF, N, 7), dtype=np.float32)
    EGOA = np.zeros((K, NF, 4), dtype=np.float32)
    iu, ju = np.triu_indices(N, 1)
    keep = ~(PEDS[iu] & PEDS[ju])                    # pedestrians do not collide with each other
    iu, ju = iu[keep], ju[keep]
    hit_f = np.full((K, len(iu)), -1, dtype=int)     # frame of first contact per pair
    pb = int(np.nonzero((iu == EGO) & (ju == P1))[0][0])   # the (ego, P1) pair: its event may be a crosswalk block
    is_blk = np.zeros(K, dtype=bool)                 # (ego, P1) event is a block, not a contact
    reach = np.hypot(LEN / 2, WID / 2)
    reach2 = (reach[iu] + reach[ju]) ** 2
    lane_of = [a.lane for a in ACT]
    halfL, halfW = LEN / 2, WID / 2
    veh = ~PEDS
    ar = np.arange(K)
    lat_thr = (WID[:, None] + WID[None, :]) / 2 + 0.25            # (i, j) default corridor half-width
    rep_mask = np.array([a.id != "ego" for a in ACT]) if replay else np.zeros(N, dtype=bool)

    def fired(trig, t0, t, xh):
        """A hazard fires once t >= t0 and the ego is within trig metres behind xh (trig nan = at t0)."""
        return (t >= t0) & (np.isnan(trig) | (xh - x[:, EGO] <= trig))

    for f in range(NF):
        t = f * DT
        x = np.empty((K, N)); z = np.empty((K, N)); hx = np.empty((K, N)); hz = np.empty((K, N))
        for i in range(N):
            if i in xwalk:
                X, ln = xwalk[i]
                x[:, i], z[:, i], hx[:, i], hz[:, i] = ln.at(s[:, i], d[:, i])
                x[:, i] = X + x[:, i]
            else:
                x[:, i], z[:, i], hx[:, i], hz[:, i] = LANES[lane_of[i]].at(s[:, i], d[:, i])
        if replay:
            L = log.F[:, f]
            x[:, rep_mask], z[:, rep_mask] = L[:, rep_mask, 0], L[:, rep_mask, 1]
            hx[:, rep_mask], hz[:, rep_mask] = L[:, rep_mask, 2], L[:, rep_mask, 3]
            v[:, rep_mask] = L[:, rep_mask, 4]
        acc = np.zeros((K, N))
        blk_ego = np.zeros(K, dtype=bool)

        # pairwise frame: j relative to i
        rx = x[:, None, :] - x[:, :, None]
        rz = z[:, None, :] - z[:, :, None]
        lon = rx * hx[:, :, None] + rz * hz[:, :, None]
        lat = -rx * hz[:, :, None] + rz * hx[:, :, None]
        align = hx[:, None, :] * hx[:, :, None] + hz[:, None, :] * hz[:, :, None]

        def seg_blocked(x0, z0, x1, z1, blockers):
            """(K,) True if the segment crosses any of the boxes in `blockers` ((K, n) bool mask over actors)."""
            hit = seg_hits_box(x0[:, None], z0[:, None], x1[:, None], z1[:, None], x, z, hx, hz, halfL[None, :], halfW[None, :])
            return np.any(hit & blockers, axis=1)

        def bus_clear(x0, z0, x1, z1, self_idx=None):
            """Only the bus is tall enough to hide road users from people."""
            if self_idx == BUSI:
                return np.ones(K, dtype=bool)
            b = BUSI
            return ~seg_hits_box(x0, z0, x1, z1, x[:, b], z[:, b], hx[:, b], hz[:, b], halfL[b], halfW[b])

        vis_cache = {}

        def sensor_visible(i, j):
            """The ego's roof sensor: any vehicle box blocks the line of sight."""
            if (i, j) not in vis_cache:
                m = np.broadcast_to(veh, (K, N)).copy(); m[:, i] = False; m[:, j] = False
                vis_cache[(i, j)] = ~seg_blocked(x[:, i], z[:, i], x[:, j], z[:, j], m)
            return vis_cache[(i, j)]

        # leaders for every vehicle i (gap, leader speed, index)
        gap = np.full((K, N), 1e9); lv = np.zeros((K, N)); lidx = np.full((K, N), -1)
        egolat = np.where(human, lat_thr[EGO], POL["cutinLat"][:, None] * np.ones(N))   # (K, N) ego's corridor
        for i in range(N):
            if PEDS[i]:
                continue
            ahead = (lon[:, i] > 0) & (lon[:, i] <= 120)
            ahead[:, i] = False
            cand_g = np.full((K, N), 1e9); cand_v = np.zeros((K, N))
            # vehicles in my corridor, same direction
            thr = egolat if i == EGO else np.broadcast_to(lat_thr[i], (K, N))
            vm = ahead & veh[None, :] & (align[:, i] >= 0.3) & (np.abs(lat[:, i]) <= thr)
            cand_g = np.where(vm, lon[:, i] - halfL[i] - halfL[None, :], cand_g)
            cand_v = np.where(vm, v, cand_v)
            # pedestrians
            for j in np.nonzero(PEDS)[0]:
                if lane_of[j] == "WALK":
                    civ = civil & (v[:, j] > 0) & (np.abs(hx[:, i]) > 0.7) & (z[:, j] > z[:, i] - 1.9) & (z[:, j] < 9.3)
                    lonCW = (WALK_X - x[:, i]) * hx[:, i]
                    gc = lonCW - halfL[i] - 1.5
                    if civ.any():
                        seen = sensor_visible(i, j) if i == EGO else bus_clear(x[:, i], z[:, i], x[:, j], z[:, j], i)
                    else:
                        seen = np.zeros(K, dtype=bool)
                    use_civ = ahead[:, j] & civ & (lonCW > 0) & seen
                else:
                    civ = np.zeros(K, dtype=bool); use_civ = civ; gc = 0.0
                plain = ahead[:, j] & ~civ & (np.abs(lat[:, i, j]) <= 1.9)
                cand_g[:, j] = np.where(use_civ, gc, np.where(plain, lon[:, i, j] - halfL[i] - 1.2, 1e9))
                cand_v[:, j] = 0.0
            # JS keeps the first strictly smaller gap in index order: argmin does the same
            jbest = np.argmin(cand_g, axis=1)
            gbest = cand_g[ar, jbest]
            has = gbest < 1e9
            gap[:, i] = np.where(has, gbest, 1e9)
            lv[:, i] = np.where(has, cand_v[ar, jbest], 0.0)
            lidx[:, i] = np.where(has, jbest, -1)

        # ---- the ego
        i = EGO
        g_e, lv_e, li_e = gap[:, i].copy(), lv[:, i].copy(), lidx[:, i].copy()
        mode = np.where((li_e >= 0) & (g_e < 45), 1.0, 0.0)
        vis = sensor_visible(EGO, P1)
        dxw = WALK_X - x[:, EGO]
        if script:
            ae = np.maximum(amin, np.minimum(3.0, ea[:, min(f, ea.shape[1] - 1)]))
            acc[:, EGO] = np.where((v[:, EGO] <= 0) & (ae < 0), 0.0, ae)
            EGOA[:, f, 0], EGOA[:, f, 1], EGOA[:, f, 2], EGOA[:, f, 3] = 4.0, 0.0, vis, li_e
        elif human:
            pv = vis.copy()
            drop = (li_e == P1) & ~pv
            g_e = np.where(drop, 1e9, g_e); lv_e = np.where(drop, 0.0, lv_e); li_e = np.where(drop, -1, li_e)
            e_v0, e_T, e_a, e_b, e_s0 = (np.full(K, HUMAN[k]) for k in ("v0", "T", "a", "b", "s0"))
        else:
            e_v0, e_T, e_a, e_b, e_s0 = (POL[k].copy() for k in ("v0", "T", "a", "b", "s0"))
            vp = v[:, P1]
            will = (vis | ~civil) & (vp > 0.1) & (z[:, P1] > -2) & (z[:, P1] < 9.5) & (dxw > 0)   # civil: only a pedestrian it sees
            t_enter = np.maximum(0, (z[:, P1] - (z[:, EGO] + 1.9)) / np.maximum(vp, 0.3))
            t_arrive = (dxw - 3) / np.maximum(v[:, EGO], 1)
            yieldp = will & (t_enter < t_arrive + POL["pedBuffer"])
            gp = dxw - halfL[EGO] - 2.5
            take = yieldp & (gp < g_e)
            g_e = np.where(take, gp, g_e); lv_e = np.where(take, 0.0, lv_e); li_e = np.where(take, P1, li_e)
            mode = np.where(yieldp, 3.0, mode)
            caution = (dxw > 0) & (dxw < POL["cautionRange"]) & ~vis & (mode != 3)
            e_v0 = np.where(caution, np.minimum(e_v0, POL["cautionV"]), e_v0)
            mode = np.where(caution, 2.0, mode)
        if not script:
            acc[:, EGO] = _idm_acc(e_v0, e_T, e_a, e_b, e_s0, v[:, EGO], g_e, lv_e, li_e >= 0, amin)
            EGOA[:, f, 0], EGOA[:, f, 1], EGOA[:, f, 2], EGOA[:, f, 3] = mode, e_v0, vis, li_e
        if brake_from is not None:
            stop = f >= bf
            acc[:, EGO] = np.where(stop, np.where(v[:, EGO] > 0, amin, 0.0), acc[:, EGO])
            EGOA[:, f, 0] = np.where(stop, 5.0, EGOA[:, f, 0])

        # ---- everyone else, in actor order (order matters only through flags set this frame)
        for i in range(1, N):
            a = ACT[i]
            if replay and rep_mask[i]:
                acc[:, i] = log.F[:, f, i, 5]
                continue
            if a.ctl == "walk":
                if i == P1:
                    ok = np.ones(K, dtype=bool)
                    eye_z = np.where((P["pedLook"] > 0.5) & (z[:, i] > 3.7), 3.7, z[:, i])
                    for j in range(N):
                        if PEDS[j]:
                            continue
                        dx = WALK_X - x[:, j]
                        cand = (np.abs(hx[:, j]) >= 0.7) & (np.abs(z[:, j]) <= 7) & (dx * hx[:, j] > 0) & (np.abs(dx) <= 90)
                        tta = (np.abs(dx) - halfL[j]) / np.maximum(v[:, j], 0.5)
                        sees = bus_clear(x[:, i], eye_z, x[:, j], z[:, j], j)
                        ok &= ~(cand & (tta < P["pedGap"]) & sees)
                    ok = np.where(gated_ped, ok, True)
                elif i in walk_trig:
                    ok = fired(walk_trig[i][0], t0w[:, i], t, walk_trig[i][1])
                else:
                    ok = np.ones(K, dtype=bool)
                newly = np.isnan(started[:, i]) & (t >= t0w[:, i]) & ok
                started[:, i] = np.where(newly, t, started[:, i])
                if i in walk_trig:
                    fire[:, walk_trig[i][2]] = np.where(newly, f, fire[:, walk_trig[i][2]])
                v[:, i] = np.where(~np.isnan(started[:, i]), vw[:, i], 0.0)
                # nobody walks into the side of a car: stop while a vehicle body fills the step ahead
                px, pz = x[:, i] + hx[:, i] * PED_LOOK, z[:, i] + hz[:, i] * PED_LOOK
                blocked = np.any(obb_overlap(px[:, None], pz[:, None], hx[:, i:i + 1], hz[:, i:i + 1], halfL[i], halfW[i],
                                             x, z, hx, hz, halfL[None, :], halfW[None, :]) & veh[None, :], axis=1)
                walking = v[:, i] > 0
                v[:, i] = np.where(blocked, 0.0, v[:, i])
                if i == P1:      # the ego's body stopped the pedestrian on the crosswalk: a failure in its own right
                    blk_ego = obb_overlap(px, pz, hx[:, i], hz[:, i], halfL[i], halfW[i],
                                          x[:, EGO], z[:, EGO], hx[:, EGO], hz[:, EGO], halfL[EGO], halfW[EGO]) & walking
                continue
            held = np.zeros(K, dtype=bool)
            if a.ctl == "hold":
                held = t < until[:, i]
                if a.lane == "XR":
                    busy = np.zeros(K, dtype=bool)
                    for q in range(N):
                        if PEDS[q] or q == i:
                            continue
                        dq = 86 - x[:, q]
                        busy |= (hx[:, q] >= 0.7) & (z[:, q] >= 3.4) & (z[:, q] <= 7.2) & (dq > -3) & (dq < 70) & \
                                ((dq - halfL[q]) / np.maximum(v[:, q], 0.5) < 3.5)
                    wait = ~held & civil & ~going[:, i] & busy
                    going[:, i] |= ~held & civil & ~busy
                    held |= wait
                if a.kind == "bus":
                    lonCW = (WALK_X - x[:, i]) * hx[:, i]
                    zp = LANES["WALK"].at(s[:, P1], 0.0)[1]
                    busyc = (lonCW >= -halfL[i]) & (lonCW <= 30) & (v[:, P1] > 0) & (zp > z[:, i] - 2.2) & (zp < 9.3)
                    wait = ~held & civil & ~going[:, i] & busyc
                    going[:, i] |= ~held & civil & ~busyc
                    held |= wait
                v[:, i] = np.where(held, 0.0, v[:, i])
            v0 = idmp["v0"][:, i]
            if True:
                lonCW = (WALK_X - x[:, i]) * hx[:, i]
                kerb_z = np.where(hx[:, i] > 0, 8.5, -8.5)
                blind = civil & (np.abs(hx[:, i]) >= 0.7) & (lonCW >= 0) & (lonCW <= 40) & \
                    ~bus_clear(x[:, i], z[:, i], np.full(K, WALK_X), kerb_z, i)
                v0 = np.where(blind, np.minimum(v0, 6.0), v0)
            ai = _idm_acc(v0, idmp["T"][:, i], idmp["a"][:, i], idmp["b"][:, i], idmp["s0"][:, i], v[:, i], gap[:, i], lv[:, i], lidx[:, i] >= 0, amin)
            if a.ctl == "cutin":
                d1 = -3.5

                def gate(rear_need, front_need):
                    okk = np.ones(K, dtype=bool)
                    for j in range(N):
                        if j == i or PEDS[j]:
                            continue
                        m = (align[:, i, j] >= 0.7) & (np.abs(lat[:, i, j] - d1) <= 1.6)
                        half = (LEN[i] + LEN[j]) / 2
                        lo = lon[:, i, j]
                        bad = m & (((lo <= 0) & (-lo - half < rear_need(j))) | ((lo > 0) & (lo - half < front_need)))
                        okk &= ~bad
                    return okk
                cut_ok = gate(lambda j: P["cutGap"] + np.maximum(0, v[:, j] - v[:, i]), 2.0)
                cut_ok = np.where(gated_cut, cut_ok, True)
                new = np.isnan(c_start) & (t >= P["cutT0"]) & cut_ok
                c_u = np.where(new, 0.0, c_u)
                c_start = np.where(new, t, c_start)
                st_ = ~np.isnan(c_start)
                u = np.where(~st_, 0.0, np.where(civil, c_u, (t - np.where(st_, c_start, 0)) / P["cutDur"]))
                clear = gate(lambda j: 0.5, 1.0)
                c_u = np.where(st_ & civil & (c_u < 1) & clear, np.minimum(1.0, c_u + DT / P["cutDur"]), c_u)
                d[:, i] = np.where(u <= 0, 0.0, np.where(u >= 1, d1, d1 * (1 - np.cos(np.pi * u)) / 2))
                c_done = np.where((u >= 1) & np.isnan(c_done), t, c_done)
                brk = (P["cutBrake"] > 0) & ~np.isnan(c_done) & (t < c_done + 2.5)
                ai = np.where(brk, np.maximum(amin, np.minimum(ai, -P["cutBrake"])), ai)
            if i in bikes:
                w = bikes[i]
                nw = np.isnan(w["start"]) & fired(w["trig"], w["t0"], t, x[:, i])
                w["start"] = np.where(nw, t, w["start"])
                fire[:, w["hz"]] = np.where(nw, f, fire[:, w["hz"]])
                uw = np.where(np.isnan(w["start"]), 0.0, np.minimum(1.0, (t - np.nan_to_num(w["start"])) / w["dur"]))
                d[:, i] = w["d0"] + w["dd"] * (1 - np.cos(np.pi * uw)) / 2
            for bk in brakes:
                if bk["i"] != i:
                    continue
                nb = np.isnan(bk["start"]) & ~held & fired(bk["trig"], bk["t0"], t, x[:, i])
                bk["start"] = np.where(nb, t, bk["start"])
                fire[:, bk["hz"]] = np.where(nb, f, fire[:, bk["hz"]])
                on = ~np.isnan(bk["start"]) & (t < np.nan_to_num(bk["start"]) + bk["dur"])
                ai = np.where(on, np.maximum(amin, np.minimum(ai, -bk["decel"])), ai)
            acc[:, i] = np.where(held, 0.0, ai)

        F[:, f, :, 0], F[:, f, :, 1], F[:, f, :, 2], F[:, f, :, 3] = x, z, hx, hz
        F[:, f, :, 4], F[:, f, :, 5], F[:, f, :, 6] = v, acc, d

        # collisions: first contact per pair. Cheap circle test first, the box test only where circles touch.
        near = ((x[:, iu] - x[:, ju]) ** 2 + (z[:, iu] - z[:, ju]) ** 2 < reach2) & (hit_f < 0)
        if near.any():
            kk, pp = np.nonzero(near)
            a_, b_ = iu[pp], ju[pp]
            ov = obb_overlap(x[kk, a_], z[kk, a_], hx[kk, a_], hz[kk, a_], halfL[a_], halfW[a_],
                             x[kk, b_], z[kk, b_], hx[kk, b_], hz[kk, b_], halfL[b_], halfW[b_])
            hit_f[kk[ov], pp[ov]] = f
        nb = blk_ego & (hit_f[:, pb] < 0)            # a contact in the same frame wins over a block
        hit_f[nb, pb] = f
        is_blk |= nb

        if f < NF - 1:
            mv = ~rep_mask
            v[:, mv] = np.maximum(0, v[:, mv] + acc[:, mv] * DT)
            s[:, mv] += v[:, mv] * DT

    # events in the JS order: by frame, then pair (u1 < u2)
    events = []
    for k in range(K):
        ev = []
        fr = hit_f[k]
        for p in np.argsort(np.where(fr < 0, 10 ** 9, fr), kind="stable"):
            if fr[p] < 0:
                break
            a, b, ff = iu[p], ju[p], fr[p]
            ev.append(dict(t=float(round(ff * DT, 10)), f=int(ff), a=IDS[a], b=IDS[b], kind="block" if p == pb and is_blk[k] else "contact",
                           x=float((F[k, ff, a, 0] + F[k, ff, b, 0]) / 2), z=float((F[k, ff, a, 1] + F[k, ff, b, 1]) / 2),
                           fake=bool(replay and ((rep_mask[a]) or (rep_mask[b])))))
        events.append(ev)
    return Rollout(F=F, EGO=EGOA, events=events, params=P, policy=POL, ego=ego, others=others, actors=ACT, hazards=hz_cards, fire=fire)


def run_many(params, summarize, policy=None, ego="planner", others="reactive", chunk=512, progress=None, hazards=None):
    """Run a long list of scenarios in chunks and keep only what `summarize(rollout)` returns (a list, one item per scenario).

    Full trajectories cost about 200 kB per scenario, so a 20 000-scenario search must not keep them.
    For others="replay", each chunk first runs the logged human driver on the same scenarios.
    """
    out = []
    pol_list = policy if isinstance(policy, list) else None
    for c0 in range(0, len(params), chunk):
        ps = params[c0:c0 + chunk]
        pol = pol_list[c0:c0 + chunk] if pol_list is not None else ([policy or {}] * len(ps))
        log = rollout(ps, ego="human", hazards=hazards) if others == "replay" else None
        R = rollout(ps, policy=pol, ego=ego, others=others, log=log, hazards=hazards)
        out.extend(summarize(R))
        if progress:
            progress(min(len(params), c0 + chunk), len(params))
    return out
