"""avsim.space: the scenario space and its natural distribution.

A scenario is a latent vector u: 11 continuous coordinates that are standard normal under the natural
distribution, plus 3 on/off coordinates that are Bernoulli. `to_params(U)` maps latent rows to the
simulator's parameters. Search methods move a proposal q(u) around this space; importance weights
p(u)/q(u) are then simple to compute because p is a plain Gaussian times Bernoullis.

The priors below are teaching assumptions chosen to give a plausible-looking town, not fitted to data.
Report question 7 asks what real data would replace each one.
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, field

import math

import numpy as np

_erf = np.vectorize(math.erf, otypes=[float])


def ndtr(z):
    """Standard normal CDF (numpy has no erf; this keeps the lab free of scipy)."""
    return 0.5 * (1.0 + _erf(np.asarray(z, dtype=float) / math.sqrt(2.0)))

# name, kind, args: "unif" lo hi | "norm" mean sd lo | "logn" median sigma lo hi
CONT = [
    ("cutT0", "unif", (1.0, 12.0)),      # when the neighbour starts looking for a gap [s]
    ("cutDur", "logn", (4.0, 0.35, 1.0, 9.0)),   # lane-change duration [s]
    ("cutV", "norm", (14.0, 1.5, 6.0)),  # neighbour's speed [m/s]
    ("cutDs", "unif", (-12.0, 10.0)),    # neighbour's start position relative to the ego [m]
    ("pedT0", "unif", (5.0, 17.0)),      # pedestrian arrives at the kerb [s]
    ("pedV", "norm", (1.35, 0.20, 0.6)), # walking speed [m/s]
    ("busGo", "unif", (12.0, 22.0)),     # bus leaves the stop [s]
    ("tgT", "unif", (0.6, 1.6)),         # tailgater's time headway [s]
    ("cutGap", "logn", (12.0, 0.5, 1.0, 60.0)),  # gap the merging driver accepts [m]
    ("pedGap", "logn", (4.5, 0.35, 0.5, 15.0)),  # time gap the pedestrian accepts [s]
    ("cutBrakeV", "unif", (1.5, 4.0)),   # how hard the merging driver brakes, if it does [m/s^2]
]
BIN = [
    ("wet", 0.20),       # wet road: friction 0.5 instead of 0.8
    ("pedLook", 0.85),   # the pedestrian peeks past the bus before crossing
    ("brakes", 0.15),    # the merging driver brakes right after merging
]
D, B = len(CONT), len(BIN)
NAMES = [c[0] for c in CONT] + [b[0] for b in BIN]


@dataclass
class Natural:
    """The natural distribution for one student (MSSV-derived variant)."""
    p_bin: np.ndarray = field(default_factory=lambda: np.array([b[1] for b in BIN]))
    cut_gap_median: float = 12.0

    def sample(self, n, rng):
        """n latent rows: (n, D) continuous, (n, B) binary."""
        return rng.standard_normal((n, D)), (rng.random((n, B)) < self.p_bin).astype(int)

    def logpdf(self, Z, Bn):
        """log p(u) up to the same constant for every row (the Gaussian normaliser cancels in p/q if q is Gaussian too)."""
        lz = -0.5 * np.sum(Z ** 2, axis=1) - 0.5 * D * np.log(2 * np.pi)
        lb = np.sum(Bn * np.log(self.p_bin) + (1 - Bn) * np.log1p(-self.p_bin), axis=1)
        return lz + lb


def to_params(Z, Bn, nat: Natural | None = None, extra_u=None):
    """Latent rows -> list of simulator parameter dicts (civil behaviour on)."""
    nat = nat or Natural()
    Z = np.atleast_2d(Z)
    Bn = np.atleast_2d(Bn)
    cols = {}
    for j, (name, kind, a) in enumerate(CONT):
        z = Z[:, j]
        if kind == "unif":
            val = a[0] + (a[1] - a[0]) * ndtr(z)
        elif kind == "norm":
            val = np.maximum(a[2], a[0] + a[1] * z)
        else:
            med = nat.cut_gap_median if name == "cutGap" else a[0]
            val = np.clip(med * np.exp(a[1] * z), a[2], a[3])
        cols[name] = val
    out = []
    for k in range(Z.shape[0]):
        p = {name: float(cols[name][k]) for name in cols if name != "cutBrakeV"}
        p["mu"] = 0.5 if Bn[k, 0] else 0.8
        p["pedLook"] = int(Bn[k, 1])
        p["cutBrake"] = float(cols["cutBrakeV"][k]) if Bn[k, 2] else 0.0
        p["civil"] = True
        out.append(p)
    return out


# Four towns. Each student gets one, so classmates' numbers differ but the grader's reference table stays small.
VARIANTS = [
    dict(name="A", p_bin=(0.20, 0.85, 0.15), cut_gap_median=12.0),   # the default town
    dict(name="B", p_bin=(0.30, 0.85, 0.15), cut_gap_median=12.0),   # rainier
    dict(name="C", p_bin=(0.20, 0.80, 0.15), cut_gap_median=10.0),   # bolder merges, fewer people peek past the bus
    dict(name="D", p_bin=(0.15, 0.90, 0.20), cut_gap_median=14.0),   # drier, more careful, more merge-then-brake
]


def variant(name: str) -> Natural:
    v = next(x for x in VARIANTS if x["name"] == name)
    return Natural(p_bin=np.array(v["p_bin"]), cut_gap_median=v["cut_gap_median"])


def student_variant(mssv: str) -> tuple[Natural, dict]:
    """Per-student town and seeds, derived from the student ID."""
    h = hashlib.sha256(str(mssv).strip().encode()).digest()
    v = VARIANTS[h[0] % len(VARIANTS)]
    nat = variant(v["name"])
    nat.name = v["name"]
    seeds = dict(mc=int.from_bytes(h[4:8], "little"), search=int.from_bytes(h[8:12], "little"),
                 test=int.from_bytes(h[12:16], "little"))
    return nat, seeds
