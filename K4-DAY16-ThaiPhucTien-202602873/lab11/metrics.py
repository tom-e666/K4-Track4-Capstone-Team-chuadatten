"""Part A: read a rollout. TODO 1, 2 and 9 live here; the helpers below them are given."""
from __future__ import annotations

import numpy as np

from avsim import core

G = 9.81
HALF_EGO = core.CAR[0] / 2   # 2.3 m: half the ego's length


def ego_events(R: core.Rollout, k: int) -> list[dict]:
    """Failure events involving the ego in scenario k, earliest first (given).

    event["kind"] is "contact" (the boxes touched) or "block" (the ego's body stopped the pedestrian P1 on the
    crosswalk: pedestrians never walk into a car, so this is how a planner that stops too late fails them)."""
    return [e for e in R.events[k] if e["a"] == "ego" or e["b"] == "ego"]


def min_ttc(R: core.Rollout, cap: float = 10.0) -> np.ndarray:
    K, NF = R.F.shape[0], R.F.shape[1]
    ego_idx = core.IX["ego"]

    leader_idx = R.EGO[..., 3].astype(int)
    valid = leader_idx >= 0
    safe_leader = np.where(valid, leader_idx, 0)

    x_ego = R.F[:, :, ego_idx, 0]
    z_ego = R.F[:, :, ego_idx, 1]
    hx_ego = R.F[:, :, ego_idx, 2]
    hz_ego = R.F[:, :, ego_idx, 3]
    v_ego = R.F[:, :, ego_idx, 4]

    k_idx = np.arange(K)[:, None]
    f_idx = np.arange(NF)[None, :]
    F_leader = R.F[k_idx, f_idx, safe_leader]
    x_leader = F_leader[..., 0]
    z_leader = F_leader[..., 1]
    v_leader = F_leader[..., 4]

    leader_half_len = core.LEN[safe_leader] / 2.0
    rx = x_leader - x_ego
    rz = z_leader - z_ego
    lon = rx * hx_ego + rz * hz_ego

    gap = np.maximum(lon - 2.3 - leader_half_len, 0.0)
    dv = v_ego - v_leader
    closing = valid & (dv > 0.1)

    safe_dv = np.where(closing, dv, 1.0)
    ttc = np.where(closing, gap / safe_dv, np.inf)
    min_val = np.minimum(cap, np.min(ttc, axis=1))

    crashed = np.array([len(ego_events(R, k)) > 0 for k in range(K)])
    return np.where(crashed, 0.0, min_val)


def contact_type(R: core.Rollout, k: int, event: dict) -> str:
    other = event["b"] if event["a"] == "ego" else event["a"]
    other_idx = core.IX[other]
    f = event["f"]

    ego = R.F[k, f, core.IX["ego"]]
    other_actor = R.F[k, f, other_idx]

    rx = other_actor[0] - ego[0]
    rz = other_actor[1] - ego[1]
    hx, hz = ego[2], ego[3]

    lon = rx * hx + rz * hz
    lat = -rx * hz + rz * hx

    if lon > 2.3 and abs(lat) < 1.6:
        return "front"
    if lon < -2.3 and abs(lat) < 1.6:
        return "rear"
    return "side"


def progress_metrics(R: core.Rollout) -> tuple[np.ndarray, np.ndarray]:
    ego_idx = core.IX["ego"]
    ego_pos = R.F[:, :, ego_idx, :2]
    diff = np.diff(ego_pos, axis=1)
    dist = np.sum(np.hypot(diff[..., 0], diff[..., 1]), axis=1)

    ego_accel = R.F[:, :, ego_idx, 5]
    harsh = np.sum(ego_accel < -3.0, axis=1)
    return dist, harsh


# ---------------------------------------------------------------- given helpers

def pair_min_ttc(R, follower, leader, cap=10.0, lat_max=2.2, leader_vmin=-1.0):
    """(K,) smallest TTC of `follower` running into `leader` while they share a corridor."""
    F = R.F
    a, b = F[:, :, core.IX[follower]], F[:, :, core.IX[leader]]
    rx, rz = b[..., 0] - a[..., 0], b[..., 1] - a[..., 1]
    lon = rx * a[..., 2] + rz * a[..., 3]
    lat = -rx * a[..., 3] + rz * a[..., 2]
    gap = np.maximum(lon - core.LEN[core.IX[follower]] / 2 - core.LEN[core.IX[leader]] / 2, 0.0)
    dv = a[..., 4] - b[..., 4]
    ok = (lon > 0) & (np.abs(lat) < lat_max) & (dv > 0.1) & (b[..., 4] >= leader_vmin)
    ttc = np.where(ok, gap / np.maximum(dv, 1e-6), np.inf)
    return np.minimum(cap, ttc.min(1))


def ped_room(R, cap=30.0):
    """(K,) smallest stopping room [m] left while the crossing pedestrian is in the ego's lane
    (distance to them minus the ego's braking distance at mu*g). Negative = could not have stopped."""
    e, p = R.F[:, :, core.IX["ego"]], R.F[:, :, core.IX["P1"]]
    mu = np.asarray(R.params["mu"])[:, None]
    inlane = np.abs(p[..., 1] - e[..., 1]) < 1.9
    ahead = p[..., 0] - e[..., 0] - HALF_EGO
    room = np.where(inlane & (ahead > -1.0), ahead - e[..., 4] ** 2 / (2 * mu * G), cap)
    return np.clip(room.min(1), -cap, cap)
