"""Public tests for the 10 TODO functions. Synthetic rollouts with answers you can work out by hand."""
import math

import numpy as np
import pytest

from avsim import core, space
from lab11 import estimate, feasible, metrics, modes, policy, search

EGO, C2, TG, P1 = core.IX["ego"], core.IX["C2"], core.IX["TG"], core.IX["P1"]


def fake_rollout(K=1, events=None, mu=0.8):
    """Everyone parked far away, heading east; tests then place the actors they need."""
    F = np.zeros((K, core.NF, core.N, 7), dtype=np.float32)
    F[..., 0] = np.arange(core.N)[None, None, :] * 100.0 + 1000.0
    F[..., 2] = 1.0
    E = np.full((K, core.NF, 4), 0.0, dtype=np.float32)
    E[..., 3] = -1
    P, _ = core.as_batch([dict(mu=mu)] * K)
    return core.Rollout(F=F, EGO=E, events=events or [[] for _ in range(K)], params=P, policy={}, ego="planner", others="reactive")


# ---------------------------------------------------------------- TODO 1
def test_min_ttc_closing_on_a_leader():
    R = fake_rollout()
    t = np.arange(core.NF) * core.DT
    R.F[0, :, EGO, 0] = 12.0 * t            # ego at 12 m/s
    R.F[0, :, EGO, 4] = 12.0
    R.F[0, :, C2, 0] = 40.0 + 8.0 * t       # leader 40 m ahead (centre to centre) at 8 m/s
    R.F[0, :, C2, 4] = 8.0
    R.EGO[0, :, 3] = C2
    # bumper gap at t: 40 - 4t - 4.6 ; TTC = gap / 4 ; smallest before contact is 0 at gap 0 (t = 8.85 s)
    ttc = metrics.min_ttc(R)
    assert ttc.shape == (1,)
    assert ttc[0] == pytest.approx(0.0, abs=0.02)


def test_min_ttc_cap_when_not_closing():
    R = fake_rollout()
    R.F[0, :, EGO, 0] = 0.0
    R.F[0, :, EGO, 4] = 10.0
    R.F[0, :, C2, 0] = 50.0
    R.F[0, :, C2, 4] = 12.0
    R.EGO[0, :, 3] = C2
    assert metrics.min_ttc(R, cap=10.0)[0] == pytest.approx(10.0)


def test_min_ttc_value():
    R = fake_rollout()
    R.F[0, :, EGO, 0] = 0.0
    R.F[0, :, EGO, 4] = 10.0
    R.F[0, :, C2, 0] = 34.6                 # bumper gap 30 m
    R.F[0, :, C2, 4] = 7.0                  # closing at 3 m/s
    R.EGO[0, 100:, 3] = C2                  # leader known only from frame 100
    assert metrics.min_ttc(R)[0] == pytest.approx(10.0, abs=1e-3)
    R.F[0, :, C2, 4] = 5.0
    assert metrics.min_ttc(R)[0] == pytest.approx(6.0, abs=1e-3)


def test_min_ttc_crash_is_zero():
    R = fake_rollout(events=[[dict(t=5.0, f=100, a="ego", b="C2", x=0, z=0, fake=False)]])
    assert metrics.min_ttc(R)[0] == 0.0


# ---------------------------------------------------------------- TODO 2
@pytest.mark.parametrize("dx,dz,want", [(4.0, 0.2, "front"), (-4.0, -0.3, "rear"), (0.5, 1.9, "side"), (3.0, 2.5, "side")])
def test_contact_type(dx, dz, want):
    R = fake_rollout()
    f = 50
    R.F[0, f, EGO, :4] = [100.0, 1.75, 1.0, 0.0]
    R.F[0, f, C2, :4] = [100.0 + dx, 1.75 + dz, 1.0, 0.0]
    assert metrics.contact_type(R, 0, dict(f=f, a="ego", b="C2")) == want


def test_contact_type_uses_ego_heading():
    R = fake_rollout()
    f = 10
    R.F[0, f, EGO, :4] = [0.0, 0.0, 0.0, -1.0]      # ego heading north (z decreasing)
    R.F[0, f, TG, :4] = [0.0, 4.0, 0.0, -1.0]       # TG south of it = behind
    assert metrics.contact_type(R, 0, dict(f=f, a="TG", b="ego")) == "rear"


# ---------------------------------------------------------------- TODO 9
def test_progress_metrics():
    R = fake_rollout(K=2)
    t = np.arange(core.NF) * core.DT
    R.F[:, :, EGO, 0] = 10.0 * t
    R.F[1, 200:220, EGO, 5] = -4.0
    dist, harsh = metrics.progress_metrics(R)
    assert dist == pytest.approx([240.0, 240.0], abs=1e-3)
    assert list(harsh) == [0, 20]


# ---------------------------------------------------------------- TODO 3
BASE = dict(cutDur=4.0, mu=0.8, cutBrake=0.0, pedV=1.4, cutV=14.0)


@pytest.mark.parametrize("change,ok", [({}, True), (dict(cutDur=1.2), False), (dict(cutDur=1.6), True),
                                       (dict(cutDur=1.6, mu=0.5), False), (dict(cutBrake=8.5), False),
                                       (dict(cutBrake=5.0, mu=0.5), False), (dict(cutBrake=4.5, mu=0.5), True),
                                       (dict(pedV=2.6), False), (dict(cutV=23.0), False), (dict(cutV=0.0), False)])
def test_is_feasible(change, ok):
    assert feasible.is_feasible({**BASE, **change}) is ok


# ---------------------------------------------------------------- TODO 4
def test_wilson_zero_events():
    lo, hi = estimate.wilson_ci(0, 100)
    assert lo == 0.0 and 0.03 < hi < 0.045


def test_wilson_known_value():
    lo, hi = estimate.wilson_ci(10, 100)
    assert lo == pytest.approx(0.0552, abs=5e-4) and hi == pytest.approx(0.1744, abs=5e-4)


def test_wilson_empty():
    assert estimate.wilson_ci(0, 0) == (0.0, 1.0)


# ---------------------------------------------------------------- TODO 5
def test_cem_update_moves_to_the_elite():
    rng = np.random.default_rng(0)
    Z = rng.standard_normal((200, space.D))
    Bn = np.zeros((200, space.B), int)
    score = Z[:, 0].copy()                    # danger grows with coordinate 0
    Bn[score > np.quantile(score, 0.9), 1] = 1
    prev = search.natural_proposal(space.Natural())
    new = search.cem_update(Z, Bn, score, prev, rho=0.1, smooth=1.0)
    elite = np.argsort(-score)[:20]
    assert new["mean"][0] == pytest.approx(Z[elite, 0].mean())
    assert new["std"][0] == pytest.approx(max(0.25, Z[elite, 0].std()))
    assert new["p_bin"][1] == pytest.approx(0.97)          # clipped
    assert new["p_bin"][0] == pytest.approx(0.03)
    assert new["level"] == pytest.approx(score[elite].min())


def test_cem_update_smoothing_and_floor():
    Z = np.zeros((10, space.D))
    Bn = np.zeros((10, space.B), int)
    prev = dict(mean=np.ones(space.D), std=np.ones(space.D), p_bin=np.full(space.B, 0.5), level=-5.0)
    new = search.cem_update(Z, Bn, np.arange(10.0), prev, rho=0.2, smooth=0.7)
    assert np.allclose(new["mean"], 0.3)
    assert np.allclose(new["std"], 0.3)                    # 0.7 * 0 + 0.3 * 1, above the floor
    assert np.allclose(new["p_bin"], 0.15)


# ---------------------------------------------------------------- TODO 6
def test_log_weights_zero_when_proposal_is_natural():
    nat = space.Natural()
    rng = np.random.default_rng(1)
    Z, Bn = nat.sample(50, rng)
    lw = estimate.log_weights(Z, Bn, nat, search.natural_proposal(nat), beta=0.2)
    assert np.allclose(lw, 0.0, atol=1e-9)


def test_log_weights_bounded_by_defensive_mixture():
    nat = space.Natural()
    prop = dict(mean=np.full(space.D, 3.0), std=np.full(space.D, 0.3), p_bin=np.full(space.B, 0.5), level=0)
    Z, Bn = nat.sample(500, np.random.default_rng(2))
    lw = estimate.log_weights(Z, Bn, nat, prop, beta=0.2)
    assert np.all(lw <= math.log(1 / 0.2) + 1e-9)          # p/q <= 1/beta always
    assert np.all(np.isfinite(lw))


# ---------------------------------------------------------------- TODO 7
def test_is_estimate_reduces_to_plain_mean():
    y = np.array([0, 1, 0, 0, 1, 0, 0, 0, 0, 0], float)
    e = estimate.is_estimate(np.zeros(10), y)
    assert e["p"] == pytest.approx(0.2)
    assert e["ess"] == pytest.approx(10)
    assert e["se"] == pytest.approx(np.std(y, ddof=1) / math.sqrt(10))
    assert e["lo"] >= 0.0 and e["n"] == 10


def test_is_estimate_is_not_self_normalised():
    lw = np.log(np.array([0.1, 0.1, 0.1, 0.1]))
    e = estimate.is_estimate(lw, np.array([1.0, 0, 0, 0]))
    assert e["p"] == pytest.approx(0.025)                  # mean(w*y), not sum(w*y)/sum(w) = 0.25


# ---------------------------------------------------------------- TODO 8
def _crash(other, f=40, c2=(100.0, 3.0)):
    R = fake_rollout(events=[[dict(t=f * core.DT, f=f, a="ego", b=other, x=0, z=0, fake=False)]])
    R.F[0, f, EGO, :4] = [100.0, 1.75, 1.0, 0.0]
    R.F[0, f, C2, 0], R.F[0, f, C2, 4] = c2
    R.F[0, f, C2, 1] = 1.75
    R.F[0, f, TG, :4] = [95.5, 1.75, 1.0, 0.0]
    return R, R.events[0][0]


def test_label_mode_cases():
    R, e = _crash("P1")
    assert modes.label_mode(R, 0, e, metrics.contact_type) == "crosswalk_block"
    R, e = _crash("TG")
    assert modes.label_mode(R, 0, e, metrics.contact_type) == "rear_ended"
    R, e = _crash("C2", c2=(104.5, 3.0))
    assert modes.label_mode(R, 0, e, metrics.contact_type) == "bus_bypass"
    R, e = _crash("C2", c2=(150.0, 3.0))
    assert modes.label_mode(R, 0, e, metrics.contact_type) == "cut_in"
    R, e = _crash("C2", c2=(104.5, 9.0))
    assert modes.label_mode(R, 0, e, metrics.contact_type) == "cut_in"
    R, e = _crash("W1a")
    assert modes.label_mode(R, 0, e, metrics.contact_type) == "other"


# ---------------------------------------------------------------- TODO 10
def test_choose_policy():
    t = [dict(policy="a", fail_hi=1e-4, progress=200.0), dict(policy="b", fail_hi=5e-5, progress=180.0),
         dict(policy="c", fail_hi=5e-5, progress=209.0), dict(policy="d", fail_hi=1e-6, progress=150.0)]
    assert policy.choose_policy(t, 210.0)["policy"] == "c"            # d loses too much progress; c beats b on progress
    assert policy.choose_policy(t, 210.0, max_loss=0.30)["policy"] == "d"
    assert policy.choose_policy(t, 1000.0) is None
