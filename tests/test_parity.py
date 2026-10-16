"""Python avsim and the viewer's sim-core.js must produce the same world (needs node; skipped without it)."""
import json, os, shutil, subprocess, tempfile
import numpy as np
import pytest
from avsim import core

HERE = os.path.dirname(__file__)
JS_CORE = os.environ.get("SIMCORE_JS", os.path.join(HERE, "..", "viewer", "sim-core.js"))     # the viewer shipped with the lab


def _random_params(n, seed):
    r = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        out.append(dict(cutT0=float(r.uniform(1, 12)), cutDur=float(np.clip(4 * np.exp(0.35 * r.normal()), 1, 9)),
                        cutV=float(14 + 1.5 * r.normal()), cutDs=float(r.uniform(-12, 10)), pedT0=float(r.uniform(5, 17)),
                        pedV=float(max(0.6, 1.35 + 0.2 * r.normal())), busGo=float(r.uniform(12, 22)), tgT=float(r.uniform(0.6, 1.6)),
                        mu=0.5 if r.random() < 0.2 else 0.8, cutGap=float(12 * np.exp(0.5 * r.normal())),
                        pedGap=float(4.5 * np.exp(0.35 * r.normal())), cutBrake=float(1.5 + 2.5 * r.random()) if r.random() < 0.15 else 0.0,
                        pedLook=1 if r.random() < 0.85 else 0, civil=True))
    return out


def _js(jobs):
    with tempfile.TemporaryDirectory() as d:
        i, o = os.path.join(d, "in.json"), os.path.join(d, "out.json")
        json.dump(jobs, open(i, "w"))
        subprocess.run(["node", os.path.join(HERE, "js_dump.js"), os.path.abspath(JS_CORE), i, o], check=True)
        return json.load(open(o))


def _compare(R, js, tol=2e-3):
    bad = []
    for k, J in enumerate(js):
        Fj = np.array(J["F"], dtype=np.float32).reshape(core.NF, R.F.shape[2], 7)
        err = np.abs(R.F[k] - Fj).max()
        if "EGO" in J:          # planner state (mode, v0, sees-ped, leader): discrete, so it must match exactly
            Ej = np.array(J["EGO"], dtype=np.float32).reshape(core.NF, 4)
            err = max(err, 10 * float(np.abs(R.EGO[k] - Ej).max()))
        ej = [(e["a"], e["b"], e["f"], e["kind"]) for e in J["events"]]
        ep = [(e["a"], e["b"], e["f"], e["kind"]) for e in R.events[k]]
        if err > tol or ej != ep:
            bad.append((k, float(err), ej, ep))
    return bad


pytestmark = pytest.mark.skipif(shutil.which("node") is None or not os.path.exists(JS_CORE), reason="node or sim-core.js missing")


def test_viewer_story_matches():
    jobs = [dict(params={}, ego=e, others="reactive") for e in ("planner", "human")]
    js = _js(jobs)
    for job, J in zip(jobs, js):
        R = core.rollout({}, ego=job["ego"])
        assert not _compare(R, [J])


def test_replay_matches():
    js = _js([dict(params={}, ego="planner", others="replay")])
    log = core.rollout({}, ego="human")
    R = core.rollout({}, ego="planner", others="replay", log=log)
    assert not _compare(R, js)
    assert [(e["a"], e["b"]) for e in R.events[0]] == [("ego", "TG")]


@pytest.mark.parametrize("seed", [1, 2])
def test_natural_scenarios_match(seed):
    ps = _random_params(40, seed)
    pol = [dict(cutinLat=float(c), cautionV=float(cv)) for c, cv in zip(np.linspace(2.15, 3.2, 40), np.linspace(5, 9, 40))]
    js = _js([dict(params=p, policy=q, ego="planner", others="reactive") for p, q in zip(ps, pol)])
    R = core.rollout(ps, policy=pol)
    bad = _compare(R, js)
    assert len(bad) == 0, bad[:3]


HAZARDS = [dict(type="ped", x=150.0, side="S", trig=28.0), dict(type="stalled", x=300.0, lane="E1"),
           dict(type="brake", who="L1", t0=4.0, trig=None, decel=7.0, dur=2.0), dict(type="bike", x=215.0, trig=20.0, swerve=2.4)]


@pytest.mark.parametrize("seed", [3, 4])
def test_hazard_cards_match(seed):
    """The student's hazard cards, with per-scenario values (h{i}_{field}) as a hunt would draw them."""
    r = np.random.default_rng(seed)
    ps = _random_params(30, seed)
    for p in ps:
        p.update(h0_x=float(r.uniform(90, 240)), h0_v=float(r.uniform(0.8, 3.0)), h0_trig=float(r.uniform(8, 40)),
                 h2_t0=float(r.uniform(0, 12)), h2_decel=float(r.uniform(3, 9)), h3_x=float(r.uniform(120, 300)),
                 h3_trig=float(r.uniform(5, 30)), h3_swerve=float(r.uniform(0.5, 3.5)))
    js = _js([dict(params=p, ego="planner", others="reactive", hazards=HAZARDS) for p in ps])
    R = core.rollout(ps, hazards=HAZARDS)
    assert R.F.shape[2] == core.N + 3                       # the brake card changes L1, it adds no actor
    bad = _compare(R, js)
    assert len(bad) == 0, bad[:3]
    assert sum(len(e) for e in R.events) > 0, "no collision at all: the test would not exercise event order"


def test_scripted_ego_matches():
    """Chapter 10's "you drive" mode: recorded pedal input instead of the planner."""
    t = np.arange(core.NF) * core.DT
    pedal = [np.where(t < 6, 0.8, np.where(t < 9, -3.0, 0.4)), np.full(core.NF, 1.2), np.sin(t / 2) * 2.5]
    ps = [dict(civil=True)] * 3
    js = _js([dict(params=p, ego="script", others="reactive", egoAcc=list(map(float, a)), hazards=HAZARDS[:1]) for p, a in zip(ps, pedal)])
    R = core.rollout(ps, ego="script", ego_acc=np.stack(pedal), hazards=HAZARDS[:1])
    assert not _compare(R, js)


def test_two_brake_cards_on_one_car_both_apply():
    """Two brake cards on the same car: the viewer must not let the second overwrite the first."""
    hz = [dict(type="brake", who="L1", t0=3.0, trig=None, decel=4.0, dur=1.5),
          dict(type="brake", who="L1", t0=9.0, trig=None, decel=6.0, dur=1.5)]
    ps = _random_params(6, 11)
    js = _js([dict(params=p, ego="planner", others="reactive", hazards=hz) for p in ps])
    R = core.rollout(ps, hazards=hz)
    assert not _compare(R, js)
    assert (R.fire >= 0).all(), "both cards should fire in every scenario"
    l1 = core.IX["L1"]
    for i, h in enumerate(hz):                      # the car really brakes at least that hard right after each fires
        f = R.fire[:, i]
        lim = np.maximum(-np.minimum(9.0, R.params["mu"] * core.G), -h["decel"])      # wet road: mu*g caps it
        assert (R.F[np.arange(R.K), f, l1, 5] <= lim + 1e-4).all()


def test_brake_from_keeps_prefix_identical():
    """The oracle's run is the planner's run, bit for bit, until the brake frame."""
    ps = _random_params(8, 5)
    R = core.rollout(ps, hazards=HAZARDS)
    f0 = np.array([40, 80, 120, -1, 60, 200, 10, 300])
    O = core.rollout(ps, hazards=HAZARDS, brake_from=f0)
    for k, f in enumerate(f0):
        upto = core.NF if f < 0 else f
        assert np.array_equal(R.F[k, :upto], O.F[k, :upto])
        if 0 <= f < core.NF - 1 and O.F[k, f, core.EGO, 4] > 0:
            assert np.isclose(O.F[k, f, core.EGO, 5], -min(9.0, O.params["mu"][k] * core.G), atol=1e-4)


# wet road, a pedestrian who does not peek past the bus: the planner stops with its nose over the crosswalk
BLOCKS = [
    {'cutT0': 7.6024, 'cutDur': 4.9515, 'cutV': 12.2896, 'cutDs': -1.3756, 'pedT0': 14.5161, 'pedV': 0.9015, 'busGo': 12.1112, 'tgT': 1.08, 'cutGap': 28.4059, 'pedGap': 2.552, 'mu': 0.5, 'pedLook': 0.0, 'cutBrake': 0.0, 'civil': True},
    {'cutT0': 2.047, 'cutDur': 4.2911, 'cutV': 11.0264, 'cutDs': -8.9041, 'pedT0': 14.7874, 'pedV': 0.785, 'busGo': 12.2008, 'tgT': 0.7814, 'cutGap': 26.0303, 'pedGap': 2.2003, 'mu': 0.5, 'pedLook': 0.0, 'cutBrake': 0.0, 'civil': True},
    {'cutT0': 7.0633, 'cutDur': 4.1274, 'cutV': 12.6183, 'cutDs': -6.8112, 'pedT0': 14.3808, 'pedV': 0.984, 'busGo': 12.0739, 'tgT': 0.7817, 'cutGap': 20.4301, 'pedGap': 2.7065, 'mu': 0.5, 'pedLook': 0.0, 'cutBrake': 0.0, 'civil': True},
    {'cutT0': 3.5475, 'cutDur': 3.5493, 'cutV': 11.7116, 'cutDs': -10.7018, 'pedT0': 14.6611, 'pedV': 0.8954, 'busGo': 12.1618, 'tgT': 0.6317, 'cutGap': 18.8476, 'pedGap': 2.313, 'mu': 0.5, 'pedLook': 0.0, 'cutBrake': 0.0, 'civil': True},
]


def test_crosswalk_block_matches():
    """A pedestrian never walks into a car body; the ego stopping on the crosswalk in its way is a 'block' event."""
    js = _js([dict(params=p, ego="planner", others="reactive") for p in BLOCKS])
    R = core.rollout(BLOCKS)
    assert not _compare(R, js)
    for k in range(R.K):
        ev = [e for e in R.events[k] if "ego" in (e["a"], e["b"])]
        assert ev and ev[0]["b"] == "P1" and ev[0]["kind"] == "block", ev
        p1 = R.F[k, ev[0]["f"]:, core.IX["P1"]]
        assert np.all(p1[:, 4] == 0) or np.argmax(p1[:, 4] > 0) > 0      # the pedestrian waits instead of walking on
