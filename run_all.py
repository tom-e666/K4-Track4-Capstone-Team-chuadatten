"""Lab 11 "Scenario Forge": run one step at a time, or `all`.

    python run_all.py world        # 0. check the simulator and the viewer agree
    python run_all.py replay       # A. log replay vs reactive agents on 400 natural scenarios
    python run_all.py mc           # C1. plain Monte Carlo: how often does the planner crash?
    python run_all.py hunt         # C2/D. one cross-entropy search per failure mode
    python run_all.py estimate     # C3. importance sampling per mode, 3 seeds
    python run_all.py fix          # E. tune the policy on TRAIN scenarios, report on TEST scenarios
    python run_all.py export       # F. evidence/viewer.json for the 3D viewer, chapter 9
    python run_all.py mine hazard.json   # G. your own hazard from the viewer's chapter 10 studio
    python run_all.py all

Your MSSV goes in REPORT.md (first line `MSSV: ...`). It sets your town's natural distribution and your seeds.
Every step writes evidence/<step>.json; the grader re-runs everything with your MSSV.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

import numpy as np

from avsim import core, space, studio, viewer
from lab11 import estimate, feasible, metrics, modes, policy, search

HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.join(HERE, "evidence")
MODES = ["bus_bypass", "cut_in", "crosswalk_block", "rear_ended"]
BETA = 0.2


# ---------------------------------------------------------------- plumbing (given)

def mssv():
    if os.environ.get("LAB11_MSSV"):          # the grader passes the roster MSSV; REPORT.md is only the student's copy
        return os.environ["LAB11_MSSV"]
    with open(os.path.join(HERE, "REPORT.md"), encoding="utf-8") as f:
        first = next((ln for ln in f if ln.strip()), "")
    m = re.match(r"\s*MSSV:[ \t]*(\d{6,12})\s*$", first)
    if not m:
        sys.exit("Write your MSSV on the first line of REPORT.md, digits only (MSSV: 2xxxxxxx).")
    return m.group(1)


RUN = {}     # provenance of this invocation, stamped into every evidence file (see main)


def save(name, obj):
    os.makedirs(EVID, exist_ok=True)
    if isinstance(obj, dict) and RUN:
        obj = dict(obj, _run=RUN)
    with open(os.path.join(EVID, name + ".json"), "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o))
    print(f"  -> evidence/{name}.json")


NEEDS = dict(world="world", replay="replay", mc="mc", hunt="hunt", estimate="hunt", fix="hunt", export="hunt",
             cloud="hunt", estimate_mc="mc")


def load(name):
    path = os.path.join(EVID, name + ".json")
    if not os.path.exists(path):
        sys.exit(f"evidence/{name}.json is missing: run `python run_all.py {NEEDS.get(name, name)}` first.")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


_WARNED = set()
USED_IN = dict(min_ttc="replay", contact_type="replay", progress_metrics="fix", label_mode="hunt", is_feasible="every step")


def _todo(fn, *args, default=None):
    """Call a student function; until its TODO is done, return `default` (once-per-name warning) so earlier steps run."""
    try:
        return fn(*args)
    except NotImplementedError as e:
        if fn.__name__ not in _WARNED:
            _WARNED.add(fn.__name__)
            print(f"  (skipping {fn.__name__}: {e}; fields that need it are left empty. Only a problem in the step "
                  f"that needs it: {USED_IN.get(fn.__name__, 'see README')})")
        return default


def _dist_to_crash(R, k, f):
    """Distance the ego covered up to frame f: travel after a crash is not progress."""
    e = R.F[k, : f + 1, core.EGO]
    return float(np.sum(np.hypot(np.diff(e[:, 0]), np.diff(e[:, 1]))))


def summarize(R, mode=None):
    """Per scenario: crash?, its mode, its contact type, min TTC, and (if a mode is given) that mode's danger score."""
    ttc = _todo(metrics.min_ttc, R, default=np.full(R.K, np.nan))
    dist, harsh = _todo(metrics.progress_metrics, R, default=(np.full(R.K, np.nan), np.zeros(R.K, int)))
    score = danger(R, mode) if mode else None
    out = []
    for k in range(R.K):
        ev = metrics.ego_events(R, k)
        row = dict(crash=bool(ev), ttc=float(ttc[k]), dist=float(dist[k]), harsh=int(harsh[k]), mode=None, contact=None)
        if ev:
            row["contact"] = _todo(metrics.contact_type, R, k, ev[0])
            row["mode"] = _todo(modes.label_mode, R, k, ev[0], metrics.contact_type, default="other")
            row["dist"] = min(row["dist"], _dist_to_crash(R, k, ev[0]["f"]))
        if score is not None:
            row["score"] = float(score[k])
        out.append(row)
    return out


def danger(R, mode):
    """Higher = closer to that mode's failure; a crash of that mode scores 0 (the maximum)."""
    if mode == "bus_bypass":
        s = -metrics.pair_min_ttc(R, "ego", "C2", lat_max=3.2)
    elif mode == "cut_in":
        s = -metrics.pair_min_ttc(R, "ego", "C2", lat_max=3.2, leader_vmin=6.0)
    elif mode == "crosswalk_block":
        s = -metrics.ped_room(R)
    elif mode == "rear_ended":
        s = -metrics.pair_min_ttc(R, "TG", "ego")
    else:
        s = -metrics.min_ttc(R)
    lab = np.array([modes.label_mode(R, k, ev[0], metrics.contact_type) if ev else None
                    for k, ev in ((k, metrics.ego_events(R, k)) for k in range(R.K))], dtype=object)
    return np.where(lab == mode, 0.0, np.minimum(s, -1e-3))


def simulate(params, mode=None, pol=None, others="reactive"):
    """Run the feasible scenarios, mark the rest infeasible (never simulated, never counted as failures)."""
    feas = np.array([_todo(feasible.is_feasible, p, default=True) for p in params])
    res = [dict(crash=False, ttc=10.0, dist=0.0, harsh=0, mode=None, contact=None, score=-math.inf, infeasible=True)
           for _ in params]
    idx = np.nonzero(feas)[0]
    if len(idx):
        got = core.run_many([params[i] for i in idx], lambda R: summarize(R, mode), policy=pol, others=others, chunk=512)
        for i, g in zip(idx, got):
            res[i] = g
    return res, feas


def prop_to_json(p):
    return {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in p.items()}


def prop_from_json(d):
    return dict(mean=np.array(d["mean"]), std=np.array(d["std"]), p_bin=np.array(d["p_bin"]), level=d.get("level", 0.0))


# ---------------------------------------------------------------- steps

def step_world(a):
    t = time.time()
    R = core.rollout([{}, {}], policy=None, ego="planner")
    log = core.rollout([{}, {}], ego="human")
    Rp = core.rollout([{}, {}], ego="planner", others="replay", log=log)
    out = dict(reactive_events=R.events[0], replay_events=Rp.events[0], progress_default=_dist_to_crash(R, 0, core.NF - 1),
               seconds=time.time() - t)        # uses no TODO: this step only checks your install
    assert [(e["a"], e["b"]) for e in Rp.events[0]] == [("ego", "TG")], "the viewer's replay story should end in ego|TG"
    print(f"  reactive: {len(R.events[0])} collisions; replay: {[(e['a'], e['b'], e['t']) for e in Rp.events[0]]}")
    save("world", out)


def step_replay(a, nat, seeds):
    """A. The same 400 scenarios three ways: the human log, the planner against replayed traffic, the planner in reactive traffic."""
    rng = np.random.default_rng(seeds["mc"] + 1)
    Z, Bn = nat.sample(a.replay_n, rng)
    params = space.to_params(Z, Bn, nat)
    log_res, _ = simulate(params, others="reactive", pol=None)
    human = core.run_many(params, lambda R: [bool(metrics.ego_events(R, k)) for k in range(R.K)], ego="human")
    keep = [i for i, h in enumerate(human) if not h]          # a real log never contains the human's own crash
    ps = [params[i] for i in keep]
    rep, _ = simulate(ps, others="replay")
    rea, _ = simulate(ps, others="reactive")
    def tally(res):
        c = [r for r in res if r["crash"]]
        types = {}
        for r in c:
            types[r["contact"]] = types.get(r["contact"], 0) + 1
        return dict(crashes=len(c), contact=types)
    both = sum(1 for x, y in zip(rep, rea) if x["crash"] and y["crash"])
    out = dict(n=len(params), dropped_human_crash_logs=len(params) - len(keep), n_used=len(ps),
               replay=tally(rep), reactive=tally(rea), crash_in_both=both,
               # crashes that exist only against the replayed log: what hit what, and the scenario itself (the viewer's
               # chapter 9 plays the first three of them with others="replay" after `export`)
               replay_only=[dict(scenario=keep[i], contact=x["contact"], min_ttc_reactive=y["ttc"], params=ps[i])
                            for i, (x, y) in enumerate(zip(rep, rea)) if x["crash"] and not y["crash"]][:20])
    print(f"  replay: {out['replay']}  reactive: {out['reactive']}  both: {both}")
    save("replay", out)


def step_mc(a, nat, seeds):
    """C1. Plain Monte Carlo under the natural distribution."""
    rng = np.random.default_rng(seeds["mc"])
    Z, Bn = nat.sample(a.mc_n, rng)
    t = time.time()
    res, feas = simulate(space.to_params(Z, Bn, nat))
    k = sum(r["crash"] for r in res)
    lo, hi = estimate.wilson_ci(k, a.mc_n)
    by = {}
    for r in res:
        if r["crash"]:
            by[r["mode"]] = by.get(r["mode"], 0) + 1
    per = {m: dict(k=by.get(m, 0), lo=estimate.wilson_ci(by.get(m, 0), a.mc_n)[0], hi=estimate.wilson_ci(by.get(m, 0), a.mc_n)[1])
           for m in MODES + ["other"]}       # every mode, seen or not: an unseen one is reported by its upper bound
    out = dict(n=a.mc_n, crashes=k, p=k / a.mc_n, lo=lo, hi=hi, modes=by, per_mode=per, infeasible=int((~feas).sum()),
               seconds=time.time() - t)
    print(f"  {k} crashes in {a.mc_n}: p = {k / a.mc_n:.2e}, 95% Wilson [{lo:.2e}, {hi:.2e}], modes {by}")
    save("mc", out)


def hunt(mode, nat, rng, iters, per_iter, cloud=None, pol=None):
    prop = search.natural_proposal(nat)
    trace, found, infeas = [], {}, 0
    prop_est = None      # the proposal used for ESTIMATION: the first one whose elite are already crashes
    for it in range(iters):
        Z, Bn = search.proposal_sample(prop, per_iter, rng)
        params = space.to_params(Z, Bn, nat)
        res, feas = simulate(params, mode=mode, pol=pol)
        infeas += int((~feas).sum())
        sc = np.array([r["score"] for r in res])
        for p, r, f in zip(params, res, feas):
            if r["crash"]:
                found[r["mode"]] = found.get(r["mode"], 0) + 1
            if cloud is not None:
                cloud.append(dict(params=p, it=it + 1, mode=r["mode"],
                                  out="infeasible" if not f else "crash" if r["crash"] else "near" if r["score"] > -1.5 else "safe"))
        if feas.sum() >= 4:
            prop = search.cem_update(Z[feas], Bn[feas], sc[feas], prop)
            if prop_est is None and prop["level"] >= -1e-3:
                prop_est = {k: (v.copy() if hasattr(v, "copy") else v) for k, v in prop.items()}
        trace.append(dict(it=it, level=prop["level"], crashes=int(sum(r["crash"] for r in res)), infeasible=int((~feas).sum())))
        print(f"    {mode:11s} round {it + 1}: elite level {prop['level']:7.2f}  crashes {trace[-1]['crashes']:3d}  infeasible {trace[-1]['infeasible']}")
    # Searching on after that keeps tightening q around the failures already found: more examples for the viewer,
    # but a proposal that overlaps the failure region badly and gives estimates that are typically too LOW.
    return (prop_est or prop), dict(trace=trace, found=found, infeasible=infeas, stopped_tightening=prop_est is not None)


def step_hunt(a, nat, seeds):
    """C2/D. One CEM search per mode. The proposals are saved for the estimate and fix steps."""
    rng = np.random.default_rng(seeds["search"])
    out, cloud = {}, []
    for m in MODES:
        prop, info = hunt(m, nat, rng, a.iters, a.per_iter, cloud)
        out[m] = dict(proposal=prop_to_json(prop), **info)
        print(f"  hunt {m:10s}: found {info.get('found')}  infeasible skipped {info.get('infeasible')}")
    save("hunt", out)
    with open(os.path.join(EVID, "cloud.json"), "w") as f:
        json.dump(cloud, f)


def is_mode(mode, prop, nat, rng, n, pol=None):
    Z, Bn = search.mixture_sample(prop, nat, n, rng, BETA)
    params = space.to_params(Z, Bn, nat)
    res, feas = simulate(params, pol=pol)
    y = np.array([r["mode"] == mode for r in res], dtype=float)
    lw = estimate.log_weights(Z, Bn, nat, prop, BETA)
    est = estimate.is_estimate(lw, y)
    est["hits"] = int(y.sum())
    wy = np.exp(lw) * y
    # degeneracy check: if one sample carries most of the estimate, the number is that one sample's weight, not a rate
    est["top_share"] = float(wy.max() / wy.sum()) if wy.sum() > 0 else 0.0
    if est["hits"] == 0:
        # Valid 95% bound for zero hits: every draw comes from q >= BETA * p, so q(mode) >= BETA * p(mode) and
        # P(0 hits) <= (1 - BETA p)^n. That gives p <= 3 / (BETA n). (3/ESS is NOT a bound here: ESS measures how
        # uneven the weights are, not how many independent tries the event had.)
        est["upper_zero_hits"] = 3.0 / (BETA * n)
    return est, params, res


def step_estimate(a, nat, seeds):
    """C3. Importance sampling per mode, 3 seeds; total = sum over modes (the modes split the crashes)."""
    H = load("hunt")
    # Crashes no mode claims ("other") are not hunted, so their share comes from the plain Monte Carlo of step C1.
    # Leaving them out would make the total a sum over a partition that is not complete.
    try:
        mc = load("mc")
        k_o, n_o = mc["modes"].get("other", 0), mc["n"]
    except FileNotFoundError:
        k_o, n_o = 0, 0
    other = dict(k=k_o, n=n_o, p=k_o / n_o if n_o else 0.0,
                 hi=estimate.wilson_ci(k_o, n_o)[1] if n_o else float("nan"), source="mc.json")
    out = []
    for s in range(a.seeds):
        rng = np.random.default_rng(seeds["test"] + 101 * s)
        per = {}
        for m in MODES:
            per[m], _, _ = is_mode(m, prop_from_json(H[m]["proposal"]), nat, rng, a.final_n)
        tot = sum(per[m]["p"] for m in MODES) + other["p"]
        se_o = math.sqrt(other["p"] * (1 - other["p"]) / n_o) if n_o else 0.0
        se = math.sqrt(sum(per[m]["se"] ** 2 for m in MODES) + se_o ** 2)
        # 0 "other" crashes in MC still leave up to other["hi"] of them: the total's upper end must carry that bound
        hi_tot = tot + 1.96 * se + (other["hi"] if n_o and k_o == 0 else 0.0)
        out.append(dict(seed=s, per=per, total=tot, lo=max(0.0, tot - 1.96 * se), hi=hi_tot))
        print(f"  seed {s}: total {tot:.2e} [{max(0, tot - 1.96 * se):.2e}, {hi_tot:.2e}]  " +
              "  ".join(f"{m} {per[m]['p']:.1e} (hits {per[m]['hits']}, ess {per[m]['ess']:.0f})" for m in MODES))
    save("estimate", dict(seeds=out, other=other, budget=len(MODES) * (a.iters * a.per_iter + a.seeds * a.final_n)))


# Knob ranges a real planner of this car could be given. Outside them a "policy" is not a planner setting.
POLICY_LIMITS = dict(v0=(5.0, 16.0), T=(0.8, 3.0), a=(0.5, 3.0), b=(1.0, 6.0), s0=(1.0, 6.0), cautionV=(3.0, 13.0),
                     cautionRange=(20.0, 90.0), pedBuffer=(0.0, 6.0), cutinLat=(1.5, 3.6))


def candidates():
    path = os.path.join(HERE, "policy_candidates.json")
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)
    ok = []
    for c in raw:
        bad = [k for k, v in c.items() if k not in POLICY_LIMITS or not POLICY_LIMITS[k][0] <= float(v) <= POLICY_LIMITS[k][1]]
        if bad:
            print(f"  rejected candidate {c}: {', '.join(bad)} outside POLICY_LIMITS")
        else:
            ok.append(c)
    return ok


def eval_policy(pol, props, nat, rng, n, prog_params, test=False):
    """TRAIN (test=False) screens: a mode with zero hits is flagged `unseen` and adds nothing, because these draws
    were aimed where ANOTHER policy fails. TEST (test=True) reports: an unseen mode adds its valid zero-hit bound."""
    row = dict(policy=pol, unseen=[])
    p_tot, hi_tot, hi_seen = 0.0, 0.0, 0.0
    for m in ("bus_bypass", "crosswalk_block"):
        est, _, _ = is_mode(m, props[m], nat, rng, n, pol=pol)
        row[m] = est
        p_tot += est["p"]
        if est["hits"] > 0:
            hi_tot += est["hi"]
            hi_seen += est["hi"]
        elif test:
            hi_tot += est["upper_zero_hits"]
            row["unseen"].append(m)
        else:
            # Zero hits here proves little: these samples were drawn where ANOTHER policy fails. The only valid bound
            # (3/ESS, the rule of three) is about 4e-2 and would rank every real fix last. So TRAIN only screens:
            # the mode is flagged, and TEST re-hunts under the chosen policy to measure it.
            row["unseen"].append(m)
    # Every OTHER ego collision (cut-in, rear-end, "other"), counted on the natural progress draws: a policy that
    # moves its failures into modes nobody hunts must not look safe.
    pr, _ = simulate(prog_params, pol=pol)
    k_other = sum(1 for r in pr if r["crash"] and r["mode"] not in ("bus_bypass", "crosswalk_block"))
    lo_o, hi_o = estimate.wilson_ci(k_other, len(pr))
    row["other_crashes"] = dict(k=k_other, n=len(pr), hi=hi_o)
    p_tot += k_other / max(1, len(pr))
    hi_tot += hi_o if (k_other > 0 or test) else 0.0
    hi_seen += hi_o if k_other > 0 else 0.0
    row["fail"] = p_tot
    row["fail_hi"] = hi_tot      # TRAIN: over the modes actually seen. TEST: a valid 95% bound, including unseen modes
    row["fail_hi_seen"] = hi_seen    # only the modes with hits: what the data show, NOT a bound on the policy
    row["progress"] = float(np.mean([r["dist"] for r in pr]))       # distance up to a crash, not past it
    row["harsh"] = float(np.mean([r["harsh"] for r in pr]))
    return row


def step_fix(a, nat, seeds):
    """E. Tune on TRAIN importance samples, then report the chosen policy on TEST samples it never saw."""
    H = load("hunt")
    props = {m: prop_from_json(H[m]["proposal"]) for m in MODES}
    rng_p = np.random.default_rng(seeds["test"] + 7)
    Z, Bn = nat.sample(a.progress_n, rng_p)
    prog_params = space.to_params(Z, Bn, nat)
    base = eval_policy(dict(core.POLICY), props, nat, np.random.default_rng(seeds["search"] + 11), a.fix_n, prog_params)
    table = []
    for i, pol in enumerate(candidates()):
        row = eval_policy(pol, props, nat, np.random.default_rng(seeds["search"] + 11), a.fix_n, prog_params)  # same TRAIN draws for every candidate
        table.append(row)
        print(f"  [{i + 1:2d}] {pol}  TRAIN fail {row['fail']:.2e} (hi {row['fail_hi']:.2e})  progress {row['progress']:.1f} m"
              + (f"  unseen: {', '.join(row['unseen'])}" if row["unseen"] else ""))
    best = policy.choose_policy(table, base["progress"])
    out = dict(baseline_train=base, table=table, chosen=best)
    if best is not None:
        # TEST: fresh seeds AND a fresh search aimed at each policy. Samples drawn for the baseline can miss where
        # another policy fails, and then one heavy weight decides the number (see top_share).
        def retarget(pol, seed):
            rng = np.random.default_rng(seed)
            return {m: hunt(m, nat, rng, a.iters, a.per_iter, None, pol)[0] if m in ("bus_bypass", "crosswalk_block") else props[m]
                    for m in MODES}
        print("  re-hunting under the chosen policy and the baseline for the TEST estimate ...")
        # A natural holdout TRAIN never saw, for progress and for every crash outside the hunted modes. Bigger than
        # TRAIN's: with 0 such crashes in n runs the bound is about 3/n, so n must be in the thousands to say anything.
        Zt, Bt = nat.sample(a.mc_n, np.random.default_rng(seeds["test"] + 8))
        prog_test = space.to_params(Zt, Bt, nat)
        test = eval_policy(best["policy"], retarget(best["policy"], seeds["test"] + 31), nat,
                           np.random.default_rng(seeds["test"] + 999), a.fix_n, prog_test, test=True)
        btest = eval_policy(dict(core.POLICY), retarget({}, seeds["test"] + 31), nat,
                            np.random.default_rng(seeds["test"] + 999), a.fix_n, prog_test, test=True)
        out.update(chosen_test=test, baseline_test=btest)
        print(f"  chosen {best['policy']} (picked on TRAIN: fail {best['fail']:.2e}). Report these, from TEST:")
        for nm, r in (("chosen", test), ("baseline", btest)):
            print(f"    {nm:8s} TEST fail {r['fail']:.2e}  fail_hi_seen {r['fail_hi_seen']:.2e}  fail_hi (valid) {r['fail_hi']:.2e}"
                  f"  unseen {r['unseen']}  progress {r['progress']:.1f} m")
    save("fix", out)


def _ego_hits(R, k):
    return [e for e in R.events[k] if "ego" in (e["a"], e["b"])]


def mine_policy(doc, hz, ps, pol, chunk=400):
    """Run the hazard sweep under one policy and sort every ego collision into one class:

    avoidable    = tránh được bằng phanh hết cỡ ngay khi thấy: braking at the tyre limit from the first frame the
                   blamed hazard was both acting and visible would have avoided it. The "planner failure" counted in p.
                   (avoidable_foreknown counts the same from the hazard's start even if hidden: an upper bound.)
    unavoidable  = not even that full stop saves it.
    unattributed = no hazard had acted before contact: an ordinary crash of this town, not evidence about your hazard.
    low_speed (a flag, not a class): the ego was under 1 m/s at contact (hit while (nearly) stopped). Counted inside
    its class and also reported apart, because "it hit me while I waited" is a different story from "I hit it".
    """
    out = dict(n=len(ps), crashes=0, avoidable=0, unavoidable=0, unattributed=0, low_speed=0, avoidable_foreknown=0,
               overlap_at_start=0, by_hazard={}, examples=[])
    for c0 in range(0, len(ps), chunk):
        sub = ps[c0:c0 + chunk]
        R = core.rollout(sub, policy=[pol] * len(sub), hazards=hz)
        # a hazard placed inside another actor (a bike inside the parked bus) touches it at frame 0: not a real scene
        start = np.array([any(e["f"] == 0 and (e["a"].startswith("H") or e["b"].startswith("H")) for e in R.events[k]) for k in range(R.K)])
        out["overlap_at_start"] += int(start.sum())
        out["n"] -= int(start.sum())
        hit = np.array([bool(_ego_hits(R, k)) and not start[k] for k in range(R.K)])
        if not hit.any():
            continue
        ff = studio.fire_frames(R, sub)
        avoid, f0, avoid_fk = studio.oracle_avoids(sub, R, hz, policy=[pol] * len(sub))
        for k in np.nonzero(hit)[0]:
            e = _ego_hits(R, k)[0]
            v_ego = float(R.F[k, e["f"], core.EGO, 4])
            b = studio.blame(R, k, ff)
            key = "H%d" % b if b >= 0 else "unattributed"
            out["crashes"] += 1
            out["by_hazard"][key] = out["by_hazard"].get(key, 0) + 1
            kind = "unattributed" if b < 0 else ("avoidable" if avoid[k] else "unavoidable")
            out[kind] += 1
            out["low_speed"] += v_ego < 1.0
            out["avoidable_foreknown"] += bool(b >= 0 and avoid_fk[k])
            if len(out["examples"]) < 12:
                out["examples"].append(dict(params=sub[k], kind=kind, low_speed=v_ego < 1.0, other=e["b"] if e["a"] == "ego" else e["a"],
                                            t=e["t"], v_ego=v_ego,
                                            fire_t=float(ff[k, b] * core.DT) if b >= 0 and ff[k, b] >= 0 else None,   # hazard started
                                            brake_t=float(f0[k] * core.DT) if f0[k] >= 0 else None))                # oracle brakes (first visible)
    lo, hi = estimate.wilson_ci(out["avoidable"], out["n"])
    out.update(p=out["avoidable"] / max(1, out["n"]), lo=lo, hi=hi)
    return out


def step_mine(a, nat, seeds):
    """G. Your own hazard (saved from the viewer's chapter 10): replay the exact scene, then sweep it over its ranges
    in your town's traffic and count the collisions a planner should have avoided."""
    path = a.hazard if os.path.isabs(a.hazard) else os.path.join(HERE, a.hazard)
    if not os.path.exists(path):
        sys.exit(f"{a.hazard} not found: save it from the viewer's chapter 10 into this folder (README step G).")
    try:
        doc = studio.load(path)
    except ValueError as e:
        sys.exit(f"hazard file: {e}")
    hz = doc["hazards"]
    pol_scene = dict(doc.get("policy") or {})
    if not studio.hazard_ok(hz, doc["params"]):
        print("  WARNING: the scene itself breaks a hazard limit (studio.LIMITS, brake > mu*g, or a bike swerve sharper than "
              f"{studio.BIKE_LAT_MAX} m/s^2): no real road produces it, so its crash is not evidence. Soften it in chapter 10.")
    R = core.rollout(doc["params"], policy=pol_scene, hazards=hz)
    exact = dict(planner=[(e["a"], e["b"], e["t"]) for e in R.events[0]])
    if doc.get("ego") == "script" and doc.get("egoAcc"):
        Rs = core.rollout(doc["params"], policy=pol_scene, ego="script", ego_acc=np.array(doc["egoAcc"], float), hazards=hz)
        exact["you"] = [(e["a"], e["b"], e["t"]) for e in Rs.events[0]]
    print(f"  the scene from the viewer: planner {exact['planner'] or 'no collision'}" + (f"; you {exact['you'] or 'no collision'}" if "you" in exact else ""))
    ex = doc.get("exposure")
    if ex is not None and not (isinstance(ex, (int, float)) and 0 < ex <= 1):
        sys.exit(f"hazard.json exposure = {ex!r}: it is how often this situation happens per trip, a number in (0, 1].")
    bad = studio.check_space(doc)
    if bad:
        sys.exit("hazard.json space:\n  " + "\n  ".join(bad))
    sp = doc.get("space") or studio.default_space(doc)
    rng = np.random.default_rng(seeds["test"] + 404)
    if doc.get("world", "town") == "scene":
        world = [dict(doc["params"]) for _ in range(a.mine_n)]
    else:
        Z, Bn = nat.sample(a.mine_n, rng)
        world = space.to_params(Z, Bn, nat)
    ps = [dict(w, **v) for w, v in zip(world, studio.sample_hazard_values(sp, a.mine_n, rng))]
    ok = [feasible.is_feasible(p) and studio.hazard_ok(hz, p) for p in ps]
    ps = [p for p, o in zip(ps, ok) if o]
    where = "your one scene's traffic (world: scene; NOT your town's mix)" if doc.get("world") == "scene" else "your town's traffic"
    print(f"  {len(ps)} feasible of {a.mine_n} draws ({len(ps) / a.mine_n:.0%} accepted) over {len(sp)} hazard ranges, {where}")
    fx = load("fix") if os.path.exists(os.path.join(EVID, "fix.json")) else {}
    chosen = (fx.get("chosen") or {}).get("policy")
    out = dict(file=os.path.basename(path), hazards=hz, space=sp, world=doc.get("world", "town"), exposure=doc.get("exposure"),
               why=doc.get("why", ""), exact=exact, infeasible=a.mine_n - len(ps), accepted=len(ps) / a.mine_n,
               conditional_on="your scene's traffic only" if doc.get("world") == "scene" else "your town's traffic")
    for name, pol in [("baseline", {})] + ([("chosen", chosen)] if chosen else []):
        r = mine_policy(doc, hz, ps, pol)
        out[name] = r
        print(f"  {name:8s}: {r['crashes']} ego collisions = {r['avoidable']} avoidable by a full brake + {r['unavoidable']} unavoidable + "
              f"{r['unattributed']} unattributed ({r['low_speed']} at < 1 m/s);  P(planner failure | hazard) = {r['p']:.2e} [{r['lo']:.2e}, {r['hi']:.2e}]")
        if doc.get("exposure") and doc.get("world") != "scene":     # one scene's rate is not a per-trip rate
            r["per_trip"] = doc["exposure"] * r["p"]
            r["per_trip_hi"] = doc["exposure"] * r["hi"]
            print(f"            x exposure {doc['exposure']:g} per trip = {r['per_trip']:.2e} per trip (upper {r['per_trip_hi']:.2e})")
    save("mine", out)


def step_export(a, nat, seeds):
    """F. evidence/viewer.json: up to 4 crashes per mode from your hunts, your search cloud, and your chosen policy."""
    H = load("hunt")
    cloud = json.load(open(os.path.join(EVID, "cloud.json")))
    fx = load("fix") if os.path.exists(os.path.join(EVID, "fix.json")) else {}
    chosen = (fx.get("chosen") or {}).get("policy") or {}
    runs = []
    for m in MODES:
        got = [c for c in cloud if c["out"] == "crash" and c["mode"] == m][-4:]
        for j, c in enumerate(got):
            runs.append(dict(id=f"{m[:1].upper()}-{j + 1:02d}", label=m, outcome="crash", params=c["params"],
                             policy=dict(core.POLICY)))      # found under the baseline: replay it so; the fix is the ghost
    if os.path.exists(os.path.join(EVID, "replay.json")):
        for j, r in enumerate(load("replay").get("replay_only", [])[:3]):
            if isinstance(r, dict):
                runs.append(dict(id=f"R-{j + 1:02d}", label="replay", outcome="crash", others="replay", params=r["params"],
                                 policy=dict(core.POLICY)))
    if os.path.exists(os.path.join(EVID, "mine.json")):
        mn = load("mine")
        for name in ("baseline", "chosen"):
            for j, ex in enumerate([x for x in (mn.get(name) or {}).get("examples", []) if x["kind"] == "avoidable"][:3]):
                runs.append(dict(id=f"G{name[0].upper()}-{j + 1:02d}", label="mine", outcome="crash", params=ex["params"], hazards=mn["hazards"],
                                 policy=dict(core.POLICY) if name == "baseline" else None))   # GB: found under the baseline planner
    if len(cloud) > 6000:
        step = len(cloud) / 6000
        cloud = [cloud[int(i * step)] for i in range(6000)]
    viewer.export(os.path.join(EVID, "viewer.json"), runs, cloud=cloud, policy=chosen, baseline_policy={},
                  title=f"MSSV {mssv()}")
    print(f"  {len(runs)} runs, {len(cloud)} cloud points -> evidence/viewer.json (open the viewer, chapter 9)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["world", "replay", "mc", "hunt", "estimate", "fix", "export", "mine", "all"])
    ap.add_argument("hazard", nargs="?", default="hazard.json", help="mine: the file saved from the viewer's chapter 10")
    ap.add_argument("--quick", action="store_true", help="smaller budgets for a first try (not for submission)")
    a = ap.parse_args()
    big = not a.quick
    a.replay_n = 400 if big else 128
    a.mc_n = 3000 if big else 600
    a.iters = 8 if big else 3
    a.per_iter = 300 if big else 120
    a.final_n = 600 if big else 200
    a.seeds = 3
    a.fix_n = 400 if big else 150
    a.progress_n = 256 if big else 64
    a.mine_n = 1500 if big else 300
    nat, seeds = space.student_variant(mssv())
    # The grader re-runs TEST with a private salt, so tuning candidates on the public TEST numbers does not pay.
    seeds["test"] += int(os.environ.get("LAB11_TEST_SALT", "0"))
    RUN.update(mssv=mssv(), town=getattr(nat, "name", "?"), quick=a.quick, step=a.step,
               salted=bool(os.environ.get("LAB11_TEST_SALT")),
               budgets={k: getattr(a, k) for k in ("replay_n", "mc_n", "iters", "per_iter", "final_n", "seeds", "fix_n", "progress_n", "mine_n")})
    hz_file = a.hazard if os.path.isabs(a.hazard) else os.path.join(HERE, a.hazard)
    steps = (["world", "replay", "mc", "hunt", "estimate", "fix"] + (["mine"] if os.path.exists(hz_file) else []) + ["export"]
             if a.step == "all" else [a.step])
    # meta.json keeps a history: one line per step ever run here, so a later single step cannot hide how the rest ran
    old = load("meta") if os.path.exists(os.path.join(EVID, "meta.json")) else {}
    hist = old.get("steps", {}) if old.get("seeds") == seeds else {}
    hist.update({st: dict(quick=a.quick, at=time.strftime("%Y-%m-%d %H:%M")) for st in steps})
    save("meta", dict(seeds=seeds, steps=hist))
    if a.quick:
        print("  --quick: tiny budgets, numbers too thin to mean anything (0 crashes, unseen modes, top_share 1). "
              "Fine for debugging; NOT for evidence/ you submit (RUBRIC: part D = 0).")
    if a.step == "all" and "mine" not in steps:
        print(f"  (no {a.hazard} yet: step G skipped; run `python run_all.py mine {a.hazard}` once you have it)")
    for st in steps:
        t = time.time()
        print(f"[{st}]")
        fn = globals()["step_" + st]
        try:
            fn(a) if st == "world" else fn(a, nat, seeds)
        except NotImplementedError as e:      # a TODO this step needs is not done yet: say which, not a traceback
            sys.exit(f"  step `{st}` needs {e}. Finish that TODO (python -m pytest -q tests/test_todos.py), then run it again.")
        print(f"  ({time.time() - t:.0f} s)")


if __name__ == "__main__":
    main()
