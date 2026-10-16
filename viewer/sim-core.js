/* sim-core.js — deterministic driving scenario for "Vòng lặp mô phỏng".
   Pure JS, no DOM: runs in the browser (window.SimCore) and in node (module.exports) for tuning.
   Units: metres, seconds. World frame: x east, z south, y up (three.js). Right-hand traffic.
   Headings are unit vectors [hx, hz]; the right-hand normal of [hx, hz] is [-hz, hx]. */
(function (root) {
  "use strict";
  var DT = 0.05, T_END = 24, NF = Math.round(T_END / DT) + 1;

  // ------------------------------------------------------------------ paths (dense polylines)
  function seg(x0, z0, x1, z1, step) {
    var n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, z1 - z0) / (step || 1))), p = [];
    for (var i = 0; i <= n; i++) p.push([x0 + (x1 - x0) * i / n, z0 + (z1 - z0) * i / n]);
    return p;
  }
  function arc(cx, cz, r, a0, a1, step) {
    var n = Math.max(2, Math.ceil(Math.abs(a1 - a0) * r / (step || 0.5))), p = [];
    for (var i = 0; i <= n; i++) { var a = a0 + (a1 - a0) * i / n; p.push([cx + r * Math.cos(a), cz + r * Math.sin(a)]); }
    return p;
  }
  function path() {
    var pts = [];
    for (var k = 0; k < arguments.length; k++) {
      var q = arguments[k];
      for (var i = 0; i < q.length; i++) {
        var l = pts[pts.length - 1];
        if (!l || Math.hypot(q[i][0] - l[0], q[i][1] - l[1]) > 1e-6) pts.push(q[i]);
      }
    }
    var s = [0];
    for (var j = 1; j < pts.length; j++) s.push(s[j - 1] + Math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1]));
    return { pts: pts, s: s, len: s[s.length - 1] };
  }
  function at(P, s, d) {             // point at arc length s with lateral offset d (right positive)
    s = Math.max(0, Math.min(P.len - 1e-6, s));
    var lo = 0, hi = P.s.length - 1;
    while (hi - lo > 1) { var m = (lo + hi) >> 1; if (P.s[m] <= s) lo = m; else hi = m; }
    var a = P.pts[lo], b = P.pts[hi], L = P.s[hi] - P.s[lo] || 1, u = (s - P.s[lo]) / L;
    var hx = (b[0] - a[0]) / L, hz = (b[1] - a[1]) / L;
    return { x: a[0] + (b[0] - a[0]) * u - hz * (d || 0), z: a[1] + (b[1] - a[1]) * u + hx * (d || 0), hx: hx, hz: hz };
  }

  // ------------------------------------------------------------------ the map
  var ROAD = { xMin: -220, xMax: 330, half: 7, crossX: 80, crossHalf: 7, walkX: 122 };
  var LANES = {
    E1: path(seg(-220, 1.75, 330, 1.75)),       // eastbound inner (ego lane)
    E2: path(seg(-220, 5.25, 330, 5.25)),       // eastbound outer
    W1: path(seg(330, -1.75, -220, -1.75)),
    W2: path(seg(330, -5.25, -220, -5.25)),
    // right turn from the cross road (northbound, x = 81.75) into E2
    XR: path(seg(81.75, 70, 81.75, 10), arc(86.5, 10, 4.75, Math.PI, 1.5 * Math.PI), seg(86.5, 5.25, 330, 5.25)),
    NB: path(seg(81.75, 120, 81.75, -120)),
    SB: path(seg(78.25, -120, 78.25, 120)),
    WALK: path(seg(ROAD.walkX, 9.2, ROAD.walkX, -9.2, 0.25)),            // crosswalk, south -> north
    SWS: path(seg(-80, 8.6, 300, 8.6)), SWN: path(seg(300, -8.6, -80, -8.6))  // sidewalks
  };

  // ------------------------------------------------------------------ actors
  var CAR = { len: 4.6, wid: 1.9 }, BUS = { len: 12, wid: 2.55 }, PED = { len: 0.6, wid: 0.6 };
  var PED_LOOK = 0.5;   // a walking pedestrian stops when a vehicle body is within this many metres ahead of its centre
  function idm(o) { return Object.assign({ v0: 13, T: 1.4, a: 1.6, b: 2.2, s0: 2.0 }, o || {}); }

  /* Scenario parameters a chapter can change (all have defaults). */
  var DEFAULTS = {
    cutT0: 5.0,        // cut-in start time [s]
    cutDur: 2.4,       // cut-in duration [s]
    cutV: 15,          // cut-in car speed [m/s]
    pedT0: 9.6,        // pedestrian starts crossing [s]
    pedV: 1.45,        // pedestrian speed [m/s]
    busGo: 17.5,       // bus pulls out [s]
    cutDs: -4,         // cut-in car starts this far ahead of the ego along the road [m] (negative = behind)
    tgT: 0.9,          // tailgater time headway [s]
    mu: 0.8,           // tyre-road friction: 0.8 dry, about 0.5 wet; caps every vehicle's braking at mu*g
    // behaviour gates (null = off, as in the viewer's scripted story)
    cutGap: null,      // the cut-in driver waits for this rear gap [m] to the car it cuts in front of
    cutBrake: 0,       // after merging, the cut-in car brakes at this rate [m/s^2] for 2.5 s (0 = no)
    pedGap: null,      // the crossing pedestrian waits until every car it can SEE is at least this many seconds away
    pedLook: 0,        // 1 = the pedestrian steps to the bus's front edge to look; 0 = looks from the kerb (the bus hides traffic)
    civil: false       // true = everyone else drives like people: yield at the crosswalk, pause a blocked merge, give way when turning
  };
  /* The ego planner's knobs (the "policy"). Defaults reproduce the planner used by the viewer. */
  var POLICY = {
    v0: 12.5, T: 1.6, a: 1.4, b: 2.0, s0: 2.5,   // IDM car following
    cautionV: 7.5,     // speed limit near a crosswalk the ego cannot see [m/s]
    cautionRange: 55,  // start of that caution zone, metres before the crosswalk
    pedBuffer: 2.5,    // extra seconds of margin when predicting whether the pedestrian will be in the lane
    cutinLat: 2.15     // a same-direction car closer than this sideways [m] counts as the car ahead (2.15 = only once it overlaps the lane)
  };
  var G = 9.81;
  var BIKE = { len: 1.8, wid: 0.7 };

  /* Hazard cards: extra trouble a student adds to the world (chapter 10 "xưởng kịch bản", lab step G).
     Every hazard fires at the first t >= t0 at which the ego is within `trig` metres (along x) behind the hazard
     (its crossing x, or the current x of the bike or the braking car; trig null = fire at t0). Types:
       ped     { x, side: "S"|"N", v, t0, trig }           a pedestrian crosses the road at x (not at the crosswalk)
       stalled { x, lane: "E1"|"E2" }                      a broken-down car standing in a lane from the start
       brake   { who: actor id, decel, dur, t0, trig }     that car brakes hard for dur seconds (a brake-check)
       bike    { x, lane, d, v, swerve, swDur, t0, trig }  a cyclist near the kerb swerves left by `swerve` metres */
  var HAZARD_DEFAULTS = {
    ped: { x: 200, side: "S", v: 1.6, t0: 0, trig: 30 },
    stalled: { x: 260, lane: "E2" },
    brake: { who: "L1", decel: 6, dur: 2.5, t0: 0, trig: null },
    bike: { x: 200, lane: "E2", d: 1.2, v: 5, swerve: 1.6, swDur: 2.0, t0: 0, trig: 25 }
  };
  var HAZARD_STRUCT_FREE = { x: 1, v: 1, t0: 1, trig: 1, d: 1, swerve: 1, swDur: 1, decel: 1, dur: 1 };   // fields a scenario may override
  function hazardFull(h) { return Object.assign({}, HAZARD_DEFAULTS[h.type] || {}, h); }
  function hazardLane(h) { return path(seg(h.x, h.side === "N" ? -9.2 : 9.2, h.x, h.side === "N" ? 9.2 : -9.2, 0.25)); }

  function actors(p, hz) {
    var A = [];
    function add(o) { o.idx = A.length; A.push(o); return o; }
    add({ id: "ego", name: "xe ta", kind: "ego", dims: CAR, lane: "E1", s: 180, v: 12, d: 0, ctl: "ego", color: "ego" });
    add({ id: "L1", name: "xe dẫn", kind: "car", dims: CAR, lane: "E1", s: 214, v: 12.4, d: 0, ctl: "idm", p: idm({ v0: 12.6 }), color: "c1" });
    add({ id: "C2", name: "xe cắt làn", kind: "car", dims: CAR, lane: "E2", s: 180 + p.cutDs, v: p.cutV, d: 0, ctl: "cutin", p: idm({ v0: p.cutV, T: 0.9, b: 3 }),
          cut: { t0: p.cutT0, dur: p.cutDur, d1: -3.5, gap: p.cutGap, brake: p.cutBrake, start: null }, color: "adv" });
    add({ id: "TG", name: "xe bám đuôi", kind: "car", dims: CAR, lane: "E1", s: 163, v: 12.2, d: 0, ctl: "idm", p: idm({ v0: 13.5, T: p.tgT, a: 1.8, b: 3.0, s0: 1.5 }), color: "c3", dark: true });
    add({ id: "E5", name: "xe", kind: "car", dims: CAR, lane: "E2", s: 120, v: 13, d: 0, ctl: "idm", p: idm({ v0: 13 }), color: "c2" });
    add({ id: "BUS", name: "xe buýt", kind: "bus", dims: BUS, lane: "E2", s: 326, v: 0, d: 0, ctl: "hold", until: p.busGo, p: idm({ v0: 9, a: 1.0 }), color: "bus" });
    add({ id: "X1", name: "xe rẽ phải", kind: "car", dims: CAR, lane: "XR", s: 58, v: 0, d: 0, ctl: "hold", until: 18.6, p: idm({ v0: 9, a: 1.4 }), color: "c4" });
    add({ id: "W1a", name: "xe", kind: "car", dims: CAR, lane: "W1", s: 150, v: 12, d: 0, ctl: "idm", p: idm({ v0: 12.5 }), color: "c2" });
    add({ id: "W1b", name: "xe", kind: "car", dims: CAR, lane: "W1", s: 120, v: 12, d: 0, ctl: "idm", p: idm({ v0: 12.5 }), color: "c1", dark: true });
    add({ id: "W2a", name: "xe", kind: "car", dims: CAR, lane: "W2", s: 190, v: 11, d: 0, ctl: "idm", p: idm({ v0: 11.5 }), color: "c4" });
    add({ id: "W2b", name: "xe", kind: "car", dims: CAR, lane: "W2", s: 60, v: 13, d: 0, ctl: "idm", p: idm({ v0: 13 }), color: "c3" });
    add({ id: "W1c", name: "xe", kind: "car", dims: CAR, lane: "W1", s: 268, v: 12, d: 0, ctl: "idm", p: idm({ v0: 12.5 }), color: "c2" });
    add({ id: "P1", name: "người đi bộ", kind: "ped", dims: PED, lane: "WALK", s: 0, v: 0, d: 0, ctl: "walk", t0: p.pedT0, vw: p.pedV, gap: p.pedGap, color: "ped" });
    add({ id: "P2", name: "người đi bộ", kind: "ped", dims: PED, lane: "SWS", s: 120, v: 1.3, d: 0, ctl: "walk", t0: 0, vw: 1.3, color: "ped" });
    add({ id: "P3", name: "người đi bộ", kind: "ped", dims: PED, lane: "SWN", s: 90, v: 1.2, d: 0, ctl: "walk", t0: 0, vw: 1.2, color: "ped" });
    add({ id: "P4", name: "người đi bộ", kind: "ped", dims: PED, lane: "SWS", s: 260, v: 1.1, d: 0.4, ctl: "walk", t0: 0, vw: 1.1, color: "ped" });
    (hz || []).forEach(function (h0, k) {
      var h = hazardFull(h0), id = "H" + k;
      Object.keys(HAZARD_STRUCT_FREE).forEach(function (f) { var o = p["h" + k + "_" + f]; if (o !== undefined) h[f] = o; });   // per-scenario values (lab hunts)
      var trig = { x: h.x, r: h.trig == null ? null : h.trig };
      if (h.type === "ped") add({ id: id, name: "người băng ngang", kind: "ped", dims: PED, lane: "H" + k, path: hazardLane(h), s: 0, v: 0, d: 0, ctl: "walk", t0: h.t0, vw: h.v, trig: trig, hz: k, color: "ped" });
      else if (h.type === "stalled") add({ id: id, name: "xe chết máy", kind: "car", dims: CAR, lane: h.lane, s: h.x + 220, v: 0, d: 0, ctl: "hold", until: Infinity, p: idm(), hz: k, color: "c4", dark: true });
      else if (h.type === "bike") add({ id: id, name: "xe đạp", kind: "bike", dims: BIKE, lane: h.lane, s: h.x + 220, v: h.v, d: h.d, ctl: "bike", p: idm({ v0: h.v, T: 1.0, a: 1.0, b: 2.5, s0: 1.5 }),
                                    sw: { t0: h.t0, trig: trig, dd: -h.swerve, dur: h.swDur, d0: h.d, start: null }, hz: k, color: "c1" });
      else if (h.type === "brake") { var tg = A.filter(function (a) { return a.id === h.who; })[0];
        if (tg) (tg.brks = tg.brks || []).push({ t0: h.t0, trig: { x: null, r: h.trig == null ? null : h.trig }, decel: h.decel, dur: h.dur, start: null, hz: k }); }   // a list: two cards on one car both apply
    });
    return A;
  }
  // parked cars along the cross road and the main-road shoulder: static boxes (also LiDAR targets)
  var PARKED = [
    { x: 89.6, z: 22, hx: 0, hz: 1, dark: false, color: "c2" }, { x: 89.6, z: 30, hx: 0, hz: 1, dark: true, color: "c1" },
    { x: 70.4, z: -24, hx: 0, hz: -1, dark: false, color: "c4" }, { x: 70.4, z: 16, hx: 0, hz: -1, dark: false, color: "c3" },
    { x: 89.6, z: -18, hx: 0, hz: 1, dark: false, color: "c3" }
  ];

  // ------------------------------------------------------------------ geometry helpers
  function obb(st, dims) { return { x: st.x, z: st.z, hx: st.hx, hz: st.hz, l: dims.len / 2, w: dims.wid / 2 }; }
  function overlap(a, b) {            // 2-D separating-axis test for two oriented boxes
    var ax = [[a.hx, a.hz], [-a.hz, a.hx], [b.hx, b.hz], [-b.hz, b.hx]], dx = b.x - a.x, dz = b.z - a.z;
    for (var i = 0; i < 4; i++) {
      var n = ax[i], ra = a.l * Math.abs(n[0] * a.hx + n[1] * a.hz) + a.w * Math.abs(-n[0] * a.hz + n[1] * a.hx);
      var rb = b.l * Math.abs(n[0] * b.hx + n[1] * b.hz) + b.w * Math.abs(-n[0] * b.hz + n[1] * b.hx);
      if (Math.abs(n[0] * dx + n[1] * dz) > ra + rb) return false;
    }
    return true;
  }
  function segHitsBox(x0, z0, x1, z1, b) {  // does segment p0->p1 cross the oriented box b?
    var rx = x0 - b.x, rz = z0 - b.z, qx = x1 - b.x, qz = z1 - b.z;
    var p = [rx * b.hx + rz * b.hz, -rx * b.hz + rz * b.hx], q = [qx * b.hx + qz * b.hz, -qx * b.hz + qz * b.hx];
    var t0 = 0, t1 = 1, d = [q[0] - p[0], q[1] - p[1]], e = [b.l, b.w];
    for (var k = 0; k < 2; k++) {
      if (Math.abs(d[k]) < 1e-9) { if (Math.abs(p[k]) > e[k]) return false; continue; }
      var u = (-e[k] - p[k]) / d[k], v = (e[k] - p[k]) / d[k]; if (u > v) { var w = u; u = v; v = w; }
      t0 = Math.max(t0, u); t1 = Math.min(t1, v); if (t0 > t1) return false;
    }
    return true;
  }

  // ------------------------------------------------------------------ one rollout
  /* opts: { ego: "planner" | "human", others: "reactive" | "replay", log: rollout used for replay, params } */
  function rollout(opts) {
    opts = opts || {};
    var p = Object.assign({}, DEFAULTS, opts.params || {}), A = actors(p, opts.hazards), N = A.length;
    var pol = Object.assign({}, POLICY, opts.policy || {}), aMin = -Math.min(9, p.mu * G);
    var F = new Float32Array(NF * N * 7);    // per frame, per actor: x, z, hx, hz, v, a, d
    var EGO = new Float32Array(NF * 4);      // ego: plan state (0 cruise,1 follow,2 caution,3 stop-for-ped), v target, ped-visible, leader idx
    var events = [], hit = {}, egoI = A.findIndex(function (a) { return a.id === "ego"; });
    var log = opts.others === "replay" ? opts.log : null;
    var replayIdx = {};
    if (log) log.actors.forEach(function (a, i) { replayIdx[a.id] = i; });
    var walk = LANES.WALK;

    function state(a) { return at(a.path || LANES[a.lane], a.s, a.d); }
    // a hazard fires once t >= t0 and the ego is within trig metres of its x (trig null = at t0)
    function fired(tr, t0, t, sts, xh) { return t >= t0 && (tr.r == null || (xh == null ? tr.x : xh) - sts[0].x <= tr.r); }
    function leaderFor(a, st, all, fi, sts) {   // nearest obstacle ahead in a's corridor: [gap, v, idx]
      var best = [1e9, 0, -1], halfL = a.dims.len / 2;
      for (var j = 0; j < all.length; j++) {
        var b = all[j]; if (b === a) continue;
        var sb = sts[j], rx = sb.x - st.x, rz = sb.z - st.z;
        var lon = rx * st.hx + rz * st.hz, lat = -rx * st.hz + rz * st.hx;
        if (lon <= 0 || lon > 120) continue;
        if (b.kind === "ped") {
          if (p.civil && b.lane === "WALK" && b.v > 0 && Math.abs(st.hx) > 0.7 && sb.z > st.z - 1.9 && sb.z < 9.3) {
            // someone is on the crosswalk and has not cleared my lane yet: stop before the crosswalk if I can see them
            var lonCW = (ROAD.walkX - st.x) * st.hx, gc = lonCW - halfL - 1.5;
            var seen = a.ctl === "ego" ? visible(st, sb, sts, a.idx, j) : busClear(st, sb, sts, a.idx);
            if (lonCW > 0 && seen && gc < best[0]) best = [gc, 0, j];
            continue;
          }
          if (Math.abs(lat) > 1.9) continue;
          var g = lon - halfL - 1.2;
          if (g < best[0]) best = [g, 0, j];
        } else {
          if (sb.hx * st.hx + sb.hz * st.hz < 0.3) continue;       // opposite or crossing traffic
          if (Math.abs(lat) > (a.ctl === "ego" && opts.ego !== "human" ? pol.cutinLat : (a.dims.wid + b.dims.wid) / 2 + 0.25)) continue;
          var gv = lon - halfL - b.dims.len / 2;
          if (gv < best[0]) best = [gv, b.v, j];
        }
      }
      return best;
    }
    function idmAcc(P, v, lead) {
      var ss = P.s0 + Math.max(0, v * P.T + v * (v - lead[1]) / (2 * Math.sqrt(P.a * P.b)));
      var free = 1 - Math.pow(v / P.v0, 4);
      var inter = lead[2] < 0 ? 0 : Math.pow(ss / Math.max(lead[0], 0.2), 2);
      return Math.max(aMin, Math.min(P.a, P.a * (free - inter)));
    }
    function visible(st, tgt, sts, self, tIdx) {  // line of sight from a roof sensor to tgt, blocked by any vehicle box
      for (var j = 0; j < A.length; j++) {
        if (j === self || j === tIdx || A[j].kind === "ped") continue;
        if (segHitsBox(st.x, st.z, tgt.x, tgt.z, obb(sts[j], A[j].dims))) return false;
      }
      return true;
    }

    // gap acceptance: the cut-in driver checks the car behind (and ahead) in the target lane
    function cutGateOK(a, i, st, sts) {
      for (var j = 0; j < N; j++) {
        var b = A[j]; if (j === i || b.kind === "ped") continue;
        var sb = sts[j]; if (sb.hx * st.hx + sb.hz * st.hz < 0.7) continue;
        var rx = sb.x - st.x, rz = sb.z - st.z, lon = rx * st.hx + rz * st.hz, lat = -rx * st.hz + rz * st.hx;
        if (Math.abs(lat - a.cut.d1) > 1.6) continue;                      // only the target lane
        var half = (a.dims.len + b.dims.len) / 2;
        if (lon <= 0 && -lon - half < a.cut.gap + Math.max(0, b.v - a.v) * 1.0) return false;
        if (lon > 0 && lon - half < 2) return false;
      }
      return true;
    }
    // people look past cars; only the bus is tall enough to hide traffic
    function busClear(p0, p1, sts, self) {
      for (var k = 0; k < N; k++) { if (A[k].kind !== "bus" || k === self) continue; if (segHitsBox(p0.x, p0.z, p1.x, p1.z, obb(sts[k], A[k].dims))) return false; }
      return true;
    }
    function pedSees(ps, sb, sts, j) {
      var eye = p.pedLook && ps.z > 3.7 ? { x: ps.x, z: 3.7 } : ps;     // a careful pedestrian peeks past the bus front
      return busClear(eye, sb, sts, j);
    }
    function crosswalkBusy(st, a) {
      var lonCW = (ROAD.walkX - st.x) * st.hx;
      if (lonCW < -a.dims.len / 2 || lonCW > 30) return false;
      for (var k = 0; k < N; k++) { var b = A[k]; if (b.lane === "WALK" && b.v > 0) { var sb = at(LANES.WALK, b.s, 0); if (sb.z > st.z - 2.2 && sb.z < 9.3) return true; } }
      return false;
    }
    function blindCrosswalk(st, a, sts) {
      if (Math.abs(st.hx) < 0.7) return false;
      var lonCW = (ROAD.walkX - st.x) * st.hx;
      if (lonCW < 0 || lonCW > 40) return false;
      var kerb = { x: ROAD.walkX, z: st.hx > 0 ? 8.5 : -8.5 };
      return !busClear(st, kerb, sts, a.idx);
    }
    // a merge in progress pauses while the target lane is blocked right beside it
    function mergeClear(a, i, st, sts) {
      for (var j = 0; j < N; j++) {
        var b = A[j]; if (j === i || b.kind === "ped") continue;
        var sb = sts[j]; if (sb.hx * st.hx + sb.hz * st.hz < 0.7) continue;
        var rx = sb.x - st.x, rz = sb.z - st.z, lon = rx * st.hx + rz * st.hz, lat = -rx * st.hz + rz * st.hx;
        if (Math.abs(lat - a.cut.d1) > 1.6) continue;
        var half = (a.dims.len + b.dims.len) / 2;
        if (lon > 0 && lon - half < 1.0) return false;
        if (lon <= 0 && -lon - half < 0.5) return false;
      }
      return true;
    }
    // the pedestrian crosses only if no car it can see will reach the crosswalk within its accepted gap
    function pedGateOK(a, i, sts) {
      var ps = sts[i];
      for (var j = 0; j < N; j++) {
        var b = A[j]; if (b.kind === "ped" || j === i) continue;
        var sb = sts[j]; if (Math.abs(sb.hx) < 0.7 || Math.abs(sb.z) > 7) continue;   // main-road traffic only
        var dx = ROAD.walkX - sb.x; if (dx * sb.hx <= 0 || Math.abs(dx) > 90) continue; // must be approaching
        var tta = (Math.abs(dx) - b.dims.len / 2) / Math.max(b.v, 0.5);
        if (tta < a.gap && pedSees(ps, sb, sts, j)) return false;
      }
      return true;
    }

    for (var f = 0; f < NF; f++) {
      var t = f * DT, sts = A.map(state), acc = new Array(N).fill(0), blkP1 = false;
      // replayed actors take their logged pose
      if (log) A.forEach(function (a, i) {
        if (a.id === "ego" || replayIdx[a.id] == null) return;
        var k = (f * log.actors.length + replayIdx[a.id]) * 7, L = log.F;
        sts[i] = { x: L[k], z: L[k + 1], hx: L[k + 2], hz: L[k + 3] }; a.v = L[k + 4];
      });
      for (var i = 0; i < N; i++) {
        var a = A[i], st = sts[i];
        if (log && a.id !== "ego" && replayIdx[a.id] != null) { acc[i] = log.F[(f * log.actors.length + replayIdx[a.id]) * 7 + 5]; continue; }
        if (a.ctl === "walk") {
          if (a.started == null && t >= a.t0 && (a.gap == null || pedGateOK(a, i, sts)) && (!a.trig || fired(a.trig, a.t0, t, sts))) a.started = t;
          a.v = a.started != null ? a.vw : 0;
          if (a.v > 0) {      // nobody walks into the side of a car: stop while a vehicle body fills the step ahead
            var probe = obb({ x: st.x + st.hx * PED_LOOK, z: st.z + st.hz * PED_LOOK, hx: st.hx, hz: st.hz }, a.dims);
            for (var jb = 0; jb < N; jb++) if (A[jb].kind !== "ped" && overlap(probe, obb(sts[jb], A[jb].dims))) { a.v = 0; break; }
            if (a.id === "P1" && overlap(probe, obb(sts[egoI], A[egoI].dims))) blkP1 = true;   // the ego's body stopped the pedestrian on the crosswalk
          }
          continue;
        }
        if (a.ctl === "hold" && t < a.until) { a.v = 0; continue; }
        if (a.ctl === "hold" && p.civil && a.kind === "bus" && !a.going) {         // the bus driver waits while someone is on the crosswalk ahead
          if (crosswalkBusy(st, a)) { a.v = 0; continue; }
          a.going = true;
        }
        if (a.ctl === "hold" && p.civil && a.lane === "XR" && !a.going) {
          var busy = false;
          for (var q = 0; q < N; q++) { var bq = A[q], sq = sts[q]; if (bq.kind === "ped" || q === i || sq.hx < 0.7 || sq.z < 3.4 || sq.z > 7.2) continue;
            var dq = 86 - sq.x; if (dq > -3 && dq < 70 && (dq - bq.dims.len / 2) / Math.max(bq.v, 0.5) < 3.5) busy = true; }
          if (busy) { a.v = 0; continue; }
          a.going = true;
        }
        var lead = leaderFor(a, st, A, f, sts);
        if (a.ctl === "ego") {
          var P = opts.ego === "human" ? idm({ v0: 12.5, T: 1.1, a: 1.6, b: 3.5, s0: 2 }) : idm({ v0: pol.v0, T: pol.T, a: pol.a, b: pol.b, s0: pol.s0 });
          var mode = lead[2] >= 0 && lead[0] < 45 ? 1 : 0, ped = A.filter(function (b) { return b.id === "P1"; })[0], ps = sts[ped.idx];
          var vis = visible(st, ps, sts, i, ped.idx) ? 1 : 0;
          var dxw = ROAD.walkX - st.x;
          if (opts.ego === "script") {
            // a person at the wheel (chapter 10): the recorded pedal input, clipped to what the tyres allow
            var ea = opts.egoAcc || [], fa = Math.min(f, ea.length - 1);
            acc[i] = fa < 0 ? 0 : Math.max(aMin, Math.min(3.0, +ea[fa] || 0));
            if (a.v <= 0 && acc[i] < 0) acc[i] = 0;
            EGO[f * 4] = 4; EGO[f * 4 + 1] = 0; EGO[f * 4 + 2] = vis; EGO[f * 4 + 3] = lead[2];
            continue;
          }
          if (opts.ego === "human") {
            // the logged human: reacts only to what is in the lane and visible
            if (!(lead[2] === ped.idx && vis)) { if (lead[2] === ped.idx) lead = [1e9, 0, -1]; }
          } else {
            // the planner: predicts the pedestrian and slows near an occluded crosswalk
            var willCross = (vis || !p.civil) && ped.v > 0.1 && ps.z > -2 && ps.z < 9.5;   // civil world: it can only predict a pedestrian its sensor sees
            if (willCross && dxw > 0) {
              var tEnter = Math.max(0, (ps.z - (st.z + 1.9)) / Math.max(ped.v, 0.3)), tArrive = (dxw - 3) / Math.max(a.v, 1);
              if (tEnter < tArrive + pol.pedBuffer) { var g = dxw - a.dims.len / 2 - 2.5; if (g < lead[0]) lead = [g, 0, ped.idx]; mode = 3; }
            }
            if (dxw > 0 && dxw < pol.cautionRange && !vis && mode !== 3) { P.v0 = Math.min(P.v0, pol.cautionV); mode = 2; }
          }
          acc[i] = idmAcc(P, a.v, lead);
          EGO[f * 4] = mode; EGO[f * 4 + 1] = P.v0; EGO[f * 4 + 2] = vis; EGO[f * 4 + 3] = lead[2];
        } else {
          var Pa = a.p;
          if (p.civil && blindCrosswalk(st, a, sts)) Pa = Object.assign({}, a.p, { v0: Math.min(a.p.v0, 6) });   // slow past a bus that hides the crosswalk
          acc[i] = idmAcc(Pa, a.v, lead);
          if (a.ctl === "cutin") {
            var c = a.cut;
            if (c.start == null && t >= c.t0 && (c.gap == null || cutGateOK(a, i, st, sts))) { c.start = t; c.u = 0; }
            var u = c.start == null ? 0 : p.civil ? c.u : (t - c.start) / c.dur;
            if (c.start != null && p.civil && c.u < 1 && mergeClear(a, i, st, sts)) c.u = Math.min(1, c.u + DT / c.dur);
            a.d = u <= 0 ? 0 : u >= 1 ? c.d1 : c.d1 * (1 - Math.cos(Math.PI * u)) / 2;
            if (u >= 1 && c.tDone == null) c.tDone = t;
            if (c.brake > 0 && c.tDone != null && t < c.tDone + 2.5) acc[i] = Math.max(aMin, Math.min(acc[i], -c.brake));
          }
          if (a.ctl === "bike") {
            var w = a.sw;
            if (w.start == null && fired(w.trig, w.t0, t, sts, st.x)) w.start = t;
            var uw = w.start == null ? 0 : Math.min(1, (t - w.start) / w.dur);
            a.d = w.d0 + w.dd * (1 - Math.cos(Math.PI * uw)) / 2;
          }
          (a.brks || []).forEach(function (bk) {
            if (bk.start == null && fired(bk.trig, bk.t0, t, sts, st.x)) bk.start = t;
            if (bk.start != null && t < bk.start + bk.dur) acc[i] = Math.max(aMin, Math.min(acc[i], -bk.decel));
          });
        }
      }
      // write the frame (pose at t, then integrate to t + dt)
      for (var i2 = 0; i2 < N; i2++) {
        var a2 = A[i2], s2 = sts[i2], k = (f * N + i2) * 7;
        F[k] = s2.x; F[k + 1] = s2.z; F[k + 2] = s2.hx; F[k + 3] = s2.hz; F[k + 4] = a2.v; F[k + 5] = acc[i2]; F[k + 6] = a2.d;
      }
      // collisions involving the ego or any two vehicles
      for (var u1 = 0; u1 < N; u1++) for (var u2 = u1 + 1; u2 < N; u2++) {
        var b1 = A[u1], b2 = A[u2];
        if (b1.kind === "ped" && b2.kind === "ped") continue;
        var key = b1.id + "|" + b2.id; if (hit[key]) continue;
        var touch = overlap(obb(sts[u1], b1.dims), obb(sts[u2], b2.dims));
        if (touch || (blkP1 && b1.id === "ego" && b2.id === "P1")) {      // a contact in the same frame wins over a block
          hit[key] = 1;
          events.push({ t: t, f: f, a: b1.id, b: b2.id, kind: touch ? "contact" : "block", x: (sts[u1].x + sts[u2].x) / 2, z: (sts[u1].z + sts[u2].z) / 2,
                        fake: !!(log && (replayIdx[b1.id] != null && b1.id !== "ego" || replayIdx[b2.id] != null && b2.id !== "ego")) });
        }
      }
      if (f < NF - 1) for (var i3 = 0; i3 < N; i3++) {
        var a3 = A[i3];
        if (log && a3.id !== "ego" && replayIdx[a3.id] != null) continue;
        a3.v = Math.max(0, a3.v + acc[i3] * DT); a3.s += a3.v * DT;
      }
    }
    return { F: F, EGO: EGO, events: events, actors: A.map(function (a) { return { id: a.id, name: a.name, kind: a.kind, dims: a.dims, color: a.color, dark: !!a.dark, lane: a.lane, hz: a.hz }; }), params: p, policy: pol, hazards: (opts.hazards || []).map(hazardFull), opts: opts };
  }

  /* minimum distance and time-to-collision between two actors over a rollout */
  function pairStats(R, idA, idB) {
    var N = R.actors.length, ia = -1, ib = -1;
    R.actors.forEach(function (a, i) { if (a.id === idA) ia = i; if (a.id === idB) ib = i; });
    var gapMin = 1e9, ttcMin = 1e9, dims = R.actors[ia].dims;
    for (var f = 0; f < NF; f++) {
      var ka = (f * N + ia) * 7, kb = (f * N + ib) * 7, F = R.F;
      var rx = F[kb] - F[ka], rz = F[kb + 1] - F[ka + 1], lon = rx * F[ka + 2] + rz * F[ka + 3], lat = -rx * F[ka + 3] + rz * F[ka + 2];
      if (Math.abs(lat) < 2.1 && lon > 0) {
        var gap = lon - dims.len; gapMin = Math.min(gapMin, gap);
        var dv = F[ka + 4] - F[kb + 4]; if (dv > 0.1) ttcMin = Math.min(ttcMin, Math.max(gap, 0) / dv);
      }
    }
    return { gapMin: gapMin, ttcMin: ttcMin };
  }

  var api = { DT: DT, T_END: T_END, NF: NF, LANES: LANES, ROAD: ROAD, PARKED: PARKED, CAR: CAR, BUS: BUS, DEFAULTS: DEFAULTS, POLICY: POLICY, HAZARD_DEFAULTS: HAZARD_DEFAULTS, hazardFull: hazardFull,
              at: at, rollout: rollout, pairStats: pairStats, overlap: overlap, obb: obb, segHitsBox: segHitsBox };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.SimCore = api;
})(this);
