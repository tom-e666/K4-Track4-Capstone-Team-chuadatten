/* core.js — car flip (airborne rollover) simulation with cold-gas attitude thrusters.
   Pure JS, no DOM: runs in the browser (window.FlipCore) and in node (module.exports) for batch evaluation.
   Units SI. World frame: x along the road (driving direction), y up, z to the right of the road.
   Body frame: x forward, y up, z right. Quaternions are [w, x, y, z] and map body -> world.
   Deterministic: same options + same seed -> identical run. */
(function (root) {
  "use strict";
  var G = 9.81, DT = 0.001, SUB = 10, FDT = DT * SUB, T_END = 6, NF = Math.round(T_END / FDT) + 1, NSTEP = (NF - 1) * SUB;
  var D2R = Math.PI / 180, R2D = 180 / Math.PI;

  // ------------------------------------------------------------------ small vector / quaternion helpers
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function scl(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function norm(a) { return Math.sqrt(dot(a, a)); }
  function nrm(a) { var n = norm(a); return n > 1e-12 ? scl(a, 1 / n) : [0, 0, 0]; }
  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
  function qmul(a, b) {
    return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
  }
  function qrot(q, v) {               // body vector -> world
    var w = q[0], x = q[1], y = q[2], z = q[3];
    var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
  }
  function qinv(q, v) { return qrot([q[0], -q[1], -q[2], -q[3]], v); }   // world vector -> body
  function qnorm(q) { var n = Math.hypot(q[0], q[1], q[2], q[3]); return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]; }
  function qstep(q, w, dt) {          // q <- q * exp(w dt / 2), w in body frame
    var n = norm(w), a = n * dt; if (a < 1e-12) return q;
    var s = Math.sin(a / 2) / n;
    return qnorm(qmul(q, [Math.cos(a / 2), w[0] * s, w[1] * s, w[2] * s]));
  }
  function qyaw(a) { return [Math.cos(-a / 2), 0, Math.sin(-a / 2), 0]; }   // nose points at (cos a, 0, sin a)
  function tiltOf(q) { return Math.acos(clamp(qrot(q, [0, 1, 0])[1], -1, 1)) * R2D; }   // 0 upright, 180 on the roof
  function rollOf(q) {                // signed roll about the body's own forward axis, + = right side down
    var up = qrot(q, [0, 1, 0]), rt = qrot(q, [0, 0, 1]);
    return Math.atan2(-rt[1], up[1]) * R2D;
  }
  function rng(seed) { var a = seed >>> 0; return function () { a = (a + 0x6D2B79F5) >>> 0; var t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
  function gaussFn(r) { return function () { var u = Math.max(1e-12, r()), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }; }

  // ------------------------------------------------------------------ the car (assumed values for a ~2 t electric SUV; not a published spec)
  var CAR = {
    m: 2000, I: [650, 3400, 3200],        // roll (x), yaw (y), pitch (z) inertia [kg m^2]
    len: 4.6, wid: 1.9, height: 1.5, comH: 0.61,
    hubs: [[1.45, -0.2, -0.82], [1.45, -0.2, 0.82], [-1.45, -0.2, -0.82], [-1.45, -0.2, 0.82]],
    wheelR: 0.36, susLen: 0.55, kS: 35000, cS: 4500,   // suspension: ray from the hub along -y_body, spring + damper
    crr: 0.015, cLat: 20000,                           // rolling resistance, lateral tyre damping (capped by mu*Fn)
    kH: 3e5, cH: 1.2e4, cF: 1e4                        // body-ground contact (penalty spring per hull point), friction regulariser
  };
  // hull points: the body shell that touches the ground when the car is not on its wheels
  var HULL = [];
  [-2.3, 0, 2.3].forEach(function (x) { [-1, 1].forEach(function (s) {
    HULL.push({ r: [x, -0.35, 0.9 * s], tag: "floor" }); HULL.push({ r: [x, 0.25, 0.95 * s], tag: "side" }); }); });
  [-1.1, 0, 1.1].forEach(function (x) { [-1, 1].forEach(function (s) { HULL.push({ r: [x, 0.9, 0.75 * s], tag: "roof" }); }); });

  // cold-gas thrusters: r = mount point (body), d = force direction (body). Exhaust leaves along -d.
  // 8 vertical thrusters at the four corners (up/down pairs: roll + pitch), 4 lateral at the bumpers (yaw).
  var THRUSTERS = [];
  [1.9, -1.9].forEach(function (x) { [-1, 1].forEach(function (s) { [1, -1].forEach(function (u) {
    THRUSTERS.push({ r: [x, 0.35, 0.92 * s], d: [0, u, 0], name: (x > 0 ? "trước" : "sau") + (s > 0 ? "-phải" : "-trái") + (u > 0 ? " ↑" : " ↓") }); }); }); });
  [2.25, -2.25].forEach(function (x) { [1, -1].forEach(function (s) {
    THRUSTERS.push({ r: [x, 0.1, 0], d: [0, 0, s], name: (x > 0 ? "mũi" : "đuôi") + (s > 0 ? " →phải" : " →trái") }); }); });
  THRUSTERS.forEach(function (t) { t.tq = cross(t.r, t.d); t.yaw = Math.abs(t.tq[1]) > Math.abs(t.tq[0]) && Math.abs(t.tq[1]) > Math.abs(t.tq[2]); });

  /* thrusters: F per nozzle [N], isp [s] (N2 cold gas: ~73 s in vacuum, ~65 s at sea level), fuel = N2 mass [kg],
     dry = tank (composite, 300 bar) + 12 valves + piping [kg], P0 = tank pressure [bar], lat = valve latency [s],
     reg = pressure-regulated (constant thrust) or blowdown (thrust falls with the remaining tank pressure) */
  var THR_DEFAULTS = { on: true, F: 2500, isp: 65, fuel: 15, dry: 30, P0: 300, lat: 0.02, reg: true };
  var VEH_DEFAULTS = { m: 2000 };   // kerb mass [kg]; inertia and suspension scale with it (the CAR values are for 2000 kg)
  function n2Density(Pbar) { var Z = 1 + 0.00035 * Pbar; return Pbar * 1e5 * 0.028 / (8.314 * 293 * Z); }   // kg/m^3 at 20 C (rough Z)
  /* mass budget of the thruster system, for the panel and the README */
  function budget(veh, th) {
    var rho = n2Density(th.P0), tankL = th.fuel / rho * 1000, sys = th.dry + th.fuel, mdot = th.F / (th.isp * G);
    return { car: veh.m, sys: sys, total: veh.m + sys, pct: 100 * sys / veh.m, tankL: tankL, mdot: mdot,
             burn4: th.fuel / (4 * mdot), impulse: th.fuel * th.isp * G };
  }
  var EST_DEFAULTS = { mode: "gated", gyroSig: 0.01, gyroBias: 0.02, accSig: 0.25, kp: 5.0, gnssP: 0.3, gnssV: 0.1, init: 1.5 };
  var CTRL_DEFAULTS = { kp: 10, kd: 5, db: 0.08, yawW: 0.6, retro: true, retroT: 0.6 };
  // landing classes: on the wheels with tilt <= 25 deg and sink <= 9 m/s (about a 4 m free fall) is "safe"
  var SAFE = { tilt: 25, sink: 9 };

  // ------------------------------------------------------------------ scenarios
  /* Road: two lanes |z| < 3.5, gravel shoulder to zEdge. Beyond zEdge (right side): an earth berm rising Hb over Lb,
     then the embankment slope (slope m horizontal per m of drop) down to a field D metres below the road. */
  var SCEN = {
    berm: { name: "Lao qua gờ đất mép taluy", note: "Mất lái lệch phải ψ, bánh phải leo lên gờ đất ở mép đường đắp cao, xe lăn và bay xuống ruộng.",
      params: { v0: 25, psi: 14, slide: 0, z0: 1.75, Hb: 0.7, Lb: 1.4, D: 4, slope: 1.0, zSoft: 3.5, zEdge: 5.0, muRoad: 0.8, muSoft: 0.7, dv: 0, tImp: 0 } },
    trip: { name: "Trượt ngang vào lề đất mềm", note: "Xe đang trượt ngang, bánh phía trước cày vào đất mềm, lật ngay tại chỗ: gần như không có thời gian bay.",
      params: { v0: 20, psi: 18, slide: 75, z0: 1.75, Hb: 0, Lb: 1.0, D: 0, slope: 1.0, zSoft: 3.5, zEdge: 5.0, muRoad: 0.8, muSoft: 1.8, dv: 0, tImp: 0 } },
    impact: { name: "Bị húc hông rồi lao khỏi đường", note: "Xe khác húc vào hông sau bên trái, xe ta xoay và trượt ra mép đường đắp.",
      params: { v0: 25, psi: 0, slide: 0, z0: 1.75, Hb: 0.7, Lb: 1.4, D: 4, slope: 1.0, zSoft: 3.5, zEdge: 5.0, muRoad: 0.8, muSoft: 0.7, dv: 6, tImp: 0.3 } }
  };

  function makeTerrain(P) {
    var zE = P.zEdge, Lb = Math.max(P.Lb, 1e-3), Hb = P.Hb, D = P.D, sl = Math.max(P.slope, 0.2);
    var z1 = zE + Lb, z2 = z1 + (Hb + D) * sl;
    function H(x, z) {
      if (z <= zE) return 0;
      if (z <= z1) return Hb * (z - zE) / Lb;
      if (z <= z2) return Hb - (z - z1) / sl;
      return -D;
    }
    function dHdz(z) { return z <= zE ? 0 : z <= z1 ? Hb / Lb : z <= z2 ? -1 / sl : 0; }
    function normal(x, z) { return nrm([0, 1, -dHdz(z)]); }
    function mu(x, z) { return Math.abs(z) <= P.zSoft ? P.muRoad : P.muSoft; }
    return { H: H, normal: normal, mu: mu, breaks: [-3.5, 0, 3.5, zE, z1, z2], zE: zE, z1: z1, z2: z2 };
  }

  // ------------------------------------------------------------------ baseline attitude controller (swap it: opts.controller)
  /* o: { t, q (estimated), w (gyro, body), v, p (GNSS), air (detector), tti, fuel }. Returns a bitmask of thrusters to open.
     Goal: wheels down at touchdown, nose along the horizontal velocity so the car lands rolling instead of sliding sideways. */
  function baselineController(o, mem, K) {
    if (!o.air || o.fuel <= 0) return 0;
    var ub = qrot(o.q, [0, 1, 0]), th = Math.acos(clamp(ub[1], -1, 1));
    var ax = cross(ub, [0, 1, 0]), s = norm(ax);
    ax = s < 1e-3 ? qrot(o.q, [1, 0, 0]) : scl(ax, 1 / s);
    var phi = qinv(o.q, scl(ax, th));                                 // body rotation vector that brings the car upright
    var dir = nrm(phi);
    if (th > 120 * D2R && dot(o.w, dir) < -1.0) phi = scl(dir, -(2 * Math.PI - th));   // already spinning the other way fast: keep rolling through
    var hv = [o.v[0], 0, o.v[2]];
    if (norm(hv) > 2 && th < 70 * D2R) {
      var xb = qrot(o.q, [1, 0, 0]), hx = nrm([xb[0], 0, xb[2]]), hn = nrm(hv);
      var yawErr = Math.atan2(cross(hx, hn)[1], dot(hx, hn));
      phi = add(phi, qinv(o.q, [0, yawErr * K.yawW, 0]));
    }
    var tau = [0, 1, 2].map(function (i) { return o.I[i] * (K.kp * phi[i] - K.kd * o.w[i]); });
    var sg = [0, 1, 2].map(function (i) { return Math.abs(tau[i]) > K.db * o.tauMax[i] ? Math.sign(tau[i]) : 0; });
    // retro burn: nearly upright and about to land -> all upward corner thrusters soften the touchdown,
    // minus any whose torque would fight an attitude correction still in progress
    var retro = K.retro && th < 15 * D2R && o.tti > 0 && o.tti < K.retroT;
    var cmd = 0;
    THRUSTERS.forEach(function (t, i) {
      var on;
      if (t.yaw) on = sg[1] !== 0 && Math.sign(t.tq[1]) === sg[1];
      else if (retro) on = t.d[1] > 0 && !(sg[0] !== 0 && Math.sign(t.tq[0]) === -sg[0]) && !(sg[2] !== 0 && Math.sign(t.tq[2]) === -sg[2]);
      else on = (sg[0] !== 0 || sg[2] !== 0) && (sg[0] === 0 || Math.sign(t.tq[0]) === sg[0]) && (sg[2] === 0 || Math.sign(t.tq[2]) === sg[2]);
      if (on) cmd |= 1 << i;
    });
    return cmd;
  }

  // ------------------------------------------------------------------ one rollout
  /* opts: { scenario, params, thr, est, ctrl, controller, seed }
     returns per-frame (100 Hz) records for the viewer plus a summary for evaluation. */
  function rollout(opts) {
    opts = opts || {};
    var sk = opts.scenario || "berm", P = Object.assign({}, SCEN[sk].params, opts.params || {});
    var TH = Object.assign({}, THR_DEFAULTS, opts.thr || {}), EST = Object.assign({}, EST_DEFAULTS, opts.est || {}), K = Object.assign({}, CTRL_DEFAULTS, opts.ctrl || {});
    var ctrl = opts.controller || baselineController, mem = {};
    var ter = makeTerrain(P), r = rng(opts.seed || 1), gauss = gaussFn(r);
    // mass: the car, plus the thruster system (dry + gas) only when it is fitted; the car gets lighter as gas is vented
    var VEH = Object.assign({}, VEH_DEFAULTS, opts.veh || {}), ms = VEH.m / CAR.m, I = CAR.I.map(function (x) { return x * ms; });
    var fuel = TH.on ? TH.fuel : 0, mBase = VEH.m + (TH.on ? TH.dry : 0), m = mBase + fuel;
    var kS = CAR.kS * m / CAR.m, cS = CAR.cS * m / CAR.m, cLat = CAR.cLat * m / CAR.m;   // same ride height for any mass
    var tauMax = [0, 1, 2].map(function (i) { return THRUSTERS.reduce(function (s, t) { return s + Math.max(0, t.tq[i]) * TH.F; }, 0); });

    // initial state: on its wheels, suspension at rest, moving at v0 along heading psi, body yawed by `slide` (a skid)
    var psi = P.psi * D2R, yaw = (P.psi + (P.slide || 0)) * D2R;
    var p = [0, ter.H(0, P.z0) + CAR.comH, P.z0], v = [P.v0 * Math.cos(psi), 0, P.v0 * Math.sin(psi)], q = qyaw(yaw), w = [0, 0, 0];
    var comp0 = m * G / (4 * kS), compPrev = [comp0, comp0, comp0, comp0];
    var Fng = [0, m * G, 0];

    // sensors and estimator state
    var bias = [gauss() * EST.gyroBias, gauss() * EST.gyroBias, gauss() * EST.gyroBias];
    var e0 = nrm([gauss(), gauss(), gauss()]), qe = qnorm(qmul(q, [Math.cos(EST.init * D2R / 2), e0[0] * Math.sin(EST.init * D2R / 2), e0[1] * Math.sin(EST.init * D2R / 2), e0[2] * Math.sin(EST.init * D2R / 2)]));
    var pe = p.slice(), ve = v.slice(), air = false, lowCnt = 0, calmCnt = 0, gm = [0, 0, 0], fm = [0, G, 0];
    var latF = Math.round(TH.lat / FDT), cmdHist = new Int32Array(NF), active = 0;

    // records
    var REC = { P: new Float32Array(NF * 3), Q: new Float32Array(NF * 4), QE: new Float32Array(NF * 4), FIRE: new Int32Array(NF),
      tilt: new Float32Array(NF), tiltE: new Float32Array(NF), roll: new Float32Array(NF), rollE: new Float32Array(NF),
      fmag: new Float32Array(NF), air: new Uint8Array(NF), contact: new Uint8Array(NF), tti: new Float32Array(NF), fuel: new Float32Array(NF), speed: new Float32Array(NF) };
    var events = [], roofHit = false, sideHit = false, peakG = 0, maxTilt = 0;
    var lastContactT = 0, inFlight = false, flightStart = 0, cand = null, flights = [], tDetect = null, tRollWarn = null, tFuelOut = null;
    function ev(t, kind, text) { events.push({ t: t, kind: kind, text: text }); }

    for (var step = 0; step <= NSTEP; step++) {
      var t = step * DT;
      // ---------------------------------------------------- 100 Hz: sensors, estimator, controller, record
      if (step % SUB === 0) {
        var f = step / SUB;
        // IMU: gyro = body rate + bias + noise; accelerometer = specific force (non-gravity force / m) in body frame, +-16 g
        gm = [w[0] + bias[0] + gauss() * EST.gyroSig, w[1] + bias[1] + gauss() * EST.gyroSig, w[2] + bias[2] + gauss() * EST.gyroSig];
        var fb = qinv(q, scl(Fng, 1 / m));
        fm = [0, 1, 2].map(function (i) { return clamp(fb[i] + gauss() * EST.accSig, -16 * G, 16 * G); });
        var fn = norm(fm);
        // free-fall detector: |f| < 0.4 g for 30 ms -> airborne; leaves on an impact spike or 0.2 s of a calm ~1 g
        if (!air) { lowCnt = fn < 0.4 * G ? lowCnt + 1 : 0; if (lowCnt >= 3) { air = true; calmCnt = 0; if (tDetect == null) { tDetect = t; ev(t, "detect", "phát hiện rơi tự do"); } } }
        else { calmCnt = Math.abs(fn - G) < 0.2 * G ? calmCnt + 1 : 0; if (fn > 1.8 * G || calmCnt >= 20) { air = false; lowCnt = 0; } }
        // attitude estimate
        if (EST.mode === "truth") qe = q.slice();
        else {
          var wc = gm.slice();
          var useAcc = EST.mode === "naive" ? fn > 1e-6 : (!air && Math.abs(fn - G) < 0.15 * G);
          if (useAcc) { var upM = scl(fm, 1 / fn), upE = qinv(qe, [0, 1, 0]), e = cross(upM, upE); wc = add(wc, scl(e, EST.kp)); }
          qe = qstep(qe, wc, FDT);
        }
        if (f % 10 === 0) { pe = [p[0] + gauss() * EST.gnssP, p[1] + gauss() * EST.gnssP, p[2] + gauss() * EST.gnssP]; ve = [v[0] + gauss() * EST.gnssV, v[1] + gauss() * EST.gnssV, v[2] + gauss() * EST.gnssV]; }
        else { pe = add(pe, scl(ve, FDT)); ve = [ve[0], ve[1] - (air ? G * FDT : 0), ve[2]]; }
        // time to impact from the ballistic arc over the terrain (two refinements of the landing point)
        var hgt = pe[1] - CAR.comH - ter.H(pe[0], pe[2]), tti = 0;
        if (hgt > 0) for (var it = 0; it < 3; it++) { tti = (ve[1] + Math.sqrt(Math.max(0, ve[1] * ve[1] + 2 * G * hgt))) / G; hgt = pe[1] - CAR.comH - ter.H(pe[0] + ve[0] * tti, pe[2] + ve[2] * tti); if (hgt <= 0) { tti = 0; break; } }
        var tiltE = tiltOf(qe);
        if (tRollWarn == null && tiltE > 50) { tRollWarn = t; ev(t, "warn", "cảnh báo lật (ước lượng > 50°)"); }
        // controller -> commands take effect after the valve latency
        cmdHist[f] = TH.on ? ctrl({ t: t, q: qe, w: gm, v: ve, p: pe, air: air, tti: tti, fuel: fuel, tauMax: tauMax, I: I }, mem, K) : 0;
        active = f >= latF ? cmdHist[f - latF] : 0;
        if (fuel <= 0) active = 0;
        // record
        REC.P.set(p, f * 3); REC.Q.set(q, f * 4); REC.QE.set(qe, f * 4); REC.FIRE[f] = active;
        var tl = tiltOf(q); REC.tilt[f] = tl; REC.tiltE[f] = tiltE; REC.roll[f] = rollOf(q); REC.rollE[f] = rollOf(qe);
        REC.fmag[f] = fn / G; REC.air[f] = air ? 1 : 0; REC.tti[f] = tti; REC.fuel[f] = fuel; REC.speed[f] = norm(v);
        maxTilt = Math.max(maxTilt, tl);
      }

      // ---------------------------------------------------- forces
      var F = [0, -m * G, 0], tauW = [0, 0, 0], tauB = [0, 0, 0], Fn = [0, 0, 0], contact = false;
      var wW = qrot(q, w);
      function applyW(Fw, pt) { Fn = add(Fn, Fw); tauW = add(tauW, cross(sub(pt, p), Fw)); }
      // wheels (only while the car is roughly on its wheels; otherwise the hull takes the contact)
      var yb = qrot(q, [0, 1, 0]), xb = qrot(q, [1, 0, 0]);
      for (var k = 0; k < 4; k++) {
        if (yb[1] < 0.35) { compPrev[k] = 0; continue; }
        var hub = add(p, qrot(q, CAR.hubs[k])), end = sub(hub, scl(yb, CAR.susLen));
        var pen = ter.H(end[0], end[2]) - end[1];
        if (pen <= 0) { compPrev[k] = 0; continue; }
        var comp = Math.min(pen / yb[1], CAR.susLen), crate = (comp - compPrev[k]) / DT; compPrev[k] = comp;
        var fs = Math.max(0, kS * comp + cS * crate);
        if (fs <= 0) continue;
        contact = true;
        var cp = sub(hub, scl(yb, CAR.susLen - comp)), n = ter.normal(cp[0], cp[2]);
        var vc = add(v, cross(wW, sub(cp, p))), vt = sub(vc, scl(n, dot(vc, n)));
        var fw = nrm(sub(xb, scl(n, dot(xb, n)))), lw = cross(n, fw), mu = ter.mu(cp[0], cp[2]), cap = mu * fs;
        var vl = dot(vt, lw), vf = dot(vt, fw);
        var Fl = -clamp(cLat * vl, -cap, cap), Ff = -clamp(cLat * vf, -CAR.crr * fs, CAR.crr * fs);
        applyW(add(scl(n, fs), add(scl(lw, Fl), scl(fw, Ff))), cp);
      }
      // hull points: penalty spring-damper along the ground normal + Coulomb friction
      for (var h = 0; h < HULL.length; h++) {
        var hp = add(p, qrot(q, HULL[h].r)), pn = ter.H(hp[0], hp[2]) - hp[1];
        if (pn <= 0) continue;
        var nn = ter.normal(hp[0], hp[2]), vh = add(v, cross(wW, sub(hp, p))), vn = dot(vh, nn);
        var fnh = Math.max(0, CAR.kH * pn * nn[1] - CAR.cH * vn);
        if (fnh <= 0) continue;
        contact = true;
        if (HULL[h].tag === "roof" && !roofHit) { roofHit = true; ev(t, "roof", "nóc xe chạm đất"); }
        if (HULL[h].tag === "side" && !sideHit) { sideHit = true; ev(t, "side", "sườn xe chạm đất"); }
        var vth = sub(vh, scl(nn, vn)), vtn = norm(vth), ft = vtn > 1e-6 ? scl(vth, -Math.min(ter.mu(hp[0], hp[2]) * fnh, CAR.cF * vtn) / vtn) : [0, 0, 0];
        applyW(add(scl(nn, fnh), ft), hp);
      }
      // thrusters
      if (active && fuel > 0) {
        var nOn = 0, Fe = TH.F * (TH.reg ? 1 : fuel / TH.fuel);   // blowdown: thrust follows the tank pressure (isothermal, p ~ gas mass)
        for (var j = 0; j < THRUSTERS.length; j++) {
          if (!(active & (1 << j))) continue;
          var T = THRUSTERS[j]; nOn++;
          Fn = add(Fn, qrot(q, scl(T.d, Fe))); tauB = add(tauB, scl(T.tq, Fe));
        }
        fuel -= nOn * Fe / (TH.isp * G) * DT;
        if (!TH.reg && fuel < 0.05 * TH.fuel) fuel = 0;               // blowdown: below 5 % the pressure no longer makes useful thrust
        if (fuel <= 0 && tFuelOut == null) { fuel = 0; tFuelOut = t; ev(t, "fuel", "hết khí nén"); }
        m = mBase + Math.max(0, fuel);
      }
      // an external hit (scenario "impact"): a short force pulse at the left rear quarter
      if (P.dv > 0 && t >= P.tImp && t < P.tImp + 0.08) {
        if (t < P.tImp + DT / 2) ev(t, "hit", "bị húc hông");
        applyW([0, 0, m * P.dv / 0.08], add(p, qrot(q, [-1.6, -0.1, -0.95])));
      }
      F = add(F, Fn); Fng = Fn;
      peakG = Math.max(peakG, norm(Fn) / m / G);
      tauB = add(tauB, qinv(q, tauW));

      // ---------------------------------------------------- flight bookkeeping (ground truth)
      // a flight ends only when contact lasts 60 ms (a wheel scraping the berm crest is not a landing)
      if (contact) {
        if (inFlight) {
          if (!cand) cand = { t1: t, tilt: tiltOf(q), sink: -v[1], rate: norm(w) * R2D, speed: norm(v) };
          else if (t - cand.t1 >= 0.06) {
            inFlight = false;
            var dur = cand.t1 - flightStart;
            if (dur >= 0.15) {
              var fl = Object.assign({ t0: flightStart, dur: dur }, cand);
              flights.push(fl); ev(cand.t1, "touch", "tiếp đất (nghiêng " + Math.round(fl.tilt) + "°)");
            }
            cand = null;
          }
        }
        lastContactT = t;
      } else if (inFlight) cand = null;
      else if (t - lastContactT > 0.05) {
        inFlight = true; flightStart = lastContactT;
        if (!flights.length) ev(lastContactT, "launch", "rời mặt đất");
      }

      // ---------------------------------------------------- integrate (semi-implicit Euler)
      if (step === NSTEP) break;
      v = add(v, scl(F, DT / m)); p = add(p, scl(v, DT));
      var Iw = [I[0] * w[0], I[1] * w[1], I[2] * w[2]], gyr = cross(w, Iw);
      w = [w[0] + (tauB[0] - gyr[0]) / I[0] * DT, w[1] + (tauB[1] - gyr[1]) / I[1] * DT, w[2] + (tauB[2] - gyr[2]) / I[2] * DT];
      q = qstep(q, w, DT);
      if (step % SUB === 0) REC.contact[step / SUB] = contact ? 1 : 0;
    }

    // ---------------------------------------------------- outcome
    var upFinal = qrot(q, [0, 1, 0])[1], main = flights.reduce(function (b, x) { return !b || x.dur > b.dur ? x : b; }, null);
    var first = flights[0] || null;
    var outcome = upFinal < 0.5 || roofHit ? "flip" : first && (first.tilt > SAFE.tilt || first.sink > SAFE.sink) ? "hard" : "safe";
    var errAtTouch = null;
    if (first) { var ft = Math.min(NF - 1, Math.round(first.t1 / FDT)); errAtTouch = Math.abs(REC.tilt[ft] - REC.tiltE[ft]); }
    var tLaunch = flights.length ? flights[0].t0 : null;
    return {
      R: REC, events: events.sort(function (a, b) { return a.t - b.t; }), terrain: ter, params: P, thr: TH, est: EST, scenario: sk,
      summary: {
        outcome: outcome, roofHit: roofHit, sideHit: sideHit, upFinal: upFinal, maxTilt: maxTilt, peakG: peakG,
        airtime: main ? main.dur : 0, touch: first, fuelUsed: (TH.on ? TH.fuel : 0) - fuel, mass0: mBase + (TH.on ? TH.fuel : 0), tDetect: tDetect, tLaunch: tLaunch,
        detectLatency: tDetect != null && tLaunch != null ? tDetect - tLaunch : null, tRollWarn: tRollWarn, estErrTouch: errAtTouch
      }
    };
  }

  // ------------------------------------------------------------------ evaluation helpers
  function wilson(k, n, z) {
    z = z || 1.96; if (!n) return [0, 1];
    var ph = k / n, d = 1 + z * z / n, c = (ph + z * z / (2 * n)) / d, h = z * Math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / d;
    return [Math.max(0, c - h), Math.min(1, c + h)];
  }
  /* a random "berm" scenario for Monte Carlo (a judge can also pick any of these by hand in the viewer) */
  var RANGES = { v0: [18, 32], psi: [8, 22], Hb: [0.4, 1.0], Lb: [1.0, 2.0], D: [2.5, 6] };
  function randomParams(seed) {
    var r = rng(seed * 7919 + 13), P = {};
    Object.keys(RANGES).forEach(function (k) { var a = RANGES[k]; P[k] = Math.round((a[0] + (a[1] - a[0]) * r()) * 100) / 100; });
    return P;
  }
  var CONFIGS = [
    { key: "none", name: "Không vòi phun", thr: { on: false }, est: { mode: "gated" } },
    { key: "gated", name: "Vòi phun + ước lượng có cổng (gyro khi bay)", thr: { on: true }, est: { mode: "gated" } },
    { key: "naive", name: "Vòi phun + bộ lọc ngây thơ (tin gia tốc kế)", thr: { on: true }, est: { mode: "naive" } },
    { key: "truth", name: "Vòi phun + tư thế thật (cận trên)", thr: { on: true }, est: { mode: "truth" } }
  ];

  /* one Monte Carlo sample: the same random scene under every configuration (paired comparison) */
  function evalSeed(seed, base) {
    base = base || {};
    var P = randomParams(seed), row = { seed: seed, params: P, out: {} };
    CONFIGS.forEach(function (c) {
      var r = rollout({ scenario: "berm", params: P, thr: Object.assign({}, base.thr || {}, c.thr), est: Object.assign({}, base.est || {}, c.est), ctrl: base.ctrl, veh: base.veh, seed: seed });
      var s = r.summary;
      // severity proxy (not an injury model): roof contact is the worst case in real rollovers (roof crush)
      var sev = s.outcome === "safe" ? "safe" : s.outcome === "hard" ? "hard" : s.roofHit ? "roof" : "side";
      row.out[c.key] = { outcome: s.outcome, sev: sev, tilt: s.touch ? s.touch.tilt : null, sink: s.touch ? s.touch.sink : null, estErr: s.estErrTouch,
                         air: s.airtime, fuel: s.fuelUsed, maxTilt: s.maxTilt, detect: s.detectLatency, tLaunch: s.tLaunch };
    });
    return row;
  }
  function summarize(rows) {
    return CONFIGS.map(function (c) {
      var n = rows.length, safe = 0, hard = 0, flip = 0, resK = 0, resN = 0, eS = 0, eN = 0;
      rows.forEach(function (r) {
        var o = r.out[c.key]; if (o.outcome === "safe") safe++; else if (o.outcome === "hard") hard++; else flip++;
        if (r.out.none.outcome !== "safe") { resN++; if (o.outcome === "safe") resK++; }
        if (o.estErr != null) { eS += o.estErr; eN++; }
      });
      return { key: c.key, name: c.name, n: n, safe: safe, hard: hard, flip: flip, ci: wilson(safe, n), rescue: [resK, resN], rescueCi: wilson(resK, resN), estErr: eN ? eS / eN : null };
    });
  }

  var api = {
    G: G, DT: DT, FDT: FDT, T_END: T_END, NF: NF, CAR: CAR, SAFE: SAFE, HULL: HULL, THRUSTERS: THRUSTERS, SCEN: SCEN,
    THR_DEFAULTS: THR_DEFAULTS, VEH_DEFAULTS: VEH_DEFAULTS, budget: budget, EST_DEFAULTS: EST_DEFAULTS, CTRL_DEFAULTS: CTRL_DEFAULTS, RANGES: RANGES, CONFIGS: CONFIGS,
    rollout: rollout, makeTerrain: makeTerrain, baselineController: baselineController, wilson: wilson, randomParams: randomParams, evalSeed: evalSeed, summarize: summarize,
    qrot: qrot, qinv: qinv, tiltOf: tiltOf, rollOf: rollOf
  };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.FlipCore = api;
})(this);
