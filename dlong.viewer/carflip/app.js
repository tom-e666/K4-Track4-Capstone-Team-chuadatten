/* app.js — three.js view of FlipCore: the run is simulated once (1 kHz physics, 100 Hz records), then replayed and scrubbed. */
(function () {
  "use strict";
  var C = window.FlipCore, T = window.THREE;
  if (!C || !T) { document.getElementById("body").innerHTML = "<p>Không tải được thư viện 3D (vendor/three.min.js).</p>"; return; }
  var $ = function (id) { return document.getElementById(id); };
  var FDT = C.FDT, NF = C.NF, T_END = C.T_END;
  function fmt(x, d) { return x == null || isNaN(x) ? "–" : x.toFixed(d == null ? 1 : d).replace(".", ","); }
  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
  function css(k) { return getComputedStyle(document.documentElement).getPropertyValue("--" + k).trim(); }
  function pop(m) { var n = 0; while (m) { n += m & 1; m >>>= 1; } return n; }
  var REDUCED = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  // ================================================================== state
  var st = {
    sc: "berm", params: Object.assign({}, C.SCEN.berm.params), thr: Object.assign({}, C.THR_DEFAULTS), est: Object.assign({}, C.EST_DEFAULTS),
    ctrl: Object.assign({}, C.CTRL_DEFAULTS), veh: Object.assign({}, C.VEH_DEFAULTS), seed: 1, t: 0, playing: !REDUCED, speed: 1, ghost: true, estBox: false, cam: 0
  };
  var RUN = null, GHOST = null, terKey = "";

  // ================================================================== renderer, scene, lights
  var stage = $("stage"), labels = $("labels");
  var renderer = new T.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap;
  stage.insertBefore(renderer.domElement, labels);
  var scene = new T.Scene(); scene.background = new T.Color(0xdde6ee); scene.fog = new T.Fog(0xdde6ee, 120, 420);
  var camera = new T.PerspectiveCamera(40, 1, 0.2, 2000);
  scene.add(new T.HemisphereLight(0xf4f8fb, 0x7d8a72, 0.75));
  var sun = new T.DirectionalLight(0xffffff, 0.7); sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
  var sc = sun.shadow.camera; sc.left = -35; sc.right = 35; sc.top = 35; sc.bottom = -35; sc.near = 5; sc.far = 200; sun.shadow.bias = -0.0006;
  scene.add(sun); scene.add(sun.target);

  // ================================================================== terrain (rebuilt when the cross-section changes)
  var terrain = new T.Group(); scene.add(terrain);
  var MAT = {
    asphalt: new T.MeshLambertMaterial({ color: 0x8d969f, side: T.DoubleSide }), gravel: new T.MeshLambertMaterial({ color: 0xb9ae98, side: T.DoubleSide }),
    berm: new T.MeshLambertMaterial({ color: 0x9c8466, side: T.DoubleSide }), slope: new T.MeshLambertMaterial({ color: 0x93ad79, side: T.DoubleSide }),
    grass: new T.MeshLambertMaterial({ color: 0xa9c38f, side: T.DoubleSide }), field: new T.MeshLambertMaterial({ color: 0xb6cf8e, side: T.DoubleSide }),
    mark: new T.MeshBasicMaterial({ color: 0xfbfcfd }), post: new T.MeshLambertMaterial({ color: 0xf2f2f2 }), postTop: new T.MeshBasicMaterial({ color: 0xe0662e }),
    crown: new T.MeshLambertMaterial({ color: 0x7fa36a }), trunk: new T.MeshLambertMaterial({ color: 0x8b7a68 })
  };
  function clearGroup(g) { while (g.children.length) { var o = g.children.pop(); if (o.geometry) o.geometry.dispose(); } }
  function buildTerrain(ter) {
    clearGroup(terrain);
    var X0 = -60, X1 = 300;
    function strip(z0, z1, m) {
      if (z1 - z0 < 1e-4) return;
      var y0 = ter.H(0, z0), y1 = ter.H(0, z1), g = new T.BufferGeometry();
      g.setAttribute("position", new T.Float32BufferAttribute([X0, y0, z0, X1, y1, z1, X1, y0, z0, X0, y0, z0, X0, y1, z1, X1, y1, z1], 3));
      g.computeVertexNormals();
      var o = new T.Mesh(g, m); o.receiveShadow = true; terrain.add(o);
    }
    var zs = [-60, -6, -3.5, 3.5, ter.zE, ter.z1, ter.z2, ter.z2 + 80], ms = [MAT.grass, MAT.gravel, MAT.asphalt, MAT.gravel, MAT.berm, MAT.slope, MAT.field];
    for (var i = 0; i < ms.length; i++) strip(zs[i], zs[i + 1], ms[i]);
    // lane markings: dashed centre line, solid edge lines
    var PL = new T.PlaneGeometry(1, 1); PL.rotateX(-Math.PI / 2);
    function mark(x, z, len, w) { var o = new T.Mesh(PL, MAT.mark); o.scale.set(len, 1, w); o.position.set(x, 0.012, z); terrain.add(o); }
    for (var x = X0; x < X1; x += 9) mark(x + 1.5, 0, 3, 0.14);
    mark((X0 + X1) / 2, 3.35, X1 - X0, 0.16); mark((X0 + X1) / 2, -3.35, X1 - X0, 0.16);
    // delineator posts on the shoulder, for scale and speed
    var PB = new T.BoxGeometry(0.12, 1.0, 0.12), PT = new T.BoxGeometry(0.13, 0.18, 0.13);
    for (var xp = X0; xp < X1; xp += 12) {
      var zp = ter.zE - 0.4, p = new T.Mesh(PB, MAT.post); p.position.set(xp, 0.5, zp); p.castShadow = true; terrain.add(p);
      var tp = new T.Mesh(PT, MAT.postTop); tp.position.set(xp, 0.95, zp); terrain.add(tp);
    }
    // trees in the field below the embankment and beyond the far side
    var cg = new T.IcosahedronGeometry(1.8, 0), tg = new T.CylinderGeometry(0.16, 0.22, 2.6, 6), seed = 7;
    function rnd() { seed = (seed * 16807) % 2147483647; return seed / 2147483647; }
    for (var k = 0; k < 70; k++) {
      var far = k % 3 === 0, tx = X0 + rnd() * (X1 - X0), tz = far ? -12 - rnd() * 40 : ter.z2 + 14 + rnd() * 50, ty = ter.H(tx, tz), s = 0.8 + rnd() * 0.6;
      var cr = new T.Mesh(cg, MAT.crown); cr.position.set(tx, ty + 3.4 * s, tz); cr.scale.set(s, s * 1.2, s); cr.castShadow = true; terrain.add(cr);
      var tr = new T.Mesh(tg, MAT.trunk); tr.position.set(tx, ty + 1.3, tz); terrain.add(tr);
    }
  }

  // ================================================================== the car, its thrusters, the ghost and the estimate frame
  var WHEEL = new T.CylinderGeometry(0.36, 0.36, 0.26, 18); WHEEL.rotateX(Math.PI / 2);
  var PLUME = new T.ConeGeometry(0.26, 1.7, 14, 1, true); PLUME.translate(0, -0.85, 0);      // apex at the nozzle, opening along -y
  function carMesh(color, opacity) {
    var g = new T.Group(), ghost = opacity < 1;
    function m(c) { return new T.MeshLambertMaterial({ color: c, transparent: ghost, opacity: opacity, depthWrite: !ghost }); }
    function box(w, h, d, x, y, z, mat) { var o = new T.Mesh(new T.BoxGeometry(w, h, d), mat); o.position.set(x, y, z); o.castShadow = !ghost; g.add(o); return o; }
    var paint = m(color), glass = m(0x2d3a48);
    box(4.6, 0.75, 1.9, 0, 0.025, 0, paint);                    // body: floor at -0.35, belt at 0.4 (CoM frame)
    box(2.3, 0.46, 1.52, -0.15, 0.63, 0, glass);                // cabin
    box(2.0, 0.06, 1.46, -0.2, 0.88, 0, paint);                 // roof at 0.9
    box(0.06, 0.12, 0.42, 2.3, 0.18, 0.62, m(0xfff4c8)); box(0.06, 0.12, 0.42, 2.3, 0.18, -0.62, m(0xfff4c8));
    box(0.06, 0.12, 0.4, -2.3, 0.2, 0.64, m(0xd2453a)); box(0.06, 0.12, 0.4, -2.3, 0.2, -0.64, m(0xd2453a));
    C.CAR.hubs.forEach(function (h) { var w = new T.Mesh(WHEEL, m(0x23282e)); w.position.set(h[0], -0.25, h[2] * 1.05); w.castShadow = !ghost; g.add(w); });
    g.userData.plumes = [];
    if (!ghost) {
      var nz = m(0x3b434c), pm = new T.MeshBasicMaterial({ color: 0xd9efff, transparent: true, opacity: 0.6, depthWrite: false });
      C.THRUSTERS.forEach(function (t) {
        var e = new T.Vector3(-t.d[0], -t.d[1], -t.d[2]);
        var n = new T.Mesh(new T.CylinderGeometry(0.07, 0.09, 0.16, 8), nz); n.position.set(t.r[0], t.r[1], t.r[2]);
        n.quaternion.setFromUnitVectors(new T.Vector3(0, 1, 0), e); g.add(n);
        var p = new T.Mesh(PLUME, pm); p.position.set(t.r[0] + e.x * 0.08, t.r[1] + e.y * 0.08, t.r[2] + e.z * 0.08);
        p.quaternion.setFromUnitVectors(new T.Vector3(0, -1, 0), e); p.visible = false; g.add(p); g.userData.plumes.push(p);
      });
    }
    scene.add(g); return g;
  }
  var car = carMesh(0x158f78, 1), ghostCar = carMesh(0x8a929c, 0.3);
  var estBox = new T.LineSegments(new T.EdgesGeometry(new T.BoxGeometry(4.6, 1.25, 1.9).translate(0, 0.275, 0)), new T.LineBasicMaterial({ color: 0x1f8fb5 }));
  scene.add(estBox);

  // ================================================================== replay
  var V = new T.Vector3(), Qa = new T.Quaternion(), Qb = new T.Quaternion();
  function frameAt(t) { var x = clamp(t / FDT, 0, NF - 1.0001), f = Math.floor(x); return { f: f, u: x - f }; }
  function poseOf(R, key, t, obj) {
    var fu = frameAt(t), f = fu.f, u = fu.u, P = R.P, Q = R[key];
    if (obj) obj.position.set(P[f * 3] + (P[f * 3 + 3] - P[f * 3]) * u, P[f * 3 + 1] + (P[f * 3 + 4] - P[f * 3 + 1]) * u, P[f * 3 + 2] + (P[f * 3 + 5] - P[f * 3 + 2]) * u);
    Qa.set(Q[f * 4 + 1], Q[f * 4 + 2], Q[f * 4 + 3], Q[f * 4]); Qb.set(Q[f * 4 + 5], Q[f * 4 + 6], Q[f * 4 + 7], Q[f * 4 + 4]); Qa.slerp(Qb, u);
    if (obj) obj.quaternion.copy(Qa);
    return fu;
  }

  // ================================================================== camera rig
  var CAMS = [{ name: "sau xe", yaw: Math.PI + 0.38, pitch: 0.3, dist: 20 }, { name: "bên hông", yaw: -Math.PI / 2 - 0.15, pitch: 0.22, dist: 26 },
    { name: "trên cao", yaw: Math.PI + 0.25, pitch: 1.2, dist: 38 }, { name: "tự do", free: true }];
  var rig = { tx: 0, ty: 1, tz: 1.75, yaw: CAMS[0].yaw, pitch: CAMS[0].pitch, dist: CAMS[0].dist };
  function setCam(i) {
    st.cam = i; var c = CAMS[i]; $("camBtn").textContent = "camera: " + c.name;
    if (!c.free) { rig.yaw = c.yaw; rig.pitch = c.pitch; rig.dist = c.dist; }
  }
  (function pointer() {
    var el = renderer.domElement, last = null;
    el.addEventListener("pointerdown", function (e) { last = [e.clientX, e.clientY]; el.setPointerCapture(e.pointerId); });
    el.addEventListener("pointermove", function (e) {
      if (!last) return; var dx = e.clientX - last[0], dy = e.clientY - last[1]; last = [e.clientX, e.clientY];
      rig.yaw += dx * 0.006; rig.pitch = clamp(rig.pitch + dy * 0.004, 0.03, 1.5);
      if (st.cam !== 3) { st.cam = 3; $("camBtn").textContent = "camera: tự do"; }
    });
    el.addEventListener("pointerup", function () { last = null; });
    el.addEventListener("wheel", function (e) { e.preventDefault(); rig.dist = clamp(rig.dist * Math.exp(e.deltaY * 0.0012), 5, 200); }, { passive: false });
  })();

  // ================================================================== side panel: scenario, system, estimator
  var deg = function (v) { return fmt(v, 0) + "°"; }, met = function (v) { return fmt(v, 2) + " m"; }, kmh = function (v) { return fmt(v * 3.6, 0) + " km/h (" + fmt(v, 1) + " m/s)"; };
  var SC_FIELDS = {
    v0: ["Tốc độ lúc mất lái", 10, 35, 0.5, kmh], psi: ["Góc lệch khỏi hướng đường", 0, 30, 0.5, deg],
    slide: ["Thân xe xoay ngang so với hướng trượt", 0, 90, 1, deg], Hb: ["Chiều cao gờ đất mép đường", 0, 1.4, 0.05, met],
    Lb: ["Bề rộng mặt dốc của gờ", 0.6, 3, 0.1, met], D: ["Độ cao taluy (đường đắp)", 0, 8, 0.1, met],
    muSoft: ["Ma sát lề đất (bánh cày vào đất)", 0.4, 2.2, 0.05, function (v) { return "μ " + fmt(v, 2); }],
    dv: ["Cú húc: Δv ngang", 0, 10, 0.5, function (v) { return fmt(v, 1) + " m/s"; }], tImp: ["Thời điểm bị húc", 0.1, 2, 0.05, function (v) { return fmt(v, 2) + " s"; }]
  };
  var SC_SHOW = { berm: ["v0", "psi", "Hb", "Lb", "D"], trip: ["v0", "psi", "slide", "muSoft"], impact: ["v0", "dv", "tImp", "Hb", "D"] };
  var RAND = { v0: [18, 32], psi: [8, 22], Hb: [0.4, 1.0], Lb: [1.0, 2.0], D: [2.5, 6], slide: [50, 90], muSoft: [1.2, 2.0], dv: [3, 9], tImp: [0.2, 0.8] };
  var ADV = [
    ["thr", "F", "Lực mỗi vòi", 500, 6000, 100, function (v) { return fmt(v / 1000, 1) + " kN"; }],
    ["thr", "lat", "Độ trễ van", 0, 0.15, 0.005, function (v) { return fmt(v * 1000, 0) + " ms"; }],
    ["est", "gyroBias", "Độ lệch gyro (σ)", 0, 0.2, 0.005, function (v) { return fmt(v, 3) + " rad/s"; }],
    ["est", "accSig", "Nhiễu gia tốc kế (σ)", 0, 1.5, 0.05, function (v) { return fmt(v, 2) + " m/s²"; }],
    ["est", "kp", "Hệ số tin gia tốc kế k_p", 0.5, 15, 0.5, function (v) { return fmt(v, 1); }]
  ];
  var EST_MODES = [["gated", "Có cổng"], ["naive", "Ngây thơ"], ["truth", "Hoàn hảo"]];
  var EST_NOTE = {
    gated: "Gyro tích phân liên tục; gia tốc kế chỉ được dùng để sửa độ nghiêng khi |f| ≈ 1 g và xe không bay. Khi rơi tự do gia tốc kế đọc ≈ 0 g nên bị bỏ qua.",
    naive: "Ca lỗi: luôn tin gia tốc kế là “hướng lên”. Khi rơi tự do nó chỉ đo nhiễu, khi vòi phun bật nó đo lực đẩy, nên tư thế ước lượng trôi sai.",
    truth: "Cận trên: bộ điều khiển biết tư thế thật (không có trên xe thật). Dùng để tách lỗi ước lượng khỏi lỗi điều khiển."
  };
  function slider(id, f, v) {
    return '<div class="sl"><span>' + f[0] + '</span><output id="o' + id + '">' + f[4](v) + '</output><input type="range" id="' + id + '" min="' + f[1] + '" max="' + f[2] + '" step="' + f[3] + '" value="' + v + '"></div>';
  }
  function renderScenario() {
    $("scSeg").innerHTML = Object.keys(C.SCEN).map(function (k) { return '<button type="button" data-sc="' + k + '" aria-pressed="' + (k === st.sc) + '">' + C.SCEN[k].name + "</button>"; }).join("");
    $("scSeg").querySelectorAll("button").forEach(function (b) { b.addEventListener("click", function () { st.sc = b.dataset.sc; st.params = Object.assign({}, C.SCEN[st.sc].params); st.t = 0; st.playing = true; renderScenario(); rerun(); }); });
    $("scNote").textContent = C.SCEN[st.sc].note;
    var box = $("scFields");
    box.innerHTML = SC_SHOW[st.sc].map(function (k) { return slider("p_" + k, SC_FIELDS[k], st.params[k]); }).join("");
    SC_SHOW[st.sc].forEach(function (k) {
      var el = $("p_" + k); el.addEventListener("input", function () { st.params[k] = +el.value; $("op_" + k).textContent = SC_FIELDS[k][4](+el.value); later(); });
    });
    $("seedTxt").textContent = "seed " + st.seed;
  }
  function renderSystem() {
    $("estSeg").innerHTML = EST_MODES.map(function (m) { return '<button type="button" data-m="' + m[0] + '" aria-pressed="' + (st.est.mode === m[0]) + '">' + m[1] + "</button>"; }).join("");
    $("estSeg").querySelectorAll("button").forEach(function (b) { b.addEventListener("click", function () { st.est.mode = b.dataset.m; renderSystem(); rerun(); }); });
    $("estNote").textContent = EST_NOTE[st.est.mode];
    $("advFields").innerHTML = ADV.map(function (a) { return slider("a_" + a[1], [a[2], a[3], a[4], a[5], a[6]], st[a[0]][a[1]]); }).join("");
    ADV.forEach(function (a) { var el = $("a_" + a[1]); el.addEventListener("input", function () { st[a[0]][a[1]] = +el.value; $("oa_" + a[1]).textContent = a[6](+el.value); later(); }); });
  }
  // vehicle mass and gas: generic classes (assumed values, not any manufacturer's spec)
  var VEH_PRESETS = [["Xe nhỏ", 1400], ["SUV cỡ trung", 2000], ["SUV điện lớn", 2600]];
  var VEH_FIELDS = [
    ["veh", "m", "Khối lượng xe (không tải)", 1100, 3000, 50, function (v) { return fmt(v, 0) + " kg"; }],
    ["thr", "fuel", "Khí nén N₂ mang theo", 0, 40, 0.5, function (v) { return fmt(v, 1) + " kg"; }],
    ["thr", "P0", "Áp suất bình", 200, 700, 10, function (v) { return fmt(v, 0) + " bar"; }]
  ];
  function renderVehicle() {
    $("vehSeg").innerHTML = VEH_PRESETS.map(function (p) { return '<button type="button" data-m="' + p[1] + '" aria-pressed="' + (st.veh.m === p[1]) + '">' + p[0] + " ~" + fmt(p[1] / 1000, 1) + " t</button>"; }).join("");
    $("vehSeg").querySelectorAll("button").forEach(function (b) { b.addEventListener("click", function () { st.veh.m = +b.dataset.m; renderVehicle(); rerun(); }); });
    $("vehFields").innerHTML = VEH_FIELDS.map(function (a) { return slider("v_" + a[1], [a[2], a[3], a[4], a[5], a[6]], st[a[0]][a[1]]); }).join("");
    VEH_FIELDS.forEach(function (a) { var el = $("v_" + a[1]); el.addEventListener("input", function () { st[a[0]][a[1]] = +el.value; $("ov_" + a[1]).textContent = a[6](+el.value);
      if (a[1] === "m") $("vehSeg").querySelectorAll("button").forEach(function (b) { b.setAttribute("aria-pressed", +b.dataset.m === st.veh.m); }); renderBudget(); later(); }); });
    $("tReg").checked = st.thr.reg !== false;
  }
  function renderBudget() {
    var b = C.budget(st.veh, st.thr);
    function row(k, v) { return "<span>" + k + "</span><b>" + v + "</b>"; }
    $("budget").innerHTML = row("hệ thống (bình, van, ống) + khí", fmt(st.thr.dry, 0) + " + " + fmt(st.thr.fuel, 1) + " = " + fmt(b.sys, 1) + " kg") +
      row("tăng khối lượng xe", "+" + fmt(b.pct, 1) + " %") +
      row("thể tích bình N₂ ở " + fmt(st.thr.P0, 0) + " bar", "≈ " + fmt(b.tankL, 0) + " L") +
      row("lưu lượng mỗi vòi (Isp " + fmt(st.thr.isp, 0) + " s)", fmt(b.mdot, 1) + " kg/s") +
      row("4 vòi mở liên tục được", fmt(b.burn4, 2) + " s") +
      row("tổng xung", fmt(b.impulse / 1000, 1) + " kN·s");
  }
  $("tReg").addEventListener("change", function () { st.thr.reg = this.checked; rerun(); });
  $("tThr").addEventListener("change", function () { st.thr.on = this.checked; rerun(); });
  $("tRetro").addEventListener("change", function () { st.ctrl.retro = this.checked; rerun(); });
  $("tGhost").addEventListener("change", function () { st.ghost = this.checked; });
  $("tEstBox").addEventListener("change", function () { st.estBox = this.checked; });
  function randomize() {
    st.seed = 1 + Math.floor(Math.random() * 99999);
    SC_SHOW[st.sc].forEach(function (k) { var r = RAND[k], f = SC_FIELDS[k]; st.params[k] = Math.round((r[0] + Math.random() * (r[1] - r[0])) / f[3]) * f[3]; });
    st.t = 0; st.playing = true; renderScenario(); rerun();
  }
  $("bRand").addEventListener("click", randomize);
  $("bReset").addEventListener("click", function () { st.params = Object.assign({}, C.SCEN[st.sc].params); st.seed = 1; st.t = 0; st.playing = true; renderScenario(); rerun(); });

  // ================================================================== running
  var timer = 0;
  function later() { clearTimeout(timer); timer = setTimeout(rerun, 60); }
  function runOpts(thrOn) { return { scenario: st.sc, params: st.params, thr: Object.assign({}, st.thr, { on: thrOn }), est: st.est, ctrl: st.ctrl, veh: st.veh, seed: st.seed }; }
  function rerun() {
    RUN = C.rollout(runOpts(st.thr.on));
    GHOST = C.rollout(runOpts(false));
    var ter = RUN.terrain, key = [ter.zE, ter.z1, ter.z2, st.params.Hb, st.params.D].join(",");
    if (key !== terKey) { terKey = key; buildTerrain(ter); }
    renderResult(); renderBudget(); buildChart(); drawTicks();
  }
  var OUT = { safe: "Tiếp đất an toàn", hard: "Tiếp đất cứng", flip: "Lật" };
  function renderResult() {
    var s = RUN.summary, g = GHOST.summary, td = s.touch;
    function row(k, v) { return "<span>" + k + "</span><b>" + v + "</b>"; }
    $("result").innerHTML =
      row("kết quả", '<span class="badge ' + s.outcome + '">' + OUT[s.outcome] + "</span>") +
      row("không vòi phun (bóng mờ)", '<span class="badge ' + g.outcome + '">' + OUT[g.outcome] + "</span>") +
      row("thời gian bay", fmt(s.airtime, 2) + " s") +
      row("nghiêng lúc chạm đất", td ? fmt(td.tilt, 0) + "° (ngưỡng " + C.SAFE.tilt + "°)" : "–") +
      row("vận tốc rơi lúc chạm", td ? fmt(td.sink, 1) + " m/s (ngưỡng " + C.SAFE.sink + ")" : "–") +
      row("nghiêng lớn nhất", fmt(s.maxTilt, 0) + "°") +
      row("IMU phát hiện rơi tự do", s.detectLatency == null ? "–" : s.detectLatency >= 0 ? "trễ " + fmt(s.detectLatency * 1000, 0) + " ms sau khi rời đất"
          : "sớm " + fmt(-s.detectLatency * 1000, 0) + " ms (bánh đã hết tải, chưa rời hẳn)") +
      row("sai số tư thế lúc chạm", s.estErrTouch == null ? "–" : fmt(s.estErrTouch, 1) + "°") +
      row("khí nén đã dùng", fmt(s.fuelUsed, 1) + " / " + fmt(st.thr.fuel, 1) + " kg") +
      row("khối lượng lúc đầu", fmt(s.mass0, 0) + " kg") +
      row("gia tốc đỉnh", fmt(s.peakG, 1) + " g");
  }
  function drawTicks() {
    var COL = { launch: "blue", detect: "teal", warn: "gold", touch: "green", roof: "red", side: "gold", fuel: "grey", hit: "purple" };
    $("ticks").innerHTML = RUN.events.map(function (e) {
      return '<i style="left:' + (100 * e.t / T_END) + "%;background:var(--" + (COL[e.kind] || "grey") + ')" title="' + fmt(e.t, 2) + " s · " + e.text + '"></i>'; }).join("");
  }

  // ================================================================== HUD: rear view and the tilt chart
  var chartCache = document.createElement("canvas");
  function buildChart() {
    var cv = $("chart"); chartCache.width = cv.width; chartCache.height = cv.height;
    var x = chartCache.getContext("2d"), W = cv.width, H = cv.height, L = 44, B = 26, top = 10;
    var R = RUN.R, G = GHOST.R, sx = function (t) { return L + t / T_END * (W - L - 8); }, sy = function (d) { return top + (1 - d / 180) * (H - top - B); };
    x.clearRect(0, 0, W, H);
    // airborne (truth) bands
    x.fillStyle = "rgba(31,143,181,.16)";
    for (var f = 0; f < NF; f++) if (!R.contact[f]) x.fillRect(sx(f * FDT), top, sx(FDT) - L + 1, H - top - B);
    // firing count bars
    x.fillStyle = css("gold");
    for (var f2 = 0; f2 < NF; f2++) { var n = pop(R.FIRE[f2]); if (n) { var h = n / 8 * 34; x.fillRect(sx(f2 * FDT), H - B - h, Math.max(1, sx(FDT) - L), h); } }
    // grid
    x.strokeStyle = css("line"); x.fillStyle = css("ink-3"); x.font = "600 18px " + css("f-mono"); x.lineWidth = 1;
    [0, 45, 90, 135, 180].forEach(function (d) { x.beginPath(); x.moveTo(L, sy(d)); x.lineTo(W - 8, sy(d)); x.stroke(); x.textAlign = "right"; x.fillText(d + "°", L - 6, sy(d) + 6); });
    for (var s = 0; s <= T_END; s++) { x.textAlign = "center"; x.fillText(s + "s", sx(s), H - 4); }
    x.setLineDash([6, 5]); x.strokeStyle = css("green"); x.beginPath(); x.moveTo(L, sy(C.SAFE.tilt)); x.lineTo(W - 8, sy(C.SAFE.tilt)); x.stroke(); x.setLineDash([]);
    function line(arr, col, w, dash) { x.strokeStyle = col; x.lineWidth = w; x.setLineDash(dash || []); x.beginPath(); for (var i = 0; i < NF; i++) { var px = sx(i * FDT), py = sy(arr[i]); if (i) x.lineTo(px, py); else x.moveTo(px, py); } x.stroke(); x.setLineDash([]); }
    line(G.tilt, css("grey"), 2.5);
    line(R.tiltE, css("blue"), 2.5, [8, 6]);
    line(R.tilt, css("teal"), 3.5);
  }
  function drawChart(t) {
    var cv = $("chart"), x = cv.getContext("2d"), W = cv.width, H = cv.height, L = 44;
    x.clearRect(0, 0, W, H); x.drawImage(chartCache, 0, 0);
    var px = L + t / T_END * (W - L - 8); x.strokeStyle = css("ink"); x.lineWidth = 2; x.beginPath(); x.moveTo(px, 6); x.lineTo(px, H - 22); x.stroke();
  }
  function drawAtt(f) {
    var cv = $("att"), x = cv.getContext("2d"), W = cv.width, H = cv.height, cx = W / 2, cy = H * 0.56;
    x.clearRect(0, 0, W, H);
    x.fillStyle = "rgba(147,173,121,.45)"; x.fillRect(0, cy + 52, W, H - cy - 52);
    x.strokeStyle = css("ink-3"); x.lineWidth = 2; x.beginPath(); x.moveTo(0, cy + 52); x.lineTo(W, cy + 52); x.stroke();
    function silhouette(roll, fill, stroke, dash) {
      x.save(); x.translate(cx, cy); x.rotate(roll * Math.PI / 180); x.setLineDash(dash || []);
      x.beginPath(); x.rect(-76, -10, 152, 50); x.moveTo(-58, -10); x.lineTo(-46, -52); x.lineTo(46, -52); x.lineTo(58, -10);
      if (fill) { x.fillStyle = fill; x.fill(); } if (stroke) { x.strokeStyle = stroke; x.lineWidth = 4; x.stroke(); }
      if (fill) { x.fillStyle = "#23282e"; x.fillRect(-70, 40, 28, 14); x.fillRect(42, 40, 28, 14); }
      x.restore();
    }
    var R = RUN.R;
    silhouette(R.roll[f], css("teal"), null);
    silhouette(R.rollE[f], null, css("blue"), [10, 8]);
    x.fillStyle = css("ink"); x.font = "700 22px " + css("f-mono"); x.textAlign = "left";
    x.fillText("lăn " + fmt(R.roll[f], 0) + "°", 12, 30);
    x.fillStyle = css("blue"); x.font = "600 18px " + css("f-mono"); x.fillText("ước lượng " + fmt(R.rollE[f], 0) + "°", 12, 54);
    x.fillStyle = css("ink-3"); x.font = "600 16px " + css("f-mono"); x.fillText("nhìn từ sau xe", 12, H - 12);
  }
  function live(f) {
    var R = RUN.R, n = pop(R.FIRE[f]), air = R.air[f];
    $("live").innerHTML = "<span>t <b>" + fmt(f * FDT, 2) + " s</b></span><span>tốc độ <b>" + fmt(R.speed[f] * 3.6, 0) + " km/h</b></span>" +
      "<span>nghiêng <b>" + fmt(R.tilt[f], 0) + "°</b> / ước lượng <b>" + fmt(R.tiltE[f], 0) + "°</b></span>" +
      "<span>|f| IMU <b>" + fmt(R.fmag[f], 2) + " g</b></span><span>bộ phát hiện <b class=\"" + (air ? "c-gold" : "") + "\">" + (air ? "ĐANG BAY" : "mặt đất") + "</b></span>" +
      (air ? "<span>chạm đất sau <b>" + fmt(R.tti[f], 2) + " s</b></span>" : "") +
      "<span>vòi đang phun <b>" + n + "</b></span><span>khí còn <b>" + fmt(R.fuel[f], 1) + " kg</b></span>";
  }

  // ================================================================== labels
  var LBL = [], pool = [], vt = new T.Vector3();
  function label(p, y, text, cls) { LBL.push([p.x, p.y + y, p.z, text, cls]); }
  function flushLabels() {
    var w = stage.clientWidth, h = stage.clientHeight;
    while (pool.length < LBL.length) { var d = document.createElement("div"); labels.appendChild(d); pool.push(d); }
    pool.forEach(function (d, i) {
      var L = LBL[i]; if (!L) { d.hidden = true; return; }
      vt.set(L[0], L[1], L[2]).project(camera);
      if (vt.z > 1 || Math.abs(vt.x) > 1.1 || Math.abs(vt.y) > 1.1) { d.hidden = true; return; }
      d.hidden = false; d.className = "lbl " + (L[4] || ""); if (d.textContent !== L[3]) d.textContent = L[3];
      d.style.left = ((vt.x + 1) / 2 * w).toFixed(1) + "px"; d.style.top = ((1 - vt.y) / 2 * h).toFixed(1) + "px";
    });
    LBL = [];
  }

  // ================================================================== transport
  var playBtn = $("play"), timeEl = $("time"), toastT = 0;
  timeEl.max = T_END;
  function syncPlay() { playBtn.textContent = st.playing ? "❚❚" : "▶"; }
  playBtn.addEventListener("click", function () { st.playing = !st.playing; if (st.playing && st.t >= T_END - 0.01) st.t = 0; syncPlay(); });
  $("restart").addEventListener("click", function () { st.t = 0; st.playing = true; syncPlay(); });
  $("speed").addEventListener("click", function () { var s = [0.1, 0.25, 0.5, 1]; st.speed = s[(s.indexOf(st.speed) + 1) % s.length]; this.textContent = fmt(st.speed, st.speed < 0.25 ? 1 : st.speed < 1 ? 2 : 0) + "×"; });
  timeEl.addEventListener("input", function () { st.t = +timeEl.value; });
  $("camBtn").addEventListener("click", function () { setCam((st.cam + 1) % CAMS.length); });
  document.addEventListener("keydown", function (e) {
    if (e.target && (e.target.tagName === "INPUT" && e.target.type !== "checkbox" || e.target.tagName === "SELECT")) return;
    if (e.key === " ") { st.playing = !st.playing; syncPlay(); e.preventDefault(); }
    else if (e.key === "r" || e.key === "R") randomize();
  });
  function toast(msg) { var el = $("toast"); el.textContent = msg; el.hidden = false; toastT = performance.now() + 2600; }

  // ================================================================== CSV export (data pillar: every run is a reproducible log)
  function download(name, text) {
    var a = document.createElement("a"); a.href = URL.createObjectURL(new Blob([text], { type: "text/csv" })); a.download = name;
    document.body.appendChild(a); a.click(); setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 500);
  }
  $("bCSV").addEventListener("click", function () {
    var R = RUN.R, rows = ["t,x,y,z,qw,qx,qy,qz,qe_w,qe_x,qe_y,qe_z,tilt_deg,tilt_est_deg,roll_deg,roll_est_deg,imu_f_g,air_detected,contact_truth,tti_s,fuel_kg,speed_ms,fire_mask,n_fire"];
    for (var f = 0; f < NF; f++) {
      var r = [fmtc(f * FDT, 2)];
      for (var i = 0; i < 3; i++) r.push(fmtc(R.P[f * 3 + i], 4));
      for (var j = 0; j < 4; j++) r.push(fmtc(R.Q[f * 4 + j], 5)); for (var k = 0; k < 4; k++) r.push(fmtc(R.QE[f * 4 + k], 5));
      r.push(fmtc(R.tilt[f], 2), fmtc(R.tiltE[f], 2), fmtc(R.roll[f], 2), fmtc(R.rollE[f], 2), fmtc(R.fmag[f], 3), R.air[f], R.contact[f], fmtc(R.tti[f], 3), fmtc(R.fuel[f], 3), fmtc(R.speed[f], 3), R.FIRE[f], pop(R.FIRE[f]));
      rows.push(r.join(","));
    }
    var meta = "# scenario=" + st.sc + " seed=" + st.seed + " params=" + JSON.stringify(st.params).replace(/,/g, ";") + " est=" + st.est.mode + " thrusters=" + st.thr.on;
    download("carflip_" + st.sc + "_seed" + st.seed + ".csv", meta + "\n" + rows.join("\n"));
  });
  function fmtc(v, d) { return (+v).toFixed(d); }

  // ================================================================== Monte Carlo (evidence pillar)
  var MC = null;
  $("bMC").addEventListener("click", function () {
    var N = +$("mcN").value, rows = [], s = 1, btn = this; btn.disabled = true; $("bMCcsv").disabled = true;
    var base = { thr: st.thr, est: st.est, ctrl: st.ctrl, veh: st.veh }, t0 = performance.now();
    (function chunk() {
      var end = Math.min(N, s + 3);
      for (; s <= end; s++) rows.push(C.evalSeed(s, base));
      $("mcProg").textContent = "đang chạy " + rows.length + " / " + N + " kịch bản × 4 cấu hình…";
      if (s <= N) { setTimeout(chunk, 0); return; }
      MC = { rows: rows, sum: C.summarize(rows), base: JSON.parse(JSON.stringify(base)) };
      LAB.sel = -1; $("lab").hidden = false; labRender();
      $("mcProg").textContent = N + " kịch bản × 4 cấu hình trong " + fmt((performance.now() - t0) / 1000, 1) + " s (seed 1…" + N + ", tái lập được).";
      renderMC(); btn.disabled = false; $("bMCcsv").disabled = false;
    })();
  });
  function renderMC() {
    var pct = function (x) { return fmt(100 * x, 0) + "%"; };
    $("mcOut").innerHTML = '<table class="mc"><tr><th>Cấu hình</th><th>An toàn</th><th>CI 95%</th><th>Cứu được*</th><th>Sai số tư thế lúc chạm</th></tr>' +
      MC.sum.map(function (r) {
        return "<tr><td>" + r.name + '</td><td class="n">' + r.safe + "/" + r.n + '</td><td class="n">' + pct(r.ci[0]) + "–" + pct(r.ci[1]) + '</td><td class="n">' +
          (r.key === "none" ? "–" : r.rescue[0] + "/" + r.rescue[1]) + '</td><td class="n">' + (r.estErr == null ? "–" : fmt(r.estErr, 1) + "°") + "</td></tr>"; }).join("") +
      '</table><p class="note">* trong các kịch bản mà xe không vòi phun không tiếp đất an toàn, cấu hình này cứu được bao nhiêu. Cùng seed cho mọi cấu hình nên đây là so sánh theo cặp.</p>';
  }
  $("bMCcsv").addEventListener("click", function () {
    if (!MC) return;
    var keys = Object.keys(C.RANGES), head = ["seed"].concat(keys);
    C.CONFIGS.forEach(function (c) { ["outcome", "tilt", "sink", "estErr", "air", "fuel"].forEach(function (k) { head.push(c.key + "_" + k); }); });
    var lines = [head.join(",")].concat(MC.rows.map(function (r) {
      var a = [r.seed].concat(keys.map(function (k) { return r.params[k]; }));
      C.CONFIGS.forEach(function (c) { var o = r.out[c.key]; a.push(o.outcome, o.tilt == null ? "" : o.tilt.toFixed(1), o.sink == null ? "" : o.sink.toFixed(2), o.estErr == null ? "" : o.estErr.toFixed(2), o.air.toFixed(2), o.fuel.toFixed(2)); });
      return a.join(","); }));
    download("carflip_montecarlo_N" + MC.rows.length + ".csv", lines.join("\n"));
  });

  // ================================================================== lab: every Monte Carlo scene as a dot; click to replay it in 3D
  var LAB = { cfg: "gated", sel: -1, hover: -1, xi: "air", yi: "maxTilt", map: null };
  var SEV = { safe: ["an toàn", "green"], hard: ["tiếp đất cứng: nguy cơ chấn thương nhẹ", "#c9a400"], side: ["lật nghiêng", "#e07b2e"], roof: ["lật ngửa, nóc chạm đất: nguy cơ chấn thương nặng", "red"] };
  var CMP = { saved: ["được vòi phun cứu", "teal"], still: ["vẫn không cứu được", "red"], fine: ["an toàn sẵn (không cần vòi phun)", "grey"], worse: ["tệ hơn khi có vòi phun", "purple"] };
  var RANK = { safe: 0, hard: 1, side: 2, roof: 3 };
  var LAB_CFGS = [["none", "Không vòi"], ["gated", "Có cổng"], ["naive", "Ngây thơ"], ["truth", "Hoàn hảo"], ["cmp", "So sánh"]];
  var AXES = {
    air: ["thời gian bay (không vòi phun)", function (r) { return r.out.none.air; }, " s"],
    maxTilt: ["nghiêng lớn nhất (không vòi phun)", function (r) { return r.out.none.maxTilt; }, "°"],
    v0: ["tốc độ", function (r) { return r.params.v0 * 3.6; }, " km/h"], psi: ["góc lệch khỏi đường", function (r) { return r.params.psi; }, "°"],
    Hb: ["cao gờ đất", function (r) { return r.params.Hb; }, " m"], Lb: ["rộng mặt gờ", function (r) { return r.params.Lb; }, " m"],
    D: ["cao taluy", function (r) { return r.params.D; }, " m"],
    fuel: ["khí đã dùng (có cổng)", function (r) { return r.out.gated.fuel; }, " kg"]
  };
  function colOf(c) { return c.charAt(0) === "#" ? c : css(c); }
  function labCat(r) {
    if (LAB.cfg !== "cmp") { var s = SEV[r.out[LAB.cfg].sev]; return { key: r.out[LAB.cfg].sev, name: s[0], col: s[1] }; }
    var a = RANK[r.out.none.sev], b = RANK[r.out.gated.sev];
    var k = a === 0 && b === 0 ? "fine" : b === 0 ? "saved" : b > a ? "worse" : "still";
    return { key: k, name: CMP[k][0], col: CMP[k][1] };
  }
  function labRender() {
    if (!MC) return;
    $("labCfg").innerHTML = LAB_CFGS.map(function (c) { return '<button type="button" data-c="' + c[0] + '" aria-pressed="' + (LAB.cfg === c[0]) + '">' + c[1] + "</button>"; }).join("");
    $("labCfg").querySelectorAll("button").forEach(function (b) { b.addEventListener("click", function () { LAB.cfg = b.dataset.c; labRender(); }); });
    var opts = Object.keys(AXES).map(function (k) { return '<option value="' + k + '">' + AXES[k][0] + "</option>"; }).join("");
    $("labX").innerHTML = opts; $("labY").innerHTML = opts; $("labX").value = LAB.xi; $("labY").value = LAB.yi;
    var cnt = {}; MC.rows.forEach(function (r) { var c = labCat(r); cnt[c.key] = (cnt[c.key] || 0) + 1; });
    var src = LAB.cfg === "cmp" ? CMP : SEV;
    $("labLegend").innerHTML = Object.keys(src).map(function (k) { return '<span><i style="background:' + colOf(src[k][1]) + '"></i>' + src[k][0] + " <b>" + (cnt[k] || 0) + "</b></span>"; }).join("");
    labPlot();
  }
  function labPlot() {
    var cv = $("labPlot"); if (!cv || !MC) return;
    var x = cv.getContext("2d"), W = cv.width, H = cv.height, L = 40, B = 34, top = 12, rows = MC.rows;
    var fx = AXES[LAB.xi][1], fy = AXES[LAB.yi][1], xv = rows.map(fx), yv = rows.map(fy);
    var x0 = Math.min.apply(null, xv), x1 = Math.max.apply(null, xv), y0 = Math.min.apply(null, yv), y1 = Math.max.apply(null, yv);
    if (x1 - x0 < 1e-9) x1 = x0 + 1; if (y1 - y0 < 1e-9) y1 = y0 + 1;
    var px = (x1 - x0) * 0.05, py = (y1 - y0) * 0.05; x0 -= px; x1 += px; y0 -= py; y1 += py;
    var sx = function (v) { return L + (v - x0) / (x1 - x0) * (W - L - 10); }, sy = function (v) { return top + (1 - (v - y0) / (y1 - y0)) * (H - top - B); };
    LAB.map = { sx: sx, sy: sy, xv: xv, yv: yv };
    x.clearRect(0, 0, W, H);
    x.strokeStyle = css("line"); x.lineWidth = 1.5; x.beginPath(); x.moveTo(L, top); x.lineTo(L, H - B); x.lineTo(W - 10, H - B); x.stroke();
    x.fillStyle = css("ink-3"); x.font = "600 14px " + css("f-mono");
    var dx = LAB.xi === "maxTilt" || LAB.xi === "v0" || LAB.xi === "psi" ? 0 : 1, dy = LAB.yi === "maxTilt" || LAB.yi === "v0" || LAB.yi === "psi" ? 0 : 1;
    x.textAlign = "left"; x.fillText(fmt(x0 + px, dx), L, H - 16); x.textAlign = "right"; x.fillText(AXES[LAB.xi][0] + "  " + fmt(x1 - px, dx) + AXES[LAB.xi][2], W - 10, H - 16);
    x.save(); x.translate(15, H - B); x.rotate(-Math.PI / 2); x.textAlign = "left"; x.fillText(fmt(y0 + py, dy), 0, 0); x.textAlign = "right";
    x.fillText(AXES[LAB.yi][0] + "  " + fmt(y1 - py, dy) + AXES[LAB.yi][2], H - B - top, 0); x.restore();
    if (LAB.yi === "maxTilt") { x.setLineDash([5, 5]); x.strokeStyle = css("green"); x.beginPath(); x.moveTo(L, sy(C.SAFE.tilt)); x.lineTo(W - 10, sy(C.SAFE.tilt)); x.stroke(); x.setLineDash([]); }
    // worst outcomes drawn last so they stay visible
    var order = rows.map(function (_, i) { return i; }), cats = rows.map(labCat);
    order.sort(function (a, b) { return (RANK[cats[a].key] || 0) - (RANK[cats[b].key] || 0); });
    order.forEach(function (i) {
      x.globalAlpha = 0.85; x.fillStyle = colOf(cats[i].col);
      x.beginPath(); x.arc(sx(xv[i]), sy(yv[i]), 6, 0, 7); x.fill();
    });
    x.globalAlpha = 1;
    [[LAB.hover, css("ink-3"), 2], [LAB.sel, css("ink"), 3]].forEach(function (h) {
      if (h[0] < 0) return; x.strokeStyle = h[1]; x.lineWidth = h[2]; x.beginPath(); x.arc(sx(xv[h[0]]), sy(yv[h[0]]), 11, 0, 7); x.stroke(); });
  }
  function labPick(e) {
    if (!MC || !LAB.map) return -1;
    var cv = $("labPlot"), r = cv.getBoundingClientRect(), px = (e.clientX - r.left) * cv.width / r.width, py = (e.clientY - r.top) * cv.height / r.height, best = -1, bd = 1e9;
    MC.rows.forEach(function (_, i) { var d = Math.pow(LAB.map.sx(LAB.map.xv[i]) - px, 2) + Math.pow(LAB.map.sy(LAB.map.yv[i]) - py, 2); if (d < bd) { bd = d; best = i; } });
    return bd < 18 * 18 ? best : -1;
  }
  function labInfo(i) {
    var r = MC.rows[i], p = r.params, c = labCat(r);
    function o(k) { return SEV[r.out[k].sev][0].split(":")[0]; }
    $("labSel").innerHTML = "<b>seed " + r.seed + "</b> · " + fmt(p.v0 * 3.6, 0) + " km/h, lệch " + fmt(p.psi, 1) + "°, gờ " + fmt(p.Hb, 2) + " m, taluy " + fmt(p.D, 1) + " m<br>" +
      'không vòi: <b>' + o("none") + "</b> · có cổng: <b>" + o("gated") + "</b> · ngây thơ: <b>" + o("naive") + "</b> · hoàn hảo: <b>" + o("truth") + "</b>" +
      (i === LAB.sel ? '<br><span class="c-teal">đang chạy trong cảnh 3D (' + c.name + ")</span>" : "");
  }
  function labLoad(i) {
    var r = MC.rows[i], cfg = LAB.cfg === "cmp" ? "gated" : LAB.cfg, c = C.CONFIGS.filter(function (q) { return q.key === cfg; })[0];
    st.sc = "berm"; st.params = Object.assign({}, C.SCEN.berm.params, r.params); st.seed = r.seed;
    st.thr = Object.assign({}, MC.base.thr, c.thr); st.est = Object.assign({}, MC.base.est, c.est); st.ctrl = Object.assign({}, MC.base.ctrl); st.veh = Object.assign({}, MC.base.veh); $("tReg").checked = st.thr.reg !== false;
    $("tThr").checked = st.thr.on; $("tRetro").checked = st.ctrl.retro !== false;
    renderScenario(); renderSystem(); renderVehicle(); rerun();
    var tl = r.out[cfg].tLaunch; st.t = tl == null ? 0 : Math.max(0, tl - 0.8); st.playing = true; syncPlay();
    LAB.sel = i; labInfo(i); labPlot();
  }
  $("labWide").addEventListener("click", function () { var on = document.body.classList.toggle("wide"); this.textContent = on ? "Thu nhỏ" : "Mở rộng"; resize(); });
  $("labX").addEventListener("change", function () { LAB.xi = this.value; labPlot(); });
  $("labY").addEventListener("change", function () { LAB.yi = this.value; labPlot(); });
  $("labPlot").addEventListener("mousemove", function (e) { var i = labPick(e); if (i !== LAB.hover) { LAB.hover = i; labPlot(); if (i >= 0) labInfo(i); } });
  $("labPlot").addEventListener("mouseleave", function () { LAB.hover = -1; labPlot(); if (LAB.sel >= 0) labInfo(LAB.sel); });
  $("labPlot").addEventListener("click", function (e) { var i = labPick(e); if (i >= 0) labLoad(i); });

  // ================================================================== scripted demo (index.html#demo): captions + the whole story, hands-free
  var DEMO = { on: false, auto: true, step: 0, timer: 0, timer2: 0, poll: 0 };
  function demoReset(seed, params, mode) {
    st.sc = "berm"; st.params = Object.assign({}, C.SCEN.berm.params, params || {}); st.seed = seed || 1;
    st.thr = Object.assign({}, C.THR_DEFAULTS); st.ctrl = Object.assign({}, C.CTRL_DEFAULTS); st.veh = Object.assign({}, C.VEH_DEFAULTS);
    st.est = Object.assign({}, C.EST_DEFAULTS, { mode: mode || "gated" }); st.ghost = true; st.estBox = mode === "naive";
    $("tThr").checked = true; $("tRetro").checked = true; $("tGhost").checked = true; $("tEstBox").checked = st.estBox;
    renderScenario(); renderSystem(); renderVehicle(); rerun();
  }
  function demoPlay(from, speed) { st.t = from || 0; st.speed = speed || 1; $("speed").textContent = fmt(st.speed, st.speed < 1 ? 2 : 0) + "×"; st.playing = true; syncPlay(); }
  function demoPick(kind) {   // first Monte Carlo dot of a comparison category
    var cfg = LAB.cfg; LAB.cfg = "cmp"; var hit = -1;
    MC.rows.forEach(function (r, i) { if (hit < 0 && labCat(r).key === kind) hit = i; });
    LAB.cfg = cfg; return hit;
  }
  var DEMO_STEPS = [
    { cap: "Bài toán: xe lao qua gờ đất ở mép đường đắp cao, bay khỏi mặt đường và lật trên không. Bóng mờ là cùng kịch bản nhưng không có vòi phun.", ms: 12000,
      run: function () { demoReset(1); setCam(0); demoPlay(0, 0.5); } },
    { cap: "Nhìn từ bên hông: không vòi phun thì xe lật ngửa, nóc chạm đất. Có 12 vòi phun khí lạnh, xe xoay về bánh xuống trước khi chạm đất.", ms: 11000,
      run: function () { setCam(1); demoPlay(0.7, 0.35); } },
    { cap: "Xe biết mình đang nghiêng thế nào nhờ gyro và gia tốc kế. Khi rơi tự do gia tốc kế đọc ≈ 0 g, nên bộ lọc có cổng chỉ tin gyro lúc bay (khung xanh trùng thân xe).", ms: 12000,
      run: function () { demoReset(38, { v0: 22.06, psi: 19.55, Hb: 0.56, Lb: 1.07, D: 5.56 }, "gated"); $("tEstBox").checked = st.estBox = true; setCam(0); demoPlay(0.8, 0.4); } },
    { cap: "CA LỖI: bộ lọc ngây thơ vẫn tin gia tốc kế khi đang bay. Khung xanh (xe tự tưởng mình nghiêng thế nào) lệch khỏi thân xe, vòi phun xoay sai, tiếp đất cứng.", ms: 13000,
      run: function () { demoReset(38, { v0: 22.06, psi: 19.55, Hb: 0.56, Lb: 1.07, D: 5.56 }, "naive"); setCam(0); demoPlay(0.8, 0.4); } },
    { cap: "Đo trên 50 kịch bản ngẫu nhiên × 4 cấu hình (cùng seed, so sánh theo cặp)…", wait: function () { return !!MC && MC.rows.length >= 50; }, ms: 60000,
      run: function () { demoReset(1); $("mcN").value = "50"; $("bMC").click(); $("lab").scrollIntoView({ block: "nearest" }); } },
    { cap: function () { var g = MC.sum[1], n = MC.sum[0], v = MC.sum[2]; return "Số đo: tiếp đất an toàn " + g.safe + "/" + g.n + " (không vòi phun " + n.safe + "/" + n.n + ", bộ lọc ngây thơ " + v.safe + "/" + v.n + "). Mỗi chấm là một kịch bản."; }, ms: 8000,
      run: function () { LAB.cfg = "gated"; labRender(); $("labPlot").scrollIntoView({ block: "center" }); } },
    { cap: "Chế độ so sánh: xanh ngọc = được vòi phun cứu, đỏ = vẫn không cứu được. Bấm một chấm đỏ để chạy lại đúng kịch bản đó.", ms: 12000,
      run: function () { LAB.cfg = "cmp"; labRender(); var i = demoPick("still"); if (i < 0) i = demoPick("saved"); if (i >= 0) { labLoad(i); demoPlay(Math.max(0, st.t), 0.5); } $("labPlot").scrollIntoView({ block: "center" }); } },
    { cap: "Giới hạn: thông số xe và vòi phun là giả định. Kịch bản bay quá ngắn (lật do vấp) thì vòi phun không kịp. Phần dùng ngay được: phát hiện rơi tự do và lật từ IMU.", ms: 9000,
      run: function () { $("labPlot").scrollIntoView({ block: "nearest" }); } }
  ];
  function demoCap(text, k) {
    var el = $("demoCap"); el.hidden = false;
    el.innerHTML = '<span class="step">' + (k + 1) + " / " + DEMO_STEPS.length + "</span>" + text +
      '<div class="dnav"><button type="button" id="dPrev"' + (k === 0 ? " disabled" : "") + '>← Cảnh trước</button><button type="button" id="dAuto">' + (DEMO.auto ? "❚❚ Tạm dừng tự chuyển" : "▶ Tự chuyển tiếp") + '</button>' +
      '<button type="button" id="dNext"' + (k === DEMO_STEPS.length - 1 ? " disabled" : "") + '>Cảnh sau →</button><button type="button" id="demoStop">Thoát (Esc)</button></div>';
    $("dPrev").onclick = function () { demoGo(DEMO.step - 1); }; $("dNext").onclick = function () { demoGo(DEMO.step + 1); };
    $("dAuto").onclick = function () { DEMO.auto = !DEMO.auto; if (DEMO.auto) demoSchedule(DEMO.step); else { clearTimeout(DEMO.timer); clearInterval(DEMO.poll); } demoCap(typeof DEMO_STEPS[k].cap === "function" ? DEMO_STEPS[k].cap() : DEMO_STEPS[k].cap, k); };
    $("demoStop").onclick = demoStop;
  }
  function demoStop() { DEMO.on = false; clearTimeout(DEMO.timer); clearTimeout(DEMO.timer2); clearInterval(DEMO.poll); $("demoCap").hidden = true; }
  function demoSchedule(k) {
    clearTimeout(DEMO.timer); clearInterval(DEMO.poll);
    var S = DEMO_STEPS[k], t0 = performance.now(), next = function () { if (DEMO.on && DEMO.auto && DEMO.step === k) demoGo(k + 1); };
    if (S.wait) DEMO.poll = setInterval(function () { if (S.wait() || performance.now() - t0 > S.ms) { clearInterval(DEMO.poll); DEMO.timer = setTimeout(next, 1500); } }, 300);
    else DEMO.timer = setTimeout(next, S.ms);
  }
  function demoGo(k) {
    if (!DEMO.on) return;
    clearTimeout(DEMO.timer); clearTimeout(DEMO.timer2); clearInterval(DEMO.poll);
    if (k >= DEMO_STEPS.length) { demoStop(); return; }
    if (k < 0) k = 0;
    if (k > 4 && !(MC && MC.rows.length >= 50)) k = 4;           // the later steps need the Monte Carlo results
    var S = DEMO_STEPS[k]; DEMO.step = k; S.run();
    demoCap(typeof S.cap === "function" ? "đang chạy…" : S.cap, k);
    if (typeof S.cap === "function") DEMO.timer2 = setTimeout(function () { if (DEMO.on && DEMO.step === k) demoCap(S.cap(), k); }, 600);
    if (DEMO.auto) demoSchedule(k);
  }
  function demoStart() { DEMO.on = true; document.body.classList.add("wide"); $("labWide").textContent = "Thu nhỏ"; resize(); demoGo(0); }
  document.addEventListener("keydown", function (e) {
    if (!DEMO.on) return;
    if (e.key === "Escape") demoStop(); else if (e.key === "ArrowRight") demoGo(DEMO.step + 1); else if (e.key === "ArrowLeft") demoGo(DEMO.step - 1);
  });

  // ================================================================== frame loop
  function resize() { var w = stage.clientWidth, h = stage.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix(); }
  window.addEventListener("resize", resize);
  var last = performance.now(), EVLBL = { launch: "rời mặt đất", detect: "IMU: rơi tự do", touch: "tiếp đất", roof: "nóc chạm đất", fuel: "hết khí", hit: "bị húc" };
  function frame(now) {
    var dtR = clamp((now - last) / 1000, 0, 0.05); last = now;
    var prevT = st.t;
    if (st.playing) { st.t += dtR * st.speed; if (st.t >= T_END) { st.t = T_END; st.playing = false; syncPlay(); } }
    var t = st.t; if (document.activeElement !== timeEl) timeEl.value = t;
    $("tread").textContent = fmt(t, 2) + " s";
    var fu = poseOf(RUN.R, "Q", t, car), f = fu.f, mask = RUN.R.FIRE[f];
    car.userData.plumes.forEach(function (p, i) { p.visible = !!(mask & (1 << i)); if (p.visible) p.scale.set(1, 0.85 + 0.3 * Math.random(), 1); });
    ghostCar.visible = st.ghost; if (st.ghost) poseOf(GHOST.R, "Q", t, ghostCar);
    estBox.visible = st.estBox; if (st.estBox) poseOf(RUN.R, "QE", t, estBox);
    // camera follows the car
    var k = 1 - Math.exp(-dtR * 5), cp = car.position;
    rig.tx += (cp.x - rig.tx) * k; rig.ty += (cp.y - rig.ty) * k; rig.tz += (cp.z - rig.tz) * k;
    var c = Math.cos(rig.pitch);
    camera.position.set(rig.tx + rig.dist * c * Math.cos(rig.yaw), rig.ty + rig.dist * Math.sin(rig.pitch), rig.tz + rig.dist * c * Math.sin(rig.yaw));
    camera.lookAt(rig.tx, rig.ty + 0.5, rig.tz);
    sun.position.set(rig.tx - 30, rig.ty + 60, rig.tz - 40); sun.target.position.set(rig.tx, rig.ty, rig.tz);
    // labels and event toasts
    label(cp, 2.2, "xe ta · " + fmt(RUN.R.speed[f] * 3.6, 0) + " km/h" + (pop(mask) ? " · phun " + pop(mask) : ""), "teal");
    if (st.ghost) label(ghostCar.position, 2.2, "không vòi phun", "grey");
    RUN.events.forEach(function (e) {
      if (t >= e.t && t < e.t + 1.2 && EVLBL[e.kind]) label(cp, 3.3, EVLBL[e.kind] + (e.kind === "touch" ? " · " + e.text.replace(/^.*\(|\)$/g, "") : ""), e.kind === "roof" ? "red" : e.kind === "detect" ? "gold" : "teal");
      if (st.playing && prevT < e.t && t >= e.t && (e.kind === "roof" || e.kind === "fuel")) toast(fmt(e.t, 2) + " s · " + e.text);
    });
    if (toastT && performance.now() > toastT) { $("toast").hidden = true; toastT = 0; }
    drawChart(t); drawAtt(f); live(f);
    renderer.render(scene, camera); flushLabels();
    requestAnimationFrame(frame);
  }

  // ================================================================== go
  renderScenario(); renderSystem(); renderVehicle(); rerun(); resize(); setCam(0); syncPlay();
  var p0 = RUN.R.P; rig.tx = p0[0]; rig.ty = p0[1]; rig.tz = p0[2];
  requestAnimationFrame(frame);
  if (location.hash === "#demo") setTimeout(demoStart, 400);
  if (/^#mc(\d+)?$/.test(location.hash)) { var mN = location.hash.slice(3); if (mN) { var op = document.createElement("option"); op.textContent = mN; $("mcN").appendChild(op); $("mcN").value = mN; } $("bMC").click(); }   // index.html#mc100 runs the evaluation on load
  window.__flip = { st: st, get RUN() { return RUN; }, rerun: rerun };
})();
