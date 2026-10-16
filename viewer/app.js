/* app.js — "Vòng lặp mô phỏng": three.js view of the SimCore scenario, eight chapters along the AV simulation loop. */
(function () {
  "use strict";
  var S = window.SimCore, T = window.THREE;
  if (!S || !T) { document.getElementById("body").innerHTML = "<p>Không tải được thư viện 3D. Mở lại trang khi có mạng.</p>"; return; }
  var DT = S.DT, T_END = S.T_END, NF = S.NF;
  var REDUCED = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  var $ = function (id) { return document.getElementById(id); };
  function rng(seed) { var a = seed >>> 0; return function () { a = (a + 0x6D2B79F5) >>> 0; var t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
  function hash(a, b, c) { var h = Math.imul(a | 0, 374761393) ^ Math.imul(b | 0, 668265263) ^ Math.imul(c | 0, 2246822519); h = Math.imul(h ^ (h >>> 13), 1274126177); return ((h ^ (h >>> 16)) >>> 0) / 4294967296; }
  function gauss(a, b, c) { var u = Math.max(1e-9, hash(a, b, c)), v = hash(a + 7, b + 3, c + 11); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  function fmt(x, d) { return x.toFixed(d == null ? 1 : d).replace(".", ","); }
  function ease(u) { u = Math.max(0, Math.min(1, u)); return u < .5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2; }
  function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

  // ================================================================== palette (CSS tokens -> three colours)
  var KEYS = ["sc-bg", "sc-fog", "sc-ground", "sc-road", "sc-mark", "sc-walk", "sc-bldg", "sc-bldg2", "sc-edge", "sc-glass", "sc-tree", "sc-trunk", "sc-sky",
    "sc-gnd-light", "sc-cab", "sc-wheel", "car-1", "car-2", "car-3", "car-4", "car-dark", "bus", "ego", "adv", "ped", "pt-sim", "pt-real",
    "blue", "yellow", "red", "green", "teal", "purple", "gold", "grey", "ink", "ink-3"];
  var PAL = {}, MATS = {}, onTheme = [];
  function readPal() { var cs = getComputedStyle(document.documentElement); KEYS.forEach(function (k) { PAL[k] = new T.Color(cs.getPropertyValue("--" + k).trim() || "#888"); }); }
  function isDark() { return PAL["sc-bg"].getHSL({}).l < .3; }
  function mat(key, opts, Type) {
    var m = new (Type || T.MeshLambertMaterial)(Object.assign({}, opts || {}));
    (MATS[key] = MATS[key] || []).push(m);
    if (PAL[key]) m.color.copy(PAL[key]);
    return m;
  }
  function applyPal() {
    readPal();
    Object.keys(MATS).forEach(function (k) { MATS[k].forEach(function (m) { m.color.copy(PAL[k]); }); });
    onTheme.forEach(function (f) { f(); });
  }
  readPal();

  // ================================================================== renderer, scene, lights
  var stage = $("stage"), labels = $("labels");
  var renderer = new T.WebGLRenderer({ antialias: true, preserveDrawingBuffer: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap;
  stage.insertBefore(renderer.domElement, labels);
  var scene = new T.Scene(); scene.background = new T.Color(); scene.fog = new T.Fog(0xffffff, 140, 520);
  var camera = new T.PerspectiveCamera(38, 1, 0.3, 2500);
  var hemi = new T.HemisphereLight(0xffffff, 0x888888, 0.75); scene.add(hemi);
  var sun = new T.DirectionalLight(0xffffff, 0.75); sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048); var sc = sun.shadow.camera; sc.left = -70; sc.right = 70; sc.top = 70; sc.bottom = -70; sc.near = 10; sc.far = 400;
  sun.shadow.bias = -0.0008; scene.add(sun); scene.add(sun.target);
  var env = { tod: 0.62, fog: 0.15, rain: false, wet: 0 }, ENV0 = Object.assign({}, env);      // time of day 0..1 (0.5 = noon), fog 0..1
  function applyEnv() {
    var dark = isDark(), day = Math.sin(Math.PI * clamp(env.tod, 0, 1)), night = 1 - day;
    var bg = PAL["sc-bg"].clone().lerp(new T.Color(dark ? 0x05070a : 0x2a3340), night * (dark ? 0.6 : 0.55));
    scene.background.copy(bg);
    scene.fog.color.copy(bg);
    scene.fog.near = 60 + 120 * (1 - env.fog); scene.fog.far = 140 + 520 * (1 - env.fog) * (1 - env.fog) + (env.rain ? -40 : 0);
    hemi.color.copy(PAL["sc-sky"]); hemi.groundColor.copy(PAL["sc-gnd-light"]);
    hemi.intensity = (dark ? 0.62 : 0.5) * (0.45 + 0.55 * day);
    var warm = new T.Color(0xffd9a8), white = new T.Color(0xffffff);
    sun.color.copy(white.clone().lerp(warm, clamp(1 - day * 1.4, 0, 1)));
    sun.intensity = (dark ? 0.6 : 0.62) * Math.max(0.12, day);
    roadMatTone();
  }
  onTheme.push(applyEnv);

  // ================================================================== camera rig (orbit + follow + tweens)
  var rig = { tx: -30, ty: 0, tz: 2, dist: 72, yaw: 2.72, pitch: 0.6, follow: true, fp: null };
  var tween = null;
  function camTo(p, dur) {
    var from = { tx: rig.tx, ty: rig.ty, tz: rig.tz, dist: rig.dist, yaw: rig.yaw, pitch: rig.pitch };
    var to = Object.assign({}, from, p);
    var dy = to.yaw - from.yaw; dy = Math.atan2(Math.sin(dy), Math.cos(dy)); to.yaw = from.yaw + dy;
    tween = { from: from, to: to, t0: performance.now(), dur: REDUCED ? 1 : (dur || 1600) };
  }
  function stepRig(dtReal, egoPos) {
    if (tween) {
      var u = ease((performance.now() - tween.t0) / tween.dur);
      ["dist", "yaw", "pitch"].forEach(function (k) { rig[k] = tween.from[k] + (tween.to[k] - tween.from[k]) * u; });
      if (!rig.follow) ["tx", "ty", "tz"].forEach(function (k) { rig[k] = tween.from[k] + (tween.to[k] - tween.from[k]) * u; });
      if (u >= 1) tween = null;
    }
    if (rig.follow && egoPos) {
      var k = 1 - Math.exp(-dtReal * 6);
      rig.tx += (egoPos.x + (rig.lead || 0) - rig.tx) * k; rig.tz += (egoPos.z - rig.tz) * k; rig.ty += (0 - rig.ty) * k;
    }
    if (rig.fp) {   // first-person override (Real2Sim novel view)
      camera.position.set(rig.fp.x, rig.fp.y, rig.fp.z); camera.lookAt(rig.fp.lx, rig.fp.ly, rig.fp.lz); return;
    }
    var cp = Math.cos(rig.pitch);
    camera.position.set(rig.tx + rig.dist * cp * Math.cos(rig.yaw), rig.ty + rig.dist * Math.sin(rig.pitch), rig.tz + rig.dist * cp * Math.sin(rig.yaw));
    camera.lookAt(rig.tx, rig.ty + 1, rig.tz);
  }
  (function pointer() {
    var el = renderer.domElement, ptrs = {}, last = null, pinch = 0;
    el.addEventListener("pointerdown", function (e) { ptrs[e.pointerId] = [e.clientX, e.clientY]; el.setPointerCapture(e.pointerId); last = [e.clientX, e.clientY]; tween = null; });
    el.addEventListener("pointermove", function (e) {
      if (!ptrs[e.pointerId]) return;
      var ids = Object.keys(ptrs);
      if (ids.length === 2) {
        ptrs[e.pointerId] = [e.clientX, e.clientY];
        var a = ptrs[ids[0]], b = ptrs[ids[1]], d = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) rig.dist = clamp(rig.dist * pinch / d, 6, 420); pinch = d; return;
      }
      var dx = e.clientX - last[0], dy = e.clientY - last[1]; last = [e.clientX, e.clientY]; ptrs[e.pointerId] = last;
      if (e.shiftKey || e.button === 2 || e.buttons === 2) {
        if (rig.follow) setFollow(false);
        var s = rig.dist * 0.0016, cy = Math.cos(rig.yaw), sy = Math.sin(rig.yaw);
        rig.tx += (-sy * dx - cy * dy) * s * -1; rig.tz += (cy * dx - sy * dy) * s * -1;
      } else { rig.yaw += dx * 0.006; rig.pitch = clamp(rig.pitch + dy * 0.004, 0.06, 1.48); }
    });
    function up(e) { delete ptrs[e.pointerId]; pinch = 0; }
    el.addEventListener("pointerup", up); el.addEventListener("pointercancel", up);
    el.addEventListener("contextmenu", function (e) { e.preventDefault(); });
    el.addEventListener("wheel", function (e) { e.preventDefault(); tween = null; rig.dist = clamp(rig.dist * Math.exp(e.deltaY * 0.0012), 6, 420); }, { passive: false });
  })();

  // ================================================================== static world
  var BOX = new T.BoxGeometry(1, 1, 1), PLANE = new T.PlaneGeometry(1, 1); PLANE.rotateX(-Math.PI / 2);
  var R0 = S.ROAD, world = new T.Group(); scene.add(world);
  var ground = new T.Mesh(new T.PlaneGeometry(1400, 1400).rotateX(-Math.PI / 2), mat("sc-ground")); ground.receiveShadow = true; world.add(ground); ground.position.x = 60;
  var roadMat = mat("sc-road");
  function roadMatTone() { roadMat.color.copy(PAL["sc-road"]); if (env.wet > 0) roadMat.color.lerp(isDark() ? new T.Color(0x0b0e12) : new T.Color(0x7f8994), env.wet * 0.6); }
  function flat(x0, x1, z0, z1, y, m) { var o = new T.Mesh(PLANE, m); o.scale.set(x1 - x0, 1, z1 - z0); o.position.set((x0 + x1) / 2, y, (z0 + z1) / 2); o.receiveShadow = true; world.add(o); return o; }
  flat(R0.xMin - 60, R0.xMax + 60, -R0.half, R0.half, 0.02, roadMat);
  flat(R0.crossX - R0.crossHalf, R0.crossX + R0.crossHalf, -260, 260, 0.021, roadMat);
  var walkMat = mat("sc-walk");
  function walkBox(x0, x1, z0, z1) { var o = new T.Mesh(BOX, walkMat); o.scale.set(x1 - x0, 0.16, z1 - z0); o.position.set((x0 + x1) / 2, 0.08, (z0 + z1) / 2); o.receiveShadow = true; world.add(o); }
  [[R0.xMin - 60, 73 - 3.5], [87 + 3.5, R0.xMax + 60]].forEach(function (r) { walkBox(r[0], r[1], 7, 10.6); walkBox(r[0], r[1], -10.6, -7); });
  [[-260, -7], [7, 260]].forEach(function (r) { walkBox(69.5, 73, r[0], r[1]); walkBox(87, 90.5, r[0], r[1]); });
  // markings as one instanced mesh of flat strips
  var marks = [], ymarks = [];
  function dashes(x0, x1, z, len, gap, w, skip, out) { for (var x = x0; x < x1; x += len + gap) { if (skip && x + len > skip[0] && x < skip[1]) continue; (out || marks).push([x + len / 2, z, len, w, 0]); } }
  dashes(R0.xMin, R0.xMax, 3.5, 3, 6, 0.15, [64, 96]); dashes(R0.xMin, R0.xMax, -3.5, 3, 6, 0.15, [64, 96]);
  dashes(R0.xMin, R0.xMax, 6.8, 9, 0.001, 0.16, [64, 96]); dashes(R0.xMin, R0.xMax, -6.8, 9, 0.001, 0.16, [64, 96]);
  dashes(R0.xMin, R0.xMax, 0.12, 9, 0.001, 0.12, [64, 96], ymarks); dashes(R0.xMin, R0.xMax, -0.12, 9, 0.001, 0.12, [64, 96], ymarks);
  for (var zz = -6.5; zz <= 6.6; zz += 1.0) marks.push([R0.walkX, zz, 3, 0.5, 0]);       // zebra at the crosswalk
  [[-250, -9], [9, 250]].forEach(function (r) { for (var z = r[0]; z < r[1]; z += 9) { marks.push([80.12, z + 1.5, 0.12, 3, 0]); } });
  marks.push([81.75, 9.4, 3.3, 0.35, 0]);                                                  // stop line for the cross road
  function instStrips(list, m) {
    var im = new T.InstancedMesh(PLANE, m, list.length), o = new T.Object3D();
    list.forEach(function (d, i) { o.position.set(d[0], 0.035, d[1]); o.scale.set(d[2], 1, d[3]); o.rotation.y = d[4]; o.updateMatrix(); im.setMatrixAt(i, o.matrix); });
    im.receiveShadow = true; world.add(im); return im;
  }
  var markMesh = instStrips(marks, mat("sc-mark", {}, T.MeshBasicMaterial));
  var ymarkMesh = instStrips(ymarks, mat("gold", {}, T.MeshBasicMaterial));

  // buildings: deterministic blocks on both sides; AABBs are LiDAR targets
  var BLD = [];
  (function genBuildings() {
    var r = rng(11);
    [1, -1].forEach(function (side) {
      var x = R0.xMin - 30;
      while (x < R0.xMax + 40) {
        var w = 10 + r() * 22;
        if (x + w > 64 && x < 96) { x = 96 + r() * 4; continue; }
        var depth = 12 + r() * 22, h = 6 + Math.pow(r(), 1.7) * 24, z0 = side > 0 ? 13 + r() * 3 : -13 - r() * 3 - depth;
        BLD.push({ x0: x, x1: x + w - 1.8, z0: z0, z1: z0 + depth, h: h, glass: r() < 0.22, tone: r() < 0.5 });
        x += w;
      }
    });
    // blocks along the cross road
    [[96, 128, 13, 1], [32, 64, 13, 0], [96, 130, -13, 1], [30, 64, -13, 0]].forEach(function () {});
    [-40, -70, 40, 70].forEach(function (zc, i) {
      BLD.push({ x0: 50, x1: 66, z0: zc - 12, z1: zc + 12, h: 8 + 6 * (i % 2), glass: i === 1, tone: true });
      BLD.push({ x0: 94, x1: 112, z0: zc - 12, z1: zc + 12, h: 10 + 6 * (i % 3), glass: false, tone: false });
    });
  })();
  var bldMesh = new T.InstancedMesh(BOX, new T.MeshLambertMaterial({ color: 0xffffff }), BLD.length);
  bldMesh.castShadow = true; bldMesh.receiveShadow = true; world.add(bldMesh);
  (function () { var o = new T.Object3D(); BLD.forEach(function (b, i) { o.position.set((b.x0 + b.x1) / 2, b.h / 2, (b.z0 + b.z1) / 2); o.scale.set(b.x1 - b.x0, b.h, b.z1 - b.z0); o.updateMatrix(); bldMesh.setMatrixAt(i, o.matrix); }); })();
  function colorBuildings() { BLD.forEach(function (b, i) { bldMesh.setColorAt(i, b.glass ? PAL["sc-glass"] : b.tone ? PAL["sc-bldg"] : PAL["sc-bldg2"]); }); if (bldMesh.instanceColor) bldMesh.instanceColor.needsUpdate = true; }
  colorBuildings(); onTheme.push(colorBuildings);
  var edgeMat = mat("sc-edge", { transparent: true, opacity: 0.85 }, T.LineBasicMaterial);
  (function edges() {
    var p = [];
    BLD.forEach(function (b) {
      var X = [b.x0, b.x1], Z = [b.z0, b.z1], Y = [0, b.h];
      function L(a, c) { p.push(a[0], a[1], a[2], c[0], c[1], c[2]); }
      for (var i = 0; i < 2; i++) for (var j = 0; j < 2; j++) { L([X[i], 0, Z[j]], [X[i], b.h, Z[j]]); }
      Y.forEach(function (y) { L([X[0], y, Z[0]], [X[1], y, Z[0]]); L([X[0], y, Z[1]], [X[1], y, Z[1]]); L([X[0], y, Z[0]], [X[0], y, Z[1]]); L([X[1], y, Z[0]], [X[1], y, Z[1]]); });
      if (b.glass) for (var y = 3.5; y < b.h; y += 3.5) { L([X[0], y, Z[0]], [X[1], y, Z[0]]); L([X[0], y, Z[1]], [X[1], y, Z[1]]); }
    });
    var g = new T.BufferGeometry(); g.setAttribute("position", new T.Float32BufferAttribute(p, 3));
    world.add(new T.LineSegments(g, edgeMat));
  })();
  // trees
  var TREES = [];
  [9.7, -9.7].forEach(function (z) { for (var x = R0.xMin; x < R0.xMax; x += 15) { if ((x > 60 && x < 100) || (x > 114 && x < 131) || (z > 0 && x > 96 && x < 115)) continue; TREES.push([x + (z > 0 ? 4 : 0), z]); } });
  var crown = new T.InstancedMesh(new T.IcosahedronGeometry(1.7, 0), mat("sc-tree"), TREES.length);
  var trunk = new T.InstancedMesh(new T.CylinderGeometry(0.16, 0.2, 2.6, 6), mat("sc-trunk"), TREES.length);
  (function () { var o = new T.Object3D(), r = rng(5); TREES.forEach(function (t, i) { var s = 0.8 + r() * 0.5; o.position.set(t[0], 3.4 * s + 0.4, t[1]); o.scale.set(s, s * 1.15, s); o.rotation.y = r() * 6; o.updateMatrix(); crown.setMatrixAt(i, o.matrix); o.position.set(t[0], 1.3, t[1]); o.scale.set(1, 1, 1); o.updateMatrix(); trunk.setMatrixAt(i, o.matrix); }); })();
  crown.castShadow = trunk.castShadow = true; world.add(crown); world.add(trunk);
  // bus stop shelter
  (function () { var m = mat("sc-edge"), g = new T.Group(); [[100, 9.9], [108, 9.9]].forEach(function (p) { var post = new T.Mesh(BOX, m); post.scale.set(0.12, 2.6, 0.12); post.position.set(p[0], 1.3, p[1]); g.add(post); });
    var roof = new T.Mesh(BOX, mat("sc-bldg2")); roof.scale.set(9, 0.15, 1.8); roof.position.set(104, 2.65, 9.6); roof.castShadow = true; g.add(roof); world.add(g); })();

  // ================================================================== actors
  var WHEEL = new T.CylinderGeometry(0.34, 0.34, 0.26, 14); WHEEL.rotateX(Math.PI / 2);
  var colorKey = { ego: "ego", c1: "car-1", c2: "car-2", c3: "car-3", c4: "car-4", adv: "adv", bus: "bus", ped: "ped" };
  function carMesh(key, kind) {
    var g = new T.Group(), body, cab, L = kind === "bus" ? 12 : 4.5, W = kind === "bus" ? 2.5 : 1.86;
    if (kind === "bus") {
      body = new T.Mesh(BOX, mat(key)); body.scale.set(L, 2.5, W); body.position.y = 1.65; g.add(body);
      cab = new T.Mesh(BOX, mat("sc-cab")); cab.scale.set(L - 1.2, 0.8, W + 0.02); cab.position.set(0.2, 2.15, 0); g.add(cab);
    } else {
      body = new T.Mesh(BOX, mat(key)); body.scale.set(L, 0.72, W); body.position.y = 0.66; g.add(body);
      cab = new T.Mesh(BOX, mat("sc-cab")); cab.scale.set(2.3, 0.6, W - 0.2); cab.position.set(-0.3, 1.32, 0); g.add(cab);
      var top = new T.Mesh(BOX, mat(key)); top.scale.set(2.0, 0.08, W - 0.26); top.position.set(-0.35, 1.66, 0); g.add(top);
    }
    var wx = kind === "bus" ? [4.2, -3.8] : [1.45, -1.45], wz = W / 2 - 0.12, wheels = [];
    wx.forEach(function (x) { [-1, 1].forEach(function (s) { var w = new T.Mesh(WHEEL, mat("sc-wheel")); w.position.set(x, 0.34, s * wz); g.add(w); wheels.push(w); }); });
    if (key === "ego") {
      var puck = new T.Mesh(new T.CylinderGeometry(0.32, 0.36, 0.3, 20), mat("sc-cab")); puck.position.set(-0.25, 1.86, 0); g.add(puck);
      var ring = new T.Mesh(new T.TorusGeometry(0.34, 0.035, 6, 24).rotateX(Math.PI / 2), mat("ego", {}, T.MeshBasicMaterial)); ring.position.set(-0.25, 1.9, 0); g.add(ring);
    }
    g.traverse(function (o) { if (o.isMesh) { o.castShadow = true; } });
    g.userData.wheels = wheels;
    return g;
  }
  function pedMesh() {
    var g = new T.Group(), m = mat("ped");
    var b = new T.Mesh(new T.CylinderGeometry(0.2, 0.24, 1.05, 10), m); b.position.y = 0.95; g.add(b);
    var h = new T.Mesh(new T.SphereGeometry(0.17, 12, 10), m); h.position.y = 1.62; g.add(h);
    var l1 = new T.Mesh(BOX, mat("sc-cab")); l1.scale.set(0.14, 0.5, 0.14); l1.position.set(0, 0.25, 0.1); g.add(l1);
    var l2 = l1.clone(); l2.position.z = -0.1; g.add(l2); g.userData.legs = [l1, l2];
    g.traverse(function (o) { if (o.isMesh) o.castShadow = true; });
    return g;
  }

  // rollouts: the log (human ego, reactive world), the planner in a reactive world, the planner against the replayed log
  var LOG = S.rollout({ ego: "human", others: "reactive" });
  var RUN = { reactive: S.rollout({ ego: "planner", others: "reactive" }) };
  RUN.replay = S.rollout({ ego: "planner", others: "replay", log: LOG });
  var R = RUN.reactive, ACT = R.actors, NA = ACT.length, EGO_I = 0;
  var actorObj = ACT.map(function (a) {
    var o = a.kind === "ped" ? pedMesh() : carMesh(a.dark ? "car-dark" : colorKey[a.color], a.kind);
    scene.add(o); return o;
  });
  var parkedObj = S.PARKED.map(function (p) { var o = carMesh(p.dark ? "car-dark" : colorKey[p.color], "car"); o.position.set(p.x, 0, p.z); o.rotation.y = Math.atan2(-p.hz, p.hx); scene.add(o); return o; });
  var ghost = carMesh("ego", "car"); ghost.traverse(function (o) { if (o.isMesh) { o.material = o.material.clone(); o.material.transparent = true; o.material.opacity = 0.28; o.castShadow = false; } }); ghost.visible = false; scene.add(ghost);

  function pose(Rr, i, t) {
    var x = t / DT, f = Math.min(NF - 2, Math.max(0, Math.floor(x))), u = clamp(x - f, 0, 1), N = Rr.actors.length, F = Rr.F;
    var a = (f * N + i) * 7, b = ((f + 1) * N + i) * 7;
    var hx = F[a + 2] + (F[b + 2] - F[a + 2]) * u, hz = F[a + 3] + (F[b + 3] - F[a + 3]) * u, hl = Math.hypot(hx, hz) || 1;
    return { x: F[a] + (F[b] - F[a]) * u, z: F[a + 1] + (F[b + 1] - F[a + 1]) * u, hx: hx / hl, hz: hz / hl, v: F[a + 4] + (F[b + 4] - F[a + 4]) * u, acc: F[a + 5], d: F[a + 6], f: f };
  }
  function heightOf(a) { return a.kind === "bus" ? 3.0 : a.kind === "ped" ? 1.8 : 1.7; }

  // ================================================================== app state
  var st = {
    ch: 0, t: 0, playing: !REDUCED, speed: 1, mode: "reactive", variant: null,
    lidar: false, lidarReal: false, frustums: false, radar: false, showDrop: true,
    det: false, gt: false, tracker: false, trails: false, noise: 0.25,
    pred: false, predGT: false, plan: false, st: false,
    ghostLog: false, variants: false, splat: false, splatOnly: false, holes: false, novel: 0, novelH: 1.6, labGhost: true, stuGhost: true
  };
  if (REDUCED) st.t = 11.5;

  // ================================================================== LiDAR
  var NB = 32, NAZ = 360, LID = { elev: [], spinHz: 5, az: 0, rev: 0, pos: new Float32Array(NB * NAZ * 3), col: new Float32Array(NB * NAZ * 3), drop: new Float32Array(NB * NAZ * 3) };
  for (var bi = 0; bi < NB; bi++) LID.elev.push((-25 + 33 * bi / (NB - 1)) * Math.PI / 180);
  LID.pos.fill(-999); LID.drop.fill(-999);
  var lidGeo = new T.BufferGeometry(); lidGeo.setAttribute("position", new T.BufferAttribute(LID.pos, 3)); lidGeo.setAttribute("color", new T.BufferAttribute(LID.col, 3));
  var dotTex = (function () { var c = document.createElement("canvas"); c.width = c.height = 64; var x = c.getContext("2d"); var g = x.createRadialGradient(32, 32, 0, 32, 32, 30); g.addColorStop(0, "rgba(255,255,255,1)"); g.addColorStop(.7, "rgba(255,255,255,1)"); g.addColorStop(1, "rgba(255,255,255,0)"); x.fillStyle = g; x.beginPath(); x.arc(32, 32, 30, 0, 7); x.fill(); var t = new T.CanvasTexture(c); return t; })();
  var lidPts = new T.Points(lidGeo, new T.PointsMaterial({ size: 0.32, vertexColors: true, map: dotTex, alphaTest: 0.4, sizeAttenuation: true }));
  lidPts.frustumCulled = false; lidPts.visible = false; scene.add(lidPts);
  var dropGeo = new T.BufferGeometry(); dropGeo.setAttribute("position", new T.BufferAttribute(LID.drop, 3));
  var dropPts = new T.Points(dropGeo, mat("red", { size: 0.1, map: dotTex, alphaTest: 0.4, transparent: true, opacity: 0.55 }, T.PointsMaterial)); dropPts.frustumCulled = false; dropPts.visible = false; scene.add(dropPts);
  var sweepGeo = new T.BufferGeometry(); sweepGeo.setAttribute("position", new T.BufferAttribute(new Float32Array(NB * 2 * 3), 3));
  var sweepMat = mat("yellow", { transparent: true, opacity: 0.5 }, T.LineBasicMaterial);
  var sweep = new T.LineSegments(sweepGeo, sweepMat); sweep.frustumCulled = false; sweep.visible = false; scene.add(sweep);

  var STATIC_BOX = BLD.map(function (b) { return { cx: (b.x0 + b.x1) / 2, cz: (b.z0 + b.z1) / 2, hx: 1, hz: 0, l: (b.x1 - b.x0) / 2, w: (b.z1 - b.z0) / 2, h: b.h, kind: b.glass ? "glass" : "bldg", src: b }; })
    .concat(S.PARKED.map(function (p) { return { cx: p.x, cz: p.z, hx: p.hx, hz: p.hz, l: 2.3, w: 0.93, h: 1.6, kind: p.dark ? "dark" : "car", parked: true, color: p.dark ? "car-dark" : colorKey[p.color] }; }));
  function dynBoxes(t, Rr) {
    var out = [];
    for (var i = 0; i < NA; i++) { if (i === EGO_I) continue; var a = ACT[i], p = pose(Rr, i, t);
      out.push({ cx: p.x, cz: p.z, hx: p.hx, hz: p.hz, l: a.dims.len / 2, w: a.dims.wid / 2, h: heightOf(a), kind: a.kind === "ped" ? "ped" : a.dark ? "dark" : "car", idx: i }); }
    return out;
  }
  function castRay(ox, oy, oz, dx, dy, dz, boxes, maxR) {
    var best = maxR, hit = null;
    if (dy < -1e-6) { var tg = -oy / dy; if (tg < best) { best = tg; hit = "ground"; } }
    for (var k = 0; k < boxes.length; k++) {
      var b = boxes[k], rx = ox - b.cx, rz = oz - b.cz;
      var lx = rx * b.hx + rz * b.hz, lz = -rx * b.hz + rz * b.hx, ux = dx * b.hx + dz * b.hz, uz = -dx * b.hz + dz * b.hx;
      var t0 = 0, t1 = best, o = [lx, oy, lz], d = [ux, dy, uz], lo = [-b.l, 0, -b.w], hi = [b.l, b.h, b.w], ok = true;
      for (var a = 0; a < 3; a++) {
        if (Math.abs(d[a]) < 1e-9) { if (o[a] < lo[a] || o[a] > hi[a]) { ok = false; break; } continue; }
        var u = (lo[a] - o[a]) / d[a], v = (hi[a] - o[a]) / d[a]; if (u > v) { var w = u; u = v; v = w; }
        if (u > t0) t0 = u; if (v < t1) t1 = v; if (t0 > t1) { ok = false; break; }
      }
      if (ok && t0 > 0.05 && t0 < best) { best = t0; hit = b; }
    }
    return hit ? { t: best, hit: hit } : null;
  }
  function dropProb(h, r) {
    var k = h === "ground" ? "ground" : h.kind, p = k === "glass" ? 0.88 : k === "dark" ? 0.55 : 0.02;
    if (r > 35) p += (r - 35) / 70;
    if (env.rain) p += 0.12 + r / 160;
    if (env.fog > 0.4) p += (env.fog - 0.4) * r / 60;
    return clamp(p, 0, 0.98);
  }
  function lidarStep(dtReal, t, Rr) {
    var ego = pose(Rr, EGO_I, t), ox = ego.x - 0.25 * ego.hx, oz = ego.z - 0.25 * ego.hz, oy = 1.95;
    var near = STATIC_BOX.filter(function (b) { return Math.abs(b.cx - ox) < 95 + b.l && Math.abs(b.cz - oz) < 95 + b.w; }).concat(dynBoxes(t, Rr).filter(function (b) { return Math.abs(b.cx - ox) < 80 && Math.abs(b.cz - oz) < 80; }));
    var adv = Math.min(NAZ, Math.max(1, Math.round(LID.spinHz * NAZ * dtReal)));
    var simCol = PAL["pt-sim"], realCol = PAL["pt-real"], base = st.lidarReal ? realCol : simCol;
    for (var s = 0; s < adv; s++) {
      LID.az = (LID.az + 1) % NAZ; if (LID.az === 0) LID.rev++;
      var th = LID.az * 2 * Math.PI / NAZ, ca = Math.cos(th), sa = Math.sin(th);
      for (var b = 0; b < NB; b++) {
        var el = LID.elev[b], ce = Math.cos(el), dx = ce * ca, dy = Math.sin(el), dz = ce * sa, slot = (LID.az * NB + b) * 3;
        var h = castRay(ox, oy, oz, dx, dy, dz, near, 80);
        LID.drop[slot + 1] = -999; LID.pos[slot + 1] = -999;
        if (!h) continue;
        var r = h.t, keep = true;
        if (st.lidarReal) { keep = hash(LID.az, b, LID.rev) > dropProb(h.hit, r); r += 0.03 * gauss(LID.az, b, LID.rev + 9) * (env.rain ? 3 : 1); }
        var px = ox + dx * r, py = oy + dy * r, pz = oz + dz * r;
        if (!keep) { LID.drop[slot] = px; LID.drop[slot + 1] = py; LID.drop[slot + 2] = pz; continue; }
        LID.pos[slot] = px; LID.pos[slot + 1] = py; LID.pos[slot + 2] = pz;
        var shade = h.hit === "ground" ? 0.45 : 0.6 + 0.4 * clamp(py / 8, 0, 1);
        if (h.hit !== "ground" && (h.hit.kind === "car" || h.hit.kind === "dark" || h.hit.kind === "ped")) shade = 1.0;
        var c = base;
        LID.col[slot] = c.r * shade + (1 - shade) * PAL["sc-bg"].r * 0.5; LID.col[slot + 1] = c.g * shade + (1 - shade) * PAL["sc-bg"].g * 0.5; LID.col[slot + 2] = c.b * shade + (1 - shade) * PAL["sc-bg"].b * 0.5;
      }
    }
    lidGeo.attributes.position.needsUpdate = true; lidGeo.attributes.color.needsUpdate = true; dropGeo.attributes.position.needsUpdate = true;
    // sweep fan at the current azimuth
    var sp = sweepGeo.attributes.position.array, th2 = LID.az * 2 * Math.PI / NAZ;
    for (var b2 = 0; b2 < NB; b2++) {
      var e2 = LID.elev[b2], c2 = Math.cos(e2), L = 14 + 0.6 * b2;
      sp[b2 * 6] = ox; sp[b2 * 6 + 1] = oy; sp[b2 * 6 + 2] = oz;
      sp[b2 * 6 + 3] = ox + c2 * Math.cos(th2) * L; sp[b2 * 6 + 4] = Math.max(0.05, oy + Math.sin(e2) * L); sp[b2 * 6 + 5] = oz + c2 * Math.sin(th2) * L;
    }
    sweepGeo.attributes.position.needsUpdate = true;
    sweepMat.color.copy(st.lidarReal ? PAL["blue"] : PAL["yellow"]);
  }
  function lidarClear() { LID.pos.fill(-999); LID.drop.fill(-999); lidGeo.attributes.position.needsUpdate = true; dropGeo.attributes.position.needsUpdate = true; }

  // ================================================================== sensors on the ego: camera frustums and radar
  var sensorRig = new T.Group(); sensorRig.visible = false; actorObj[EGO_I].add(sensorRig);
  (function buildFrustums() {
    var lines = [], tris = [], cams = [[0, 60, 30], [55, 50, 22], [-55, 50, 22], [120, 60, 18], [-120, 60, 18], [180, 70, 20]];
    cams.forEach(function (c) {
      var yaw = c[0] * Math.PI / 180, hf = c[1] * Math.PI / 360, vf = hf * 0.62, L = c[2], apex = [0.6, 1.7, 0];
      var corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(function (s) {
        var lx = L, ly = Math.tan(vf) * L * s[1], lz = Math.tan(hf) * L * s[0];
        return [apex[0] + lx * Math.cos(yaw) - lz * Math.sin(yaw), apex[1] + ly, apex[2] + lx * Math.sin(-yaw) + lz * Math.cos(yaw)];
      });
      corners.forEach(function (q, i) { lines.push(apex[0], apex[1], apex[2], q[0], q[1], q[2]); var n = corners[(i + 1) % 4]; lines.push(q[0], q[1], q[2], n[0], n[1], n[2]); tris.push(apex[0], apex[1], apex[2], q[0], q[1], q[2], n[0], n[1], n[2]); });
    });
    var g = new T.BufferGeometry(); g.setAttribute("position", new T.Float32BufferAttribute(lines, 3));
    sensorRig.add(new T.LineSegments(g, mat("blue", { transparent: true, opacity: 0.32 }, T.LineBasicMaterial)));
    var g2 = new T.BufferGeometry(); g2.setAttribute("position", new T.Float32BufferAttribute(tris, 3)); g2.computeVertexNormals();
    sensorRig.add(new T.Mesh(g2, mat("blue", { transparent: true, opacity: 0.03, side: T.DoubleSide, depthWrite: false }, T.MeshBasicMaterial)));
  })();
  var radarRig = new T.Group(); radarRig.visible = false; actorObj[EGO_I].add(radarRig);
  (function buildRadar() {
    [[80, 9, 0.09], [30, 45, 0.06]].forEach(function (r) {
      var g = new T.CircleGeometry(r[0], 32, -r[1] * Math.PI / 180, 2 * r[1] * Math.PI / 180); g.rotateX(-Math.PI / 2);
      var m = new T.Mesh(g, mat("green", { transparent: true, opacity: r[2], side: T.DoubleSide, depthWrite: false }, T.MeshBasicMaterial)); m.position.set(2.2, 0.6, 0); radarRig.add(m);
      var e = new T.LineSegments(new T.EdgesGeometry(g), mat("green", { transparent: true, opacity: 0.5 }, T.LineBasicMaterial)); e.position.copy(m.position); radarRig.add(e);
    });
  })();

  // ================================================================== perception: detections, ground truth, tracker, trails
  var EDGE_BOX = new T.EdgesGeometry(new T.BoxGeometry(1, 1, 1)); EDGE_BOX.translate(0, 0.5, 0);
  function boxPool(n, key, dashed) {
    var arr = [];
    for (var i = 0; i < n; i++) {
      var m = dashed ? mat(key, { dashSize: 0.35, gapSize: 0.25, transparent: true, opacity: 0.8 }, T.LineDashedMaterial) : mat(key, { transparent: true, opacity: 0.95 }, T.LineBasicMaterial);
      var g = dashed ? EDGE_BOX.clone() : EDGE_BOX, o = new T.LineSegments(g, m);
      if (dashed) o.computeLineDistances();
      o.visible = false; scene.add(o); arr.push(o);
    }
    return arr;
  }
  var detBoxes = boxPool(NA, "blue"), gtBoxes = boxPool(NA, "grey", true), trkBoxes = boxPool(NA, "teal");
  var trailPts = (function () { var g = new T.BufferGeometry(); g.setAttribute("position", new T.BufferAttribute(new Float32Array(NA * 80 * 3), 3)); var p = new T.Points(g, mat("red", { size: 0.22, map: dotTex, alphaTest: 0.4 }, T.PointsMaterial)); p.frustumCulled = false; p.visible = false; scene.add(p); return p; })();
  var trailLines = (function () { var g = new T.BufferGeometry(); g.setAttribute("position", new T.BufferAttribute(new Float32Array(NA * 80 * 6), 3)); var l = new T.LineSegments(g, mat("teal", {}, T.LineBasicMaterial)); l.frustumCulled = false; l.visible = false; scene.add(l); return l; })();
  var TRK = {}, lastPF = -1;
  function visibleFrom(ego, p, i, boxes) {
    for (var k = 0; k < boxes.length; k++) { var b = boxes[k]; if (b.idx === i) continue;
      if (S.segHitsBox(ego.x, ego.z, p.x, p.z, { x: b.cx, z: b.cz, hx: b.hx, hz: b.hz, l: b.l, w: b.w })) return false; }
    return true;
  }
  var DET = [];
  function perceive(t, Rr) {
    var f = Math.floor(t / DT + 1e-6), ego = pose(Rr, EGO_I, t), boxes = dynBoxes(t, Rr).concat(STATIC_BOX.filter(function (b) { return b.parked; }));
    if (f !== lastPF + 1) TRK = {};         // jumped: restart the tracker
    lastPF = f;
    DET = [];
    for (var i = 0; i < NA; i++) {
      if (i === EGO_I) continue;
      var a = ACT[i], p = pose(Rr, i, t), r = Math.hypot(p.x - ego.x, p.z - ego.z), maxR = a.kind === "ped" ? 45 : 75;
      var vis = r < maxR && visibleFrom(ego, p, i, boxes);
      var g = { i: i, a: a, p: p, r: r, vis: vis };
      if (vis) {
        var sg = st.noise * (0.6 + r / 60), nx = sg * gauss(i, f, 1), nz = sg * gauss(i, f, 2), ny = 0.06 * st.noise * gauss(i, f, 3);
        g.m = { x: p.x + nx, z: p.z + nz, yaw: Math.atan2(-p.hz, p.hx) + 0.25 * st.noise * gauss(i, f, 4), sl: 1 + 0.08 * st.noise * gauss(i, f, 5), conf: clamp(0.97 - r / 160 - st.noise * 0.2 + 0.04 * gauss(i, f, 6), 0.3, 0.99) };
        var k = TRK[i];
        if (!k) k = TRK[i] = { x: g.m.x, z: g.m.z, vx: p.v * p.hx, vz: p.v * p.hz, raw: [], sm: [], id: 3 + i * 7 % 41 };
        else { var px = k.x + k.vx * DT, pz = k.z + k.vz * DT, ex = g.m.x - px, ez = g.m.z - pz, al = 0.35, be = 0.06;
          k.x = px + al * ex; k.z = pz + al * ez; k.vx += be * ex / DT; k.vz += be * ez / DT; }
        k.raw.push([g.m.x, g.m.z]); k.sm.push([k.x, k.z]); if (k.raw.length > 60) { k.raw.shift(); k.sm.shift(); }
        k.seen = f; g.k = k;
      }
      DET.push(g);
    }
    Object.keys(TRK).forEach(function (id) { if (TRK[id].seen < f - 10) delete TRK[id]; });
  }
  function drawPerception() {
    var tp = trailPts.geometry.attributes.position.array, tl = trailLines.geometry.attributes.position.array, np = 0, nl = 0;
    tp.fill(-999); tl.fill(-999);
    for (var j = 0; j < NA; j++) { detBoxes[j].visible = gtBoxes[j].visible = trkBoxes[j].visible = false; }
    DET.forEach(function (g) {
      var a = g.a, H = heightOf(a), j = g.i;
      if (st.gt && st.det) { var o = gtBoxes[j]; o.visible = true; o.position.set(g.p.x, 0.02, g.p.z); o.rotation.y = Math.atan2(-g.p.hz, g.p.hx); o.scale.set(a.dims.len + 0.15, H + 0.12, a.dims.wid + 0.15); }
      if (!g.vis || !st.det) return;
      var d = detBoxes[j]; d.visible = true; d.position.set(g.m.x, 0.03, g.m.z); d.rotation.y = g.m.yaw; d.scale.set(a.dims.len * g.m.sl, H, a.dims.wid * g.m.sl);
      if (st.tracker && g.k) { var k = g.k, tb = trkBoxes[j]; tb.visible = true; tb.position.set(k.x, 0.04, k.z); tb.rotation.y = Math.atan2(-k.vz, k.vx || 1e-6); if (Math.hypot(k.vx, k.vz) < 0.5) tb.rotation.y = Math.atan2(-g.p.hz, g.p.hx); tb.scale.set(a.dims.len + 0.3, H + 0.2, a.dims.wid + 0.3); }
      if (st.trails && g.k) {
        g.k.raw.forEach(function (q) { tp[np++] = q[0]; tp[np++] = 0.15; tp[np++] = q[1]; });
        for (var s = 1; s < g.k.sm.length; s++) { var A = g.k.sm[s - 1], B = g.k.sm[s]; tl[nl++] = A[0]; tl[nl++] = 0.2; tl[nl++] = A[1]; tl[nl++] = B[0]; tl[nl++] = 0.2; tl[nl++] = B[1]; }
      }
    });
    trailPts.geometry.attributes.position.needsUpdate = true; trailLines.geometry.attributes.position.needsUpdate = true;
    trailPts.visible = trailLines.visible = st.trails && st.det;
  }

  // ================================================================== ribbons (prediction, planning)
  function Ribbon(key, opacity) {
    var g = new T.BufferGeometry(), N = 64; g.setAttribute("position", new T.BufferAttribute(new Float32Array(N * 2 * 3), 3));
    var idx = []; for (var i = 0; i < N - 1; i++) { var a = i * 2; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); } g.setIndex(idx);
    var m = mat(key, { transparent: true, opacity: opacity == null ? 0.6 : opacity, side: T.DoubleSide, depthWrite: false }, T.MeshBasicMaterial);
    var mesh = new T.Mesh(g, m); mesh.frustumCulled = false; mesh.visible = false; mesh.renderOrder = 3; scene.add(mesh);
    return {
      mesh: mesh, key: key, set: function (pts, w0, w1, y) {        // pts: [[x,z],...] up to 64
        var p = g.attributes.position.array, n = Math.min(pts.length, N);
        for (var i = 0; i < N; i++) {
          var q = pts[Math.min(i, n - 1)], a = pts[Math.max(0, Math.min(i, n - 1) - 1)], b = pts[Math.min(n - 1, Math.min(i, n - 1) + 1)];
          var tx = b[0] - a[0], tz = b[1] - a[1], L = Math.hypot(tx, tz) || 1, w = (w0 + (w1 - w0) * Math.min(i, n - 1) / Math.max(1, n - 1)) / 2;
          var nx = -tz / L * w, nz = tx / L * w, yy = (y || 0.08) + (q[2] || 0);
          p.set([q[0] + nx, yy, q[1] + nz, q[0] - nx, yy, q[1] - nz], i * 6);
        }
        g.attributes.position.needsUpdate = true; mesh.visible = true;
      },
      hide: function () { mesh.visible = false; }, color: function (k) { mesh.material.color.copy(PAL[k]); }, opacity: function (o) { mesh.material.opacity = o; }
    };
  }
  var predRibs = [], predActual = [], planRibs = [];
  for (var q = 0; q < 36; q++) predRibs.push(Ribbon("purple", 0.5));
  for (var q2 = 0; q2 < 14; q2++) predActual.push(Ribbon("ink-3", 0.5));
  for (var q3 = 0; q3 < 20; q3++) planRibs.push(Ribbon("green", 0.5));

  // ---------------------------------------------------------------- prediction: a few modes per agent
  var EB = [1.75, 5.25], WB = [-1.75, -5.25];
  function predictModes(g, Rr, t) {
    var a = g.a, p = g.p, out = [], H = 5, n = 26;
    var x = p.x, z = p.z, vx = p.v * p.hx, vz = p.v * p.hz;
    if (st.tracker && g.k) { x = g.k.x; z = g.k.z; vx = g.k.vx; vz = g.k.vz; }
    var v = Math.hypot(vx, vz), hx = v > 0.3 ? vx / v : p.hx, hz = v > 0.3 ? vz / v : p.hz;
    function line(acc, lat, durLat) {
      var pts = [], s = 0, vv = v;
      for (var i = 0; i < n; i++) { var tt = i * H / (n - 1);
        var lo = s, la = lat ? lat * (1 - Math.cos(Math.PI * Math.min(tt / durLat, 1))) / 2 : 0;
        pts.push([x + hx * lo - hz * la, z + hz * lo + hx * la]);
        var dt = H / (n - 1); vv = Math.max(0, vv + acc * dt); s += vv * dt; }
      return pts;
    }
    if (a.kind === "ped") {
      var onWalk = Math.abs(p.x - S.ROAD.walkX) < 2 && Math.abs(p.z) > 6.5;
      if (onWalk || (a.id === "P1" && p.z < 9.1 && p.z > -9.1)) {
        var cross = []; for (var i = 0; i < n; i++) cross.push([S.ROAD.walkX, z - 1.45 * i * H / (n - 1)]);
        out.push({ pts: cross, p: v > 0.2 ? 0.85 : 0.45, name: "băng qua" });
        out.push({ pts: [[x, z], [x + 0.2, z]], p: v > 0.2 ? 0.15 : 0.55, name: "đứng chờ" });
      } else { out.push({ pts: line(0, 0, 1), p: 0.9, name: "đi tiếp" }); }
      return out;
    }
    if (v < 0.4) {
      if (a.id === "X1") {
        var P = S.LANES.XR, base = 58, tr = [], st2 = [];
        for (var k = 0; k < n; k++) { var ss = base + k * 1.1; var q = S.at(P, ss, 0); tr.push([q.x, q.z]); var q2b = S.at(S.LANES.NB, 108 + k * 1.1, 0); st2.push([q2b.x, q2b.z]); }
        return [{ pts: tr, p: 0.55, name: "rẽ phải" }, { pts: st2, p: 0.25, name: "đi thẳng" }, { pts: [[x, z], [x + 0.01, z - 0.2]], p: 0.2, name: "chờ" }];
      }
      return [{ pts: [[x, z], [x + hx * 0.3, z + hz * 0.3]], p: 1, name: "đứng yên" }];
    }
    var lanes = hx > 0 ? EB : WB, cur = lanes.reduce(function (b, c) { return Math.abs(c - z) < Math.abs(b - z) ? c : b; }, lanes[0]);
    var other = lanes[0] === cur ? lanes[1] : lanes[0], latV = vz * (hx > 0 ? 1 : -1);
    var toward = Math.sign(other - z) * (hx > 0 ? 1 : -1), lcP = clamp(0.12 + Math.max(0, latV * toward) * 1.3, 0.05, 0.85);
    var right = (other - z) * (hx > 0 ? 1 : -1);
    if (Math.abs(p.d) > 0.3 && Math.abs(p.d) < 3.2) lcP = Math.max(lcP, 0.8);
    var keep = 1 - lcP;
    out.push({ pts: line(0, (cur - z) * (hx > 0 ? 1 : -1), 1.5), p: keep * 0.75, name: "giữ làn" });
    out.push({ pts: line(-3, (cur - z) * (hx > 0 ? 1 : -1), 1.5), p: keep * 0.25, name: "phanh" });
    out.push({ pts: line(0, right, 3), p: lcP, name: "chuyển làn" });
    return out;
  }
  var PRED = [];
  function drawPrediction(t, Rr) {
    predRibs.forEach(function (r) { r.hide(); }); predActual.forEach(function (r) { r.hide(); });
    PRED = [];
    if (!st.pred) return;
    var ego = pose(Rr, EGO_I, t), list = DET.filter(function (g) { return g.vis && g.r < 70; }).sort(function (a, b) { return a.r - b.r; }).slice(0, 12), ri = 0, ai = 0;
    list.forEach(function (g) {
      var modes = predictModes(g, Rr, t); PRED.push({ g: g, modes: modes });
      modes.forEach(function (m) {
        if (ri >= predRibs.length || m.pts.length < 2) return;
        var rb = predRibs[ri++]; rb.set(m.pts, 0.35, 0.35 + 1.6 * m.p, 0.1 + ri * 0.002);
        rb.color(g.a.kind === "ped" ? "ped" : "purple"); rb.opacity(0.12 + 0.6 * m.p);
      });
      if (st.predGT && ai < predActual.length) {
        var pts = []; for (var s = 0; s <= 5.001; s += 0.2) { var q = pose(Rr, g.i, Math.min(T_END, t + s)); pts.push([q.x, q.z]); }
        var r2 = predActual[ai++]; r2.set(pts, 0.14, 0.14, 0.13); r2.color("ink"); r2.opacity(0.85);
      }
    });
  }

  // ---------------------------------------------------------------- planning: a lattice of ego candidates scored against predictions
  var PLAN = { best: null, cands: [] };
  function plan(t, Rr) {
    var e = pose(Rr, EGO_I, t), H = 5, n = 26, dt = H / (n - 1), cands = [];
    var obst = [];
    function ahead(g) { var dx = g.p.x - e.x, dz = g.p.z - e.z; return dx * e.hx + dz * e.hz > 0.5 || g.a.kind === "ped"; }   // rear traffic is the follower's job
    // obstacle half-sizes; every mode with p >= 0.25 counts, not only the most likely one
    function half(a) { return a.kind === "ped" ? [0.4, 0.4] : [a.dims.len / 2, a.dims.wid / 2]; }
    PRED.forEach(function (pr) { if (!ahead(pr.g)) return; var hs = half(pr.g.a);
      pr.modes.forEach(function (m) { if (m.p >= 0.25 || m === pr.modes[0]) obst.push({ pts: m.pts, ped: pr.g.a.kind === "ped", r: hs[0], w: hs[1] }); }); });
    if (!st.pred) DET.forEach(function (g) { if (g.vis && g.r < 70 && ahead(g)) { var hs = half(g.a); obst.push({ pts: [[g.p.x, g.p.z]], ped: g.a.kind === "ped", r: hs[0], w: hs[1] }); } });
    [-3.5, -1.5, 0, 1.5, 3.5].forEach(function (lat) { [1, 0, -2, -4.5].forEach(function (acc) {
      var pts = [], s = 0, v = e.v, coll = false, offroad = false, opp = 0;
      for (var i = 0; i < n; i++) {
        var tt = i * dt, la = lat * (1 - Math.cos(Math.PI * Math.min(tt / 3, 1))) / 2, x = e.x + e.hx * s - e.hz * la, z = e.z + e.hz * s + e.hx * la;
        pts.push([x, z]);
        if (z < -6.2 || z > 6.2) offroad = true; if (z < 0.9) opp++;
        obst.forEach(function (o) { var q = o.pts[Math.min(i, o.pts.length - 1)], dx = q[0] - x, dz = q[1] - z;
          var lon = Math.abs(dx * e.hx + dz * e.hz), latd = Math.abs(-dx * e.hz + dz * e.hx); if (lon < o.r + 2.3 + 0.5 * v * dt + 0.5 && latd < o.w + 0.95 + 0.35) coll = true; });
        v = Math.max(0, v + acc * dt); s += v * dt;
      }
      var cost = (coll ? 1000 : 0) + (offroad ? 500 : 0) + (opp ? 300 + opp * 4 : 0) + Math.abs(lat) * 3 + Math.abs(acc) * 2.2 - s * 0.35 + Math.max(0, v - 13.9) * 14;   // v is the end speed; 13.9 m/s = 50 km/h limit
      cands.push({ pts: pts, lat: lat, acc: acc, cost: cost, bad: coll || offroad || opp > 0, coll: coll, offroad: offroad, opp: opp > 0, dist: s });
    }); });
    cands.sort(function (a, b) { return a.cost - b.cost; });
    PLAN = { best: cands[0], cands: cands };
  }
  function drawPlan() {
    planRibs.forEach(function (r) { r.hide(); });
    if (!st.plan || !PLAN.best) return;
    PLAN.cands.forEach(function (c, i) {
      var r = planRibs[i]; if (!r) return;
      var best = i === 0;
      r.set(c.pts, best ? 0.9 : 0.22, best ? 1.6 : 0.22, best ? 0.16 : 0.12);
      r.color(best ? "teal" : c.bad ? "red" : i < 6 ? "green" : "yellow"); r.opacity(best ? 0.9 : c.bad ? 0.55 : 0.5);
    });
  }
  function planText(c) {
    if (!c) return "–";
    var lat = c.lat === 0 ? "giữ làn" : c.lat < 0 ? "lệch trái " + fmt(-c.lat) + " m" : "lệch phải " + fmt(c.lat) + " m";
    var acc = c.acc > 0 ? "tăng tốc" : c.acc === 0 ? "giữ tốc" : c.acc > -3 ? "phanh nhẹ" : "phanh gấp";
    return lat + ", " + acc;
  }

  // ---------------------------------------------------------------- space-time (x, t, z): agents become worms, the plan must thread between them
  var stGroup = new T.Group(); stGroup.visible = false; scene.add(stGroup);
  var stK = 2.4, stLastF = -99, stMats = { purple: mat("purple", { transparent: true, opacity: 0.55 }), ped: mat("ped", { transparent: true, opacity: 0.6 }), teal: mat("teal", { transparent: true, opacity: 0.85 }), red: mat("red", { transparent: true, opacity: 0.85 }) };
  var stGrid = (function () { var p = []; for (var s = 1; s <= 6; s++) { var y = s * stK; p.push(-30, y, 0, 60, y, 0); } var g = new T.BufferGeometry(); g.setAttribute("position", new T.Float32BufferAttribute(p, 3)); return new T.LineSegments(g, mat("ink-3", { transparent: true, opacity: 0.35 }, T.LineBasicMaterial)); })();
  scene.add(stGrid); stGrid.visible = false;
  var stLabels = [];
  function buildST(t, Rr) {
    var f = Math.floor(t / DT); if (Math.abs(f - stLastF) < 3) return; stLastF = f;
    while (stGroup.children.length) { var c = stGroup.children.pop(); c.geometry.dispose(); }
    var ego = pose(Rr, EGO_I, t);
    for (var i = 0; i < NA; i++) {
      var a = ACT[i], p0 = pose(Rr, i, t); if (Math.hypot(p0.x - ego.x, p0.z - ego.z) > 75) continue;
      var pts = [];
      for (var s = 0; s <= 6.001; s += 0.25) { var q = pose(Rr, i, Math.min(T_END, t + s)); pts.push(new T.Vector3(q.x, 0.2 + s * stK, q.z)); }
      if (pts[0].distanceTo(pts[pts.length - 1]) < 0.01) pts[pts.length - 1].x += 0.01;
      var curve = new T.CatmullRomCurve3(pts), rad = i === EGO_I ? 0.55 : a.kind === "ped" ? 0.45 : a.kind === "bus" ? 1.25 : 0.95;
      var m = i === EGO_I ? stMats.teal : a.kind === "ped" ? stMats.ped : stMats.purple;
      stGroup.add(new T.Mesh(new T.TubeGeometry(curve, 48, rad, 10, false), m));
    }
    stGrid.position.set(ego.x, 0, ego.z); stGrid.scale.set(1, 1, 1);
  }

  // ---------------------------------------------------------------- collision markers
  var hitRings = [];
  function evWord(ev) { return ev.kind === "block" ? "chặn lối" : "va chạm"; }     // a block: the ego stopped on the crosswalk in a pedestrian's way
  function rebuildHits() {
    hitRings.forEach(function (o) { scene.remove(o); }); hitRings = [];
    R.events.forEach(function (ev) {
      var m = new T.Mesh(new T.TorusGeometry(2.6, 0.12, 8, 40).rotateX(Math.PI / 2), mat(ev.kind === "block" ? "gold" : "red", { transparent: true, opacity: 0.9 }, T.MeshBasicMaterial));
      m.position.set(ev.x, 0.3, ev.z); m.userData.ev = ev; m.visible = false; scene.add(m); hitRings.push(m);
    });
    drawTicks();
  }

  // ---------------------------------------------------------------- scenario variants (chapter 7)
  var VAR = null, T0S = [], DURS = [0.8, 1.2, 1.8, 2.6, 3.6];
  for (var tv = 1; tv <= 7.001; tv += 0.5) T0S.push(Math.round(tv * 10) / 10);
  function outcome(Rr, dur) {
    var alat = 1.75 * Math.pow(Math.PI / dur, 2);
    if (alat > 8) return "infeasible";
    if (Rr.events.some(function (e) { return e.a === "ego" || e.b === "ego"; })) return "collision";
    var ps = S.pairStats(Rr, "ego", "C2"); return ps.gapMin < 3 ? "near" : "safe";
  }
  var OUT_COL = { collision: "red", near: "gold", safe: "green", infeasible: "grey" };
  var varLines = new T.Group(); varLines.visible = false; scene.add(varLines);
  var varGhost = null, varMats = {};
  function computeVariants(done) {
    if (VAR) { done(); return; }
    var list = [], jobs = []; DURS.forEach(function (d) { T0S.forEach(function (t0) { jobs.push([t0, d]); }); });
    var i = 0;
    (function chunk() {
      var end = Math.min(jobs.length, i + 8);
      for (; i < end; i++) { var j = jobs[i], Rr = S.rollout({ ego: "planner", others: "reactive", params: { cutT0: j[0], cutDur: j[1] } }); list.push({ t0: j[0], dur: j[1], R: Rr, out: outcome(Rr, j[1]) }); }
      var b = $("varProg"); if (b) b.textContent = "đang chạy " + i + " / " + jobs.length + " kịch bản…";
      if (i < jobs.length) setTimeout(chunk, 0); else { VAR = list; buildVariantViz(); done(); }
    })();
  }
  function buildVariantViz() {
    while (varLines.children.length) varLines.remove(varLines.children[0]);
    var c2 = ACT.findIndex(function (a) { return a.id === "C2"; });
    ["red", "gold", "green", "grey"].forEach(function (k) { varMats[k] = mat(k, { transparent: true, opacity: k === "grey" ? 0.25 : 0.55 }, T.LineBasicMaterial); });
    VAR.forEach(function (v) {
      var p = []; for (var f = 0; f < NF; f += 2) { var q = pose(v.R, c2, f * DT); p.push(q.x, 0.25, q.z); }
      var g = new T.BufferGeometry(); g.setAttribute("position", new T.Float32BufferAttribute(p, 3));
      varLines.add(new T.Line(g, varMats[OUT_COL[v.out]]));
    });
    varGhost = new T.InstancedMesh(BOX, new T.MeshLambertMaterial({ color: 0xffffff, transparent: true, opacity: 0.38 }), VAR.length);
    varGhost.visible = false; scene.add(varGhost);
    colorGhosts(); onTheme.push(colorGhosts);
  }
  function colorGhosts() { if (!varGhost) return; VAR.forEach(function (v, i) { varGhost.setColorAt(i, PAL[OUT_COL[v.out]]); }); varGhost.instanceColor.needsUpdate = true; }
  function updateGhosts(t) {
    if (!varGhost || !st.variants) { if (varGhost) varGhost.visible = false; return; }
    var c2 = ACT.findIndex(function (a) { return a.id === "C2"; }), o = new T.Object3D();
    VAR.forEach(function (v, i) { var q = pose(v.R, c2, t); o.position.set(q.x, 0.8, q.z); o.rotation.set(0, Math.atan2(-q.hz, q.hx), 0); o.scale.set(4.5, 1.4, 1.86); o.updateMatrix(); varGhost.setMatrixAt(i, o.matrix); });
    varGhost.instanceMatrix.needsUpdate = true; varGhost.visible = true;
  }

  // ---------------------------------------------------------------- weather: rain particles
  var RAIN_N = 4000, rainGeo = new T.BufferGeometry(), rainPos = new Float32Array(RAIN_N * 6);
  (function () { var r = rng(3); for (var i = 0; i < RAIN_N; i++) { var x = (r() - .5) * 120, y = r() * 40, z = (r() - .5) * 80; rainPos.set([x, y, z, x - 0.15, y - 0.9, z], i * 6); } })();
  rainGeo.setAttribute("position", new T.BufferAttribute(rainPos, 3));
  var rain = new T.LineSegments(rainGeo, mat("blue", { transparent: true, opacity: 0.35 }, T.LineBasicMaterial)); rain.frustumCulled = false; rain.visible = false; scene.add(rain);
  function stepRain(dt, cx, cz) {
    if (!env.rain) { rain.visible = false; return; } rain.visible = true; rain.position.set(cx, 0, cz);
    var p = rainPos; for (var i = 0; i < RAIN_N; i++) { var o = i * 6; p[o + 1] -= 28 * dt; p[o + 4] -= 28 * dt; if (p[o + 4] < 0) { p[o + 1] += 40; p[o + 4] += 40; } }
    rainGeo.attributes.position.needsUpdate = true;
  }

  // ---------------------------------------------------------------- Real2Sim: accumulate the logged drive into splats, show unobserved surfaces
  var REC = null;
  var splatGeo = new T.BufferGeometry(), splat = new T.Points(splatGeo, new T.PointsMaterial({ size: 0.42, vertexColors: true, map: dotTex, alphaTest: 0.35, transparent: true, opacity: 0.95 }));
  splat.frustumCulled = false; splat.visible = false; scene.add(splat);
  var holeGeo = new T.BufferGeometry(), holes = new T.Points(holeGeo, mat("red", { size: 0.32, map: dotTex, alphaTest: 0.35 }, T.PointsMaterial));
  holes.frustumCulled = false; holes.visible = false; scene.add(holes);
  var recPath = null;
  function reconstruct(done) {
    if (REC) { done(); return; }
    var ei = EGO_I, frames = []; for (var f = 0; f < NF; f += 10) frames.push(f);
    var P = [], C = [], k = 0, grid = {};
    var stat = STATIC_BOX.filter(function (b) { return true; });
    recPath = frames.map(function (f) { return pose(LOG, ei, f * DT); });
    (function chunk() {
      var end = Math.min(frames.length, k + 3);
      for (; k < end; k++) {
        var e = recPath[k], ox = e.x - 0.25 * e.hx, oz = e.z - 0.25 * e.hz, oy = 1.95;
        var near = stat.filter(function (b) { return Math.abs(b.cx - ox) < 90 + b.l && Math.abs(b.cz - oz) < 90 + b.w; });
        for (var az = 0; az < 240; az++) {
          var th = az * 2 * Math.PI / 240, ca = Math.cos(th), sa = Math.sin(th);
          for (var b = 0; b < 24; b++) {
            var el = (-25 + 33 * b / 23) * Math.PI / 180, ce = Math.cos(el), dx = ce * ca, dy = Math.sin(el), dz = ce * sa;
            var h = castRay(ox, oy, oz, dx, dy, dz, near, 80); if (!h) continue;
            if (h.hit === "ground" && (az + b) % 3) continue;
            var x = ox + dx * h.t, y = oy + dy * h.t, z = oz + dz * h.t; P.push(x, y, z);
            var key = h.hit === "ground" ? (Math.abs(z) < 7 ? "sc-road" : "sc-walk") : h.hit.kind === "glass" ? "sc-glass" : h.hit.parked ? h.hit.color : h.hit.src && h.hit.src.tone ? "sc-bldg" : "sc-bldg2";
            C.push(key);
            var gk = Math.floor(x) + "," + Math.floor(y) + "," + Math.floor(z); grid[gk] = 1;
          }
        }
      }
      var pb = $("recProg"); if (pb) pb.textContent = "đang tái dựng " + Math.round(100 * k / frames.length) + "%";
      if (k < frames.length) { setTimeout(chunk, 0); return; }
      REC = { P: new Float32Array(P), keys: C, grid: grid };
      splatGeo.setAttribute("position", new T.BufferAttribute(REC.P, 3)); splatGeo.setAttribute("color", new T.BufferAttribute(new Float32Array(P.length), 3));
      colorSplats(); onTheme.push(colorSplats);
      buildHoles(); done();
    })();
  }
  function colorSplats() {
    if (!REC) return; var c = splatGeo.attributes.color.array, darkK = isDark() ? 1.3 : 0.78;
    var dk = isDark(), wt = new T.Color(dk ? 0x9fb4c4 : 0x000000), tc = new T.Color();
    REC.keys.forEach(function (k, i) { tc.copy(PAL[k] || PAL["sc-bldg"]).lerp(wt, dk ? 0.38 : 0.22); c[i * 3] = tc.r; c[i * 3 + 1] = tc.g; c[i * 3 + 2] = tc.b; });
    splatGeo.attributes.color.needsUpdate = true;
  }
  var HOLES = null;
  function buildHoles() {
    var H = [];
    function obs(x, y, z) { for (var dx = -1; dx <= 1; dx++) for (var dy = -1; dy <= 1; dy++) for (var dz = -1; dz <= 1; dz++) if (REC.grid[(Math.floor(x) + dx) + "," + (Math.floor(y) + dy) + "," + (Math.floor(z) + dz)]) return true; return false; }
    STATIC_BOX.forEach(function (b) {
      if (Math.abs(b.cx - 60) > 200) return;
      if (b.parked) {     // all four sides and the roof of each parked car
        for (var u = -1; u <= 1.001; u += 0.25) for (var y = 0.3; y <= 1.5; y += 0.3) {
          [[u * b.l, -b.w], [u * b.l, b.w], [-b.l, u * b.w], [b.l, u * b.w]].forEach(function (q) {
            var x = b.cx + q[0] * b.hx - q[1] * b.hz, z = b.cz + q[0] * b.hz + q[1] * b.hx; if (!obs(x, y, z)) H.push(x, y, z); });
        }
        return;
      }
      // building faces that look at the street, and roofs within a 25 m band of the road
      var fz = b.cz > 0 ? b.cz - b.w : b.cz + b.w;
      for (var x = b.cx - b.l; x <= b.cx + b.l; x += 1.2) for (var y2 = 0.6; y2 < b.h; y2 += 1.2) if (!obs(x, y2, fz)) H.push(x, y2, fz);
      for (var x2 = b.cx - b.l; x2 <= b.cx + b.l; x2 += 1.5) for (var z2 = Math.max(b.cz - b.w, -30); z2 <= Math.min(b.cz + b.w, 30); z2 += 1.5) if (!obs(x2, b.h, z2)) H.push(x2, b.h + 0.05, z2);
    });
    HOLES = new Float32Array(H); holeGeo.setAttribute("position", new T.BufferAttribute(HOLES, 3));
  }
  function holeStats() {
    if (!HOLES || !REC) return null;
    var fr = new T.Frustum(), M = new T.Matrix4(), cam = camera.position, tmp = new T.Vector3();
    camera.updateMatrixWorld(); M.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse); fr.setFromProjectionMatrix(M);
    var seenN = 0, holeN = 0, P = REC.P;
    function inView(x, y, z) { tmp.set(x, y, z); return tmp.distanceTo(cam) < 140 && fr.containsPoint(tmp); }
    for (var i = 0; i < P.length; i += 9) if (inView(P[i], P[i + 1], P[i + 2])) seenN++;
    for (var j = 0; j < HOLES.length; j += 3) if (inView(HOLES[j], HOLES[j + 1], HOLES[j + 2])) holeN++;
    return { seen: seenN * 3, holes: holeN };
  }

  // ================================================================== labels (HTML overlay)
  var LBL = [], lblPool = [];
  function label(x, y, z, text, cls) { LBL.push([x, y, z, text, cls]); }
  var vtmp = new T.Vector3();
  function flushLabels() {
    var w = stage.clientWidth, h = stage.clientHeight;
    while (lblPool.length < LBL.length) { var d = document.createElement("div"); labels.appendChild(d); lblPool.push(d); }
    lblPool.forEach(function (d, i) {
      var L = LBL[i]; if (!L) { d.hidden = true; return; }
      vtmp.set(L[0], L[1], L[2]).project(camera);
      if (vtmp.z > 1 || vtmp.x < -1.1 || vtmp.x > 1.1 || vtmp.y < -1.1 || vtmp.y > 1.1) { d.hidden = true; return; }
      d.hidden = false; d.className = "lbl " + (L[4] || ""); if (d.textContent !== L[3]) d.textContent = L[3];
      d.style.left = ((vtmp.x + 1) / 2 * w).toFixed(1) + "px"; d.style.top = ((1 - vtmp.y) / 2 * h).toFixed(1) + "px";
    });
    LBL = [];
  }

  // ================================================================== chapters
  function tog(id, key, text, cls) { return '<label class="tog"><input type="checkbox" id="' + id + '" data-k="' + key + '"' + (st[key] ? " checked" : "") + '><span' + (cls ? ' class="' + cls + '"' : "") + ">" + text + "</span></label>"; }
  var CH = [
    { key: "world", node: [0], short: "1 thế giới",
      cam: { dist: 72, yaw: 2.72, pitch: 0.6, lead: 20 },
      layers: {},
      html: function () { return '<div class="eyebrow">Chương 1 · Thế giới</div><h2>Một con phố, mười sáu tác nhân, hai mươi bốn giây</h2>' +
        '<p>Mọi thứ bạn thấy là một <b>thế giới mô phỏng</b>: bản đồ làn, nhà, cây, và các tác nhân. <span class="c-teal">Xe ta</span> (xe tự lái) đi về phía đông. Quanh nó: <span class="c-purple">một xe sắp cắt làn</span>, một xe bám đuôi, một xe buýt đang đỗ che khuất vạch qua đường, và <span class="c-gold">một người đi bộ</span> sẽ bước ra sau xe buýt.</p>' +
        '<p>Thế giới chạy theo bước thời gian cố định <span class="k">Δt = 0,05 s</span>, hoàn toàn tất định: cùng tham số thì cùng kết quả. Nhờ vậy ta tua đi tua lại được, và so được hai phiên bản của xe ta.</p>' +
        '<div class="try"><b>Thử:</b> kéo thanh thời gian về khoảng 13 s, xoay camera xem xe buýt che người đi bộ thế nào.</div>' +
        '<div class="legend"><span><i style="background:var(--ego)"></i>xe ta</span><span><i style="background:var(--adv)"></i>xe cắt làn</span><span><i style="background:var(--ped)"></i>người đi bộ</span><span><i style="background:var(--bus)"></i>xe buýt</span><span><i style="background:var(--car-1)"></i>xe khác</span></div>'; } },
    { key: "sensors", node: [1], short: "2 cảm biến",
      cam: { dist: 40, yaw: 2.45, pitch: 0.8, lead: 4 },
      layers: { lidar: true, frustums: true, radar: false },
      html: function () { return '<div class="eyebrow">Chương 2 · Cảm biến</div><h2>LiDAR quét, camera nhìn, radar đo vận tốc</h2>' +
        '<p>Mô phỏng phải <b>sinh ra dữ liệu cảm biến</b> từ thế giới. Ở đây LiDAR 32 tia được dựng bằng ray-casting thật: mỗi tia bắn ra, chạm hộp gần nhất (xe, nhà, mặt đường) và để lại một điểm. Quét 5 vòng mỗi giây.</p>' +
        '<p>Simulator ngây thơ trả về <span class="c-yellow">mọi tia</span>. LiDAR thật thì <b>mất tia</b> ở kính, sơn đen, ở xa và khi mưa. Model học trên đám điểm hoàn hảo sẽ hụt hẫng khi gặp đám điểm thật.</p>' +
        '<div class="ctl">' + tog("tLid", "lidar", "LiDAR 32 tia") + tog("tReal", "lidarReal", 'Mô hình mất tia (<span class="c-blue">LiDAR thật</span>)') + tog("tDrop", "showDrop", 'Hiện tia bị mất (<span class="c-red">đỏ</span>)') + tog("tFr", "frustums", "6 camera (vùng nhìn)") + tog("tRad", "radar", "Radar trước") + '</div>' +
        '<div class="read"><span>điểm trong vòng quét</span><b id="rPts">–</b><span>tia mất (vòng gần nhất)</span><b id="rDrop">–</b></div>' +
        '<div class="try"><b>Thử:</b> bật "mất tia", nhìn mặt kính các toà nhà và chiếc xe sơn đen đi sau xe ta.</div>'; } },
    { key: "perception", node: [2], short: "3 nhận thức",
      cam: { dist: 44, yaw: 2.75, pitch: 0.92, lead: 8 },
      layers: { det: true, gt: true, tracker: true, trails: true },
      html: function () { return '<div class="eyebrow">Chương 3 · Nhận thức</div><h2>Hộp phát hiện rung, tracker làm mượt, và cái không thấy</h2>' +
        '<p>Từ dữ liệu cảm biến, perception đặt <span class="c-blue">hộp 3D</span> quanh từng vật. Hộp luôn <b>rung</b> một chút; tracker (ở đây là bộ lọc α–β) làm mượt thành <span class="c-teal">hộp theo dõi</span> và vận tốc.</p>' +
        '<p>Khung <span class="c-grey">nét đứt</span> là <b>sự thật</b> mà chỉ simulator biết. Vật bị che (người đi bộ sau xe buýt) có khung sự thật nhưng không có hộp phát hiện: đó là chỗ model mù.</p>' +
        '<div class="ctl">' + tog("tDet", "det", "Hộp phát hiện") + tog("tGt", "gt", "Sự thật (ground truth)") + tog("tTrk", "tracker", "Tracker α–β") + tog("tTr", "trails", 'Vết: <span class="c-red">đo thô</span> và <span class="c-teal">đã làm mượt</span>') +
        '<div class="sl"><span>Nhiễu vị trí σ</span><output id="oNoise">' + fmt(st.noise, 2) + ' m</output><input type="range" id="sNoise" min="0" max="0.8" step="0.05" value="' + st.noise + '"></div></div>' +
        '<div class="read"><span>vật trong 75 m</span><b id="rIn">–</b><span>đang phát hiện</span><b id="rDet">–</b><span>bị che hoặc quá xa</span><b id="rMiss">–</b></div>' +
        '<div class="try"><b>Thử:</b> kéo σ lên 0,6 m rồi tắt/bật tracker. Tua tới khoảng 12–13 s để thấy người đi bộ “hiện ra” muộn thế nào.</div>'; } },
    { key: "prediction", node: [3], short: "4 dự đoán",
      cam: { dist: 60, yaw: 2.95, pitch: 1.05, lead: 14 },
      layers: { det: true, tracker: true, pred: true, predGT: false },
      html: function () { return '<div class="eyebrow">Chương 4 · Dự đoán</div><h2>Mỗi tác nhân có vài tương lai, mỗi tương lai một xác suất</h2>' +
        '<p>Xe ta không đủ biết hiện tại: nó cần đoán <b>5 giây tới</b>. Với mỗi vật đang theo dõi, ta vẽ vài <span class="c-purple">mode</span> (giữ làn, phanh, chuyển làn; rẽ hay đi thẳng; băng qua hay đứng chờ). Dải càng đậm và loe rộng, xác suất càng cao.</p>' +
        '<p>Simulator có một lợi thế không ai có ngoài đời: nó biết <b>tương lai thật</b>. Bật “tương lai thật” để chấm dự đoán ngay tại chỗ.</p>' +
        '<div class="ctl">' + tog("tPred", "pred", "Các mode dự đoán") + tog("tPgt", "predGT", "Tương lai thật (đường đậm)") + '</div>' +
        '<div class="read"><span>xe cắt làn: mode mạnh nhất</span><b id="rC2">–</b><span>người đi bộ: mode mạnh nhất</span><b id="rP1">–</b></div>' +
        '<div class="try"><b>Thử:</b> tua về 4–6 s: xác suất “chuyển làn” của xe tím tăng vọt khi nó bắt đầu lệch sang.</div>'; } },
    { key: "planning", node: [4, 5], short: "5 kế hoạch",
      cam: { dist: 54, yaw: 2.6, pitch: 0.78, lead: 16 },
      layers: { det: true, tracker: true, pred: true, plan: true },
      html: function () { return '<div class="eyebrow">Chương 5 · Lập kế hoạch</div><h2>Hai mươi quỹ đạo thử, một quỹ đạo được chọn</h2>' +
        '<p>Planner sinh một lưới quỹ đạo ứng viên (5 hướng lệch ngang × 4 mức tăng/giảm tốc), rồi chấm từng cái với các dự đoán: <span class="c-red">đỏ</span> là va chạm, ra khỏi đường hoặc lấn làn ngược chiều, <span class="c-green">xanh lá</span> là an toàn, <span class="c-teal">dải đậm</span> là quỹ đạo rẻ nhất.</p>' +
        '<p>Bật <b>không–thời gian</b>: trục đứng là thời gian. Mỗi xe thành một con sâu bò lên trời; kế hoạch tốt là kế hoạch luồn giữa các con sâu mà không chạm.</p>' +
        '<div class="ctl">' + tog("tPlan", "plan", "Lưới quỹ đạo ứng viên") + tog("tSt", "st", "Không–thời gian (x, t, làn)") + '</div>' +
        '<div class="read"><span>quỹ đạo được chọn</span><b id="rPlan">–</b><span>ứng viên bị loại</span><b id="rBad">–</b></div>' +
        '<p class="note">Planner minh hoạ chấm trên dự đoán; xe ta trong mô phỏng chạy theo luật bám xe IDM cộng phép phòng người đi bộ bị che.</p>'; } },
    { key: "replay", node: [0, 1, 2, 3, 4, 5], sim: true, short: "6 phát lại",
      cam: { dist: 36, yaw: 2.95, pitch: 0.72, lead: -7 },
      layers: { ghostLog: true },
      html: function () { return '<div class="eyebrow">Chương 6 · Phát lại hay phản ứng</div><h2>Phát lại log: xe phía sau không biết ta đã phanh</h2>' +
        '<p>Cách rẻ nhất để thử một phiên bản mới là <b>phát lại log</b>: mọi xe khác chạy đúng như đã ghi. Nhưng phiên bản mới của xe ta thận trọng hơn: nó chậm lại sớm vì vạch qua đường bị che. <span class="c-teal">Bóng mờ</span> là xe ta trong log.</p>' +
        '<p>Xe bám đuôi trong log không hề “thấy” sự thay đổi đó, nên nó <span class="c-red">đâm vào đuôi</span> xe ta. Va chạm này là <b>giả</b>: do phát lại, không do planner. Chuyển sang tác nhân phản ứng (IDM) và va chạm biến mất.</p>' +
        '<div class="seg" role="group" aria-label="Chế độ"><button type="button" data-mode="replay">Phát lại log</button><button type="button" data-mode="reactive">Tác nhân phản ứng</button></div>' +
        '<div class="ctl">' + tog("tGh", "ghostLog", "Bóng xe ta trong log") + '</div>' +
        '<div class="read"><span>va chạm trong lần chạy</span><b id="rEv">–</b><span>khoảng cách nhỏ nhất xe bám đuôi → xe ta</span><b id="rTg">–</b></div>' +
        '<div class="try"><b>Thử:</b> ở chế độ phát lại, tua tới khoảng 14–15 s và xoay ra phía sau xe ta.</div>'; } },
    { key: "scenarios", node: [0, 1, 2, 3, 4, 5], sim: true, short: "7 kịch bản",
      cam: { dist: 66, yaw: 2.8, pitch: 0.98, lead: 16 },
      layers: { variants: true },
      html: function () {
        return '<div class="eyebrow">Chương 7 · Sinh kịch bản</div><h2>Sáu mươi lăm vũ trụ song song của một cú cắt làn</h2>' +
        '<p>Thay vì chờ cú cắt làn nguy hiểm xảy ra ngoài đời, ta <b>quét tham số</b>: thời điểm bắt đầu (cột) × thời gian chuyển làn (hàng). Mỗi ô là một lần chạy đầy đủ với xe ta phản ứng. Các bóng xe tím trên đường là 65 phiên bản cùng lúc.</p>' +
        '<p><span class="c-red">Va chạm</span> và <span class="c-gold">suýt va</span> là dữ liệu quý. Ô <span class="c-grey">xám</span> đòi gia tốc ngang quá μg: vật lý không cho phép, phải loại, dù trông rất “nguy hiểm”.</p>' +
        '<div id="varProg" class="note">chuẩn bị kịch bản…</div><div class="grid7" id="varGrid"></div>' +
        '<div class="legend"><span><i style="background:var(--red)"></i>va chạm</span><span><i style="background:var(--gold)"></i>suýt va (&lt; 3 m)</span><span><i style="background:var(--green)"></i>an toàn</span><span><i style="background:var(--grey)"></i>bất khả thi</span></div>' +
        '<div class="ctl">' + tog("tVar", "variants", "Hiện 65 phiên bản trên đường") + '</div>' +
        '<div class="eyebrow" style="margin-top:4px">Ngẫu nhiên hoá điều kiện</div>' +
        '<div class="sl"><span>Giờ trong ngày</span><output id="oTod">–</output><input type="range" id="sTod" min="0.05" max="0.95" step="0.01" value="' + env.tod + '"></div>' +
        '<div class="sl"><span>Sương mù</span><output id="oFog">–</output><input type="range" id="sFog" min="0" max="0.9" step="0.01" value="' + env.fog + '"></div>' +
        '<div class="ctl"><label class="tog"><input type="checkbox" id="tRain"' + (env.rain ? " checked" : "") + '><span>Mưa (mặt đường ướt, LiDAR mất thêm tia)</span></label></div>' +
        '<button class="btn" type="button" id="bRand">Ngẫu nhiên một thế giới</button><div class="chips" id="envChips"></div>';
      } },
    { key: "real2sim", node: [0, 1], sim: true, short: "8 tái dựng",
      cam: { dist: 44, yaw: 2.65, pitch: 0.5, lead: 10 },
      layers: { splat: true, splatOnly: true, holes: false },
      html: function () { return '<div class="eyebrow">Chương 8 · Tái dựng (Real2Sim)</div><h2>Biến log thành thế giới, rồi lái lệch khỏi nó</h2>' +
        '<p>Thay vì dựng tay từng toà nhà, ta <b>tái dựng</b> thế giới từ chính các lần quét trên quãng đường đã ghi: mỗi điểm thành một <i>splat</i> có màu. Đây là ý tưởng của NeRF và Gaussian splatting, rút gọn.</p>' +
        '<p>Đi đúng quỹ đạo cũ thì bản tái dựng trông gần như thật. <b>Lệch khỏi quỹ đạo</b> (sang làn khác, nâng camera) thì lộ ra chỗ chưa ai quét: nóc nhà, mặt sau xe đỗ. Ở đó mô hình phải bịa.</p>' +
        '<div id="recProg" class="note">chuẩn bị…</div>' +
        '<div class="ctl">' + tog("tSp", "splatOnly", "Chỉ hiện bản tái dựng") + tog("tHo", "holes", 'Tô <span class="c-red">bề mặt chưa được quét</span>') + '</div>' +
        '<div class="sl"><span>Camera lệch ngang khỏi quỹ đạo</span><output id="oNov">0 m</output><input type="range" id="sNov" min="0" max="10" step="0.5" value="' + st.novel + '"></div>' +
        '<div class="sl"><span>Độ cao camera</span><output id="oNovH">1,6 m</output><input type="range" id="sNovH" min="1.6" max="20" step="0.2" value="' + st.novelH + '"></div>' +
        '<div class="read"><span>ô bề mặt chưa quét trong khung hình</span><b id="rHole">–</b></div>' +
        '<div class="try"><b>Thử:</b> bật “tô bề mặt chưa quét”, rồi nâng camera lên 15 m.</div>'; } },
    { key: "lab", node: [0, 1, 2, 3, 4, 5], sim: true, short: "9 phòng lab",
      cam: { dist: 46, yaw: 2.7, pitch: 0.7, lead: 8 },
      layers: { labGhost: true },
      html: function () { return '<div class="eyebrow">Chương 9 · Phòng lab</div><h2>Mở kết quả tìm kiếm của bạn</h2>' +
        '<p>Trong lab, bạn tìm kịch bản nguy hiểm bằng Python (<span class="k">avsim</span>) rồi xuất <span class="k">evidence/viewer.json</span>. Thả file vào đây: trình duyệt <b>chạy lại</b> từng kịch bản bằng đúng lõi mô phỏng này, khớp từng bit với Python.</p>' +
        '<label class="drop" id="labDrop"><input type="file" id="labFile" accept=".json,application/json"><span>Thả <b>viewer.json</b> vào đây, hoặc bấm để chọn file</span></label>' +
        '<div class="note" id="labSrc">–</div>' +
        '<div class="plotbox"><div class="plothead"><select id="labX" aria-label="Trục ngang"></select><span>×</span><select id="labY" aria-label="Trục dọc"></select></div>' +
        '<canvas id="labPlot" width="640" height="440" aria-label="Các kịch bản đã thử"></canvas></div>' +
        '<div class="legend"><span><i style="background:var(--red)"></i>va chạm</span><span><i style="background:var(--gold)"></i>suýt va</span><span><i style="background:var(--grey)"></i>an toàn</span><span><i style="background:var(--ink-3);border-radius:50%"></i>đậm dần theo vòng tìm kiếm</span></div>' +
        '<div class="runs" id="labRuns"></div>' +
        '<div class="read" id="labRead"><span>kịch bản</span><b>–</b></div>' +
        '<div class="ctl">' + tog("tLabG", "labGhost", 'Bóng xe ta với <span class="c-teal">chính sách gốc</span>') + '</div>' +
        '<p class="note">Bấm một chấm bất kỳ để chạy kịch bản đó. Bóng mờ là xe ta khi chạy chính sách mặc định trên cùng kịch bản: so sánh để thấy bản sửa của bạn thay đổi gì.</p>'; } },
    { key: "studio", node: [0, 4, 5], sim: true, short: "10 xưởng",
      cam: { dist: 58, yaw: 2.75, pitch: 0.82, lead: 14 },
      layers: { stuGhost: true },
      html: function () { return '<div class="eyebrow">Chương 10 · Xưởng kịch bản</div><h2>Bạn dựng rắc rối, hoặc bạn cầm lái</h2>' +
        '<p>Cuộc săn trong lab chỉ tìm trong những gì thế giới đã có. <b>Rắc rối nào chưa ai nghĩ tới?</b> Thêm một người băng ngang, một xe chết máy, một cú phanh gấp, một xe đạp lạng ra; chỉnh thế giới và planner; hoặc tự lái để xem người lái làm gì khác planner.</p>' +
        '<div class="eyebrow" style="margin-top:2px">Rắc rối thêm vào</div>' +
        '<div class="hzadd">' + STU_TYPES.map(function (t) { return '<button class="btn" type="button" data-add="' + t[0] + '">+ ' + t[1] + "</button>"; }).join("") + "</div>" +
        '<div id="stuHz"></div>' +
        '<details class="stu"><summary>Thế giới (tham số kịch bản)</summary><div id="stuWorld" class="stugrid"></div></details>' +
        '<details class="stu"><summary>Planner (chính sách của xe ta)</summary><div id="stuPol" class="stugrid"></div></details>' +
        '<div class="eyebrow" style="margin-top:4px">Ai cầm lái?</div>' +
        '<div class="seg" role="group" aria-label="Ai cầm lái"><button type="button" data-drive="0">Planner</button><button type="button" data-drive="1">Bạn</button></div>' +
        '<div id="stuDrive" class="pedals" hidden><button type="button" class="btn" data-pedal="1.5">▲ ga</button><button type="button" class="btn" data-pedal="-6">▼ phanh</button>' +
        '<button type="button" class="btn" id="stuRec">↺ lái lại từ đầu</button><p class="note">Giữ <span class="k">↑</span> / <span class="k">W</span> để tăng tốc, <span class="k">↓</span> / <span class="k">S</span> để phanh; thả ra thì giữ tốc độ. Mỗi lần bạn đổi chân ga, thế giới tính lại phần còn lại.</p></div>' +
        '<div class="read" id="stuRead"><span>kết quả</span><b>–</b></div>' +
        '<div class="ctl">' + tog("tStuG", "stuGhost", 'Bóng mờ: <span class="c-teal" id="stuGhostWho">planner gốc</span>') + '</div>' +
        '<div class="eyebrow" style="margin-top:4px">Mang sang Python</div>' +
        '<div class="hzadd"><button class="btn" type="button" id="stuSave">Tải hazard.json</button><button class="btn" type="button" id="stuCopy">Sao chép</button>' +
        '<label class="btn" style="position:relative">Mở file…<input type="file" id="stuOpen" accept=".json,application/json" style="position:absolute;inset:0;opacity:0;cursor:pointer"></label></div>' +
        '<textarea id="stuJson" readonly rows="5" aria-label="Kịch bản dạng JSON"></textarea>' +
        '<p class="note">Trong lab: <span class="k">python run_all.py mine hazard.json</span> chạy đúng kịch bản này bằng avsim (khớp từng bit), rồi săn và đo tần suất của rắc rối bạn vừa dựng.</p>'; } }
  ];

  // ---------------------------------------------------------------- chapter switching
  var nav = $("chapters"), bodyEl = $("body");
  CH.forEach(function (c, i) { var b = document.createElement("button"); b.type = "button"; b.textContent = c.short; b.addEventListener("click", function () { go(i); }); nav.appendChild(b); });
  var LAYER_KEYS = ["lidar", "lidarReal", "frustums", "radar", "det", "gt", "tracker", "trails", "pred", "predGT", "plan", "st", "ghostLog", "variants", "splat", "splatOnly", "holes", "labGhost", "stuGhost"];
  function go(i) {
    i = clamp(i, 0, CH.length - 1); var c = CH[i], prev = CH[st.ch];
    st.ch = i;
    LAYER_KEYS.forEach(function (k) { st[k] = !!c.layers[k]; });
    if (c.key === "replay") setMode("replay"); else if (prev && prev.key === "replay") setMode("reactive");
    if (c.key !== "scenarios" && st.variant) { st.variant = null; setRun(RUN[st.mode]); }
    if (c.key !== "lab" && LAB.cur) { LAB.cur = null; LAB.ghost = null; setRun(RUN[st.mode]); }
    if (c.key !== "studio" && STU.on) { STU.on = false; STU.keys = {}; clearTimeout(STU.timer); setRun(RUN[st.mode]); }
    if (c.key !== "real2sim") { rig.fp = null; }
    if (c.key !== "scenarios" && prev && prev.key === "scenarios") { Object.assign(env, ENV0); applyEnv(); }
    nav.querySelectorAll("button").forEach(function (b, j) { if (j === i) b.setAttribute("aria-current", "step"); else b.removeAttribute("aria-current"); }); var cb = nav.children[i]; if (cb && nav.scrollWidth > nav.clientWidth) nav.scrollLeft = cb.offsetLeft - 24;
    document.querySelectorAll(".loop .node").forEach(function (n) { n.classList.toggle("on", c.node.indexOf(+n.dataset.n) >= 0); });
    $("simframe").classList.toggle("on", !!c.sim);
    bodyEl.innerHTML = c.html() + '<div class="navrow"><button class="btn" type="button" id="prevCh"' + (i === 0 ? " disabled" : "") + '>← Chương trước</button><button class="btn" type="button" id="nextCh"' + (i === CH.length - 1 ? " disabled" : "") + ">Chương sau →</button></div>";
    bodyEl.scrollTop = 0; $("toast").hidden = true; toastT = 0;
    bind(c);
    setFollow(true); rig.lead = c.cam.lead || 0; camTo({ dist: c.cam.dist, yaw: c.cam.yaw, pitch: c.cam.pitch });
    if (c.key === "real2sim") { reconstruct(function () { var pb = $("recProg"); if (pb) pb.textContent = "bản tái dựng: " + Math.round(REC.P.length / 3).toLocaleString("vi-VN") + " splat từ " + recPath.length + " vị trí quét trên log"; }); }
    if (c.key === "scenarios") computeVariants(function () { drawVarGrid(); var b = $("varProg"); if (b) b.textContent = "Bấm một ô để chạy kịch bản đó."; });
    if (c.key !== "sensors") lidarClear();
    stLastF = -99;
    try { history.replaceState(null, "", "#" + c.key); } catch (e) {}
  }
  function bind(c) {
    bodyEl.querySelectorAll("input[type=checkbox][data-k]").forEach(function (el) { el.addEventListener("change", function () { st[el.dataset.k] = el.checked; if (el.dataset.k === "lidar" && !el.checked) lidarClear(); if (el.dataset.k === "lidarReal") lidarClear(); stLastF = -99;
      if (el.dataset.k === "st") { if (el.checked) { rig.lead = 14; camTo({ dist: 74, yaw: 1.72, pitch: 0.3 }, 1400); } else { rig.lead = c.cam.lead || 0; camTo({ dist: c.cam.dist, yaw: c.cam.yaw, pitch: c.cam.pitch }, 1200); } } }); });
    var pc = $("prevCh"), nc = $("nextCh"); function toTop() { if (window.innerWidth <= 860) try { window.scrollTo({ top: 0, behavior: REDUCED ? "auto" : "smooth" }); } catch (e) {} }
    if (pc) pc.addEventListener("click", function () { go(st.ch - 1); toTop(); }); if (nc) nc.addEventListener("click", function () { go(st.ch + 1); toTop(); });
    var sn = $("sNoise"); if (sn) sn.addEventListener("input", function () { st.noise = +sn.value; $("oNoise").textContent = fmt(st.noise, 2) + " m"; });
    bodyEl.querySelectorAll("[data-mode]").forEach(function (b) { b.setAttribute("aria-pressed", b.dataset.mode === st.mode); b.addEventListener("click", function () { setMode(b.dataset.mode); bodyEl.querySelectorAll("[data-mode]").forEach(function (x) { x.setAttribute("aria-pressed", x.dataset.mode === st.mode); }); }); });
    if (c.key === "scenarios") {
      var tod = $("sTod"), fog = $("sFog"), rn = $("tRain");
      function envOut() { $("oTod").textContent = todText(env.tod); $("oFog").textContent = Math.round(env.fog * 100) + "%"; tod.value = env.tod; fog.value = env.fog; rn.checked = env.rain; envChips(); }
      tod.addEventListener("input", function () { env.tod = +tod.value; applyEnv(); envOut(); });
      fog.addEventListener("input", function () { env.fog = +fog.value; applyEnv(); envOut(); });
      rn.addEventListener("change", function () { env.rain = rn.checked; env.wet = env.rain ? 1 : 0; applyEnv(); envOut(); });
      $("bRand").addEventListener("click", function () { var r = Math.random; env.tod = 0.08 + r() * 0.84; env.fog = r() < 0.5 ? r() * 0.25 : 0.25 + r() * 0.6; env.rain = r() < 0.4; env.wet = env.rain ? 1 : 0; applyEnv(); envOut(); });
      envOut();
    }
    if (c.key === "lab") labBind();
    if (c.key === "studio") stuBind();
    if (c.key === "real2sim") {
      var nv = $("sNov"), nh = $("sNovH");
      nv.addEventListener("input", function () { st.novel = +nv.value; $("oNov").textContent = fmt(st.novel) + " m"; });
      nh.addEventListener("input", function () { st.novelH = +nh.value; $("oNovH").textContent = fmt(st.novelH) + " m"; });
    }
  }
  function todText(v) { var h = 4 + v * 16, hh = Math.floor(h), mm = Math.round((h - hh) * 60); return (hh < 10 ? "0" : "") + hh + ":" + (mm < 10 ? "0" : "") + mm; }
  function envChips() { var c = $("envChips"); if (!c) return; c.innerHTML = "<span>" + todText(env.tod) + "</span><span>sương " + Math.round(env.fog * 100) + "%</span><span>" + (env.rain ? "mưa, đường ướt" : "khô") + "</span>"; }
  function drawVarGrid() {
    var g = $("varGrid"); if (!g || !VAR) return;
    var h = '<span></span>' + T0S.map(function (t, i) { return '<span class="hx">' + (i % 2 ? "" : fmt(t, 0)) + "</span>"; }).join("");
    DURS.forEach(function (d) {
      h += "<span>" + fmt(d) + " s</span>";
      T0S.forEach(function (t0) { var v = VAR.find(function (x) { return x.t0 === t0 && x.dur === d; });
        var sel = st.variant === v;
        h += '<button type="button" data-t0="' + t0 + '" data-d="' + d + '" aria-pressed="' + sel + '" title="bắt đầu ' + fmt(t0) + ' s, chuyển làn ' + fmt(d) + ' s" style="background:var(--' + OUT_COL[v.out] + ')' + (v.out === "infeasible" ? ";opacity:.45" : "") + '"></button>'; });
    });
    h += '<span></span><span class="hx" style="grid-column:2 / -1;text-align:left">thời điểm bắt đầu cắt làn (s) →</span>';
    g.innerHTML = h;
    g.querySelectorAll("button").forEach(function (b) { b.addEventListener("click", function () {
      var v = VAR.find(function (x) { return x.t0 === +b.dataset.t0 && x.dur === +b.dataset.d; }); st.variant = v; setRun(v.R); st.t = Math.max(0, v.t0 - 1.5); st.playing = true; drawVarGrid(); }); });
  }
  function setMode(m) { st.mode = m; if (!st.variant) setRun(RUN[m]); }
  function setRun(Rr) { R = Rr; TRK = {}; lastPF = -1; stLastF = -99; rebuildHits(); }
  function setFollow(on) { rig.follow = on; var b = $("camBtn"); b.setAttribute("aria-pressed", on); b.textContent = on ? "theo xe" : "tự do"; }

  // ---------------------------------------------------------------- chapter 9: the lab's own runs, re-simulated here
  var LAB = { data: null, src: "", cur: null, ghost: null, sel: -1, xi: 0, yi: 1 };
  var LAB_LABEL = { bus_bypass: "lách qua xe buýt", cut_in: "cắt làn gấp", crosswalk_block: "chặn lối người đi bộ", rear_ended: "bị tông đuôi", other: "khác", mine: "rắc rối của bạn", replay: "chỉ va chạm khi phát lại log" };
  var LAB_OUT = { crash: "red", near: "gold", safe: "grey", infeasible: "grey" };
  function labValidate(d) {
    if (!d || d.format !== "avsim-viewer/1") throw new Error("File không phải avsim-viewer/1 (thiếu trường format).");
    if (!Array.isArray(d.runs)) throw new Error("File thiếu danh sách runs.");
    if (d.cloud && (!Array.isArray(d.cloud.names) || !Array.isArray(d.cloud.rows))) throw new Error("cloud phải có names và rows.");
    return d;
  }
  function labLoad(d, src) {
    LAB.data = labValidate(d); LAB.src = src; LAB.sel = -1;
    var names = (d.cloud && d.cloud.names) || [];
    LAB.xi = Math.max(0, names.indexOf(d.cloud && d.cloud.x || "cutT0")); LAB.yi = Math.max(0, names.indexOf(d.cloud && d.cloud.y || "cutGap"));
    if (LAB.yi === LAB.xi) LAB.yi = Math.min(names.length - 1, LAB.xi + 1);
    if (CH[st.ch].key === "lab") labRender();
  }
  function labParamsFromCloud(i) {
    var c = LAB.data.cloud, p = {}; c.names.forEach(function (n, j) { p[n] = c.rows[i][j]; });
    Object.assign(p, c.fixed || {}); return p;
  }
  function labPlay(params, policy, others, label, hazards) {
    var pol = Object.assign({}, S.POLICY, LAB.data && LAB.data.policy || {}, policy || {});
    var log = others === "replay" ? S.rollout({ ego: "human", others: "reactive", params: params, hazards: hazards }) : null;
    var Rr = S.rollout({ ego: "planner", others: others || "reactive", params: params, policy: pol, log: log, hazards: hazards });
    LAB.cur = { params: params, policy: pol, label: label, R: Rr, hazards: hazards || [] };
    // a run pinned to a policy (hunt crashes: found under the baseline) shows the student's fix as the ghost; else the ghost is the baseline
    var gpol = policy ? (LAB.data && LAB.data.policy || {}) : (LAB.data && LAB.data.baseline_policy || {});
    LAB.ghost = S.rollout({ ego: "planner", others: others || "reactive", params: params, policy: Object.assign({}, S.POLICY, gpol), log: log, hazards: hazards });
    setRun(Rr);
    var ev = Rr.events.filter(function (e) { return e.a === "ego" || e.b === "ego"; })[0];
    st.t = ev ? Math.max(0, ev.t - 4) : 0; st.playing = true; syncPlay();
    labReadout();
  }
  function labReadout() {
    var el = $("labRead"); if (!el || !LAB.cur) return;
    var c = LAB.cur, p = c.params, ev = c.R.events.filter(function (e) { return e.a === "ego" || e.b === "ego"; })[0];
    var g = LAB.ghost.events.filter(function (e) { return e.a === "ego" || e.b === "ego"; })[0];
    function row(k, v) { return "<span>" + k + "</span><b>" + v + "</b>"; }
    el.innerHTML = row("kiểu", c.label ? (LAB_LABEL[c.label] || c.label) : "–") +
      row("chính sách của bạn", ev ? '<span class="c-red">' + evWord(ev) + ' ' + who(ev.a === "ego" ? ev.b : ev.a) + " " + fmt(ev.t) + " s</span>" : '<span class="c-green">không va chạm</span>') +
      row("chính sách gốc", g ? evWord(g) + " " + fmt(g.t) + " s" : "không va chạm") +
      row("mặt đường", p.mu < 0.65 ? "ướt (μ " + fmt(p.mu, 1) + ")" : "khô") +
      row("khe chấp nhận khi cắt làn", p.cutGap == null ? "–" : fmt(p.cutGap) + " m") +
      row("người đi bộ ngó qua xe buýt", p.pedLook ? "có" : "không") +
      (c.hazards.length ? row("rắc rối thêm vào", c.hazards.map(function (h, k) { return "H" + k + " " + (STU_TYPES.filter(function (t) { return t[0] === h.type; })[0] || [0, h.type])[1]; }).join(", ")) : "");
  }
  function labRender() {
    var src = $("labSrc"); if (!src) return;
    if (!LAB.data) { src.textContent = "Chưa có dữ liệu."; return; }
    var d = LAB.data, c = d.cloud;
    src.textContent = "Đang xem: " + LAB.src + (d.title ? " · " + d.title : "") + " · " + d.runs.length + " kịch bản lưu" + (c ? " · " + c.rows.length.toLocaleString("vi-VN") + " điểm đã thử" : "");
    var xs = $("labX"), ys = $("labY");
    if (c) {
      var opts = c.names.map(function (n, j) { return '<option value="' + j + '">' + n + "</option>"; }).join("");
      xs.innerHTML = opts; ys.innerHTML = opts; xs.value = LAB.xi; ys.value = LAB.yi;
    }
    var runs = $("labRuns");
    runs.innerHTML = d.runs.map(function (r, i) {
      return '<button type="button" data-i="' + i + '" aria-pressed="' + (LAB.sel === i) + '"><i style="background:var(--' + (r.outcome ? LAB_OUT[r.outcome] || "grey" : "red") + ')"></i>' +
        (r.id ? r.id + " · " : "") + (LAB_LABEL[r.label] || r.label || "kịch bản") + "</button>"; }).join("");
    runs.querySelectorAll("button").forEach(function (b) { b.addEventListener("click", function () { var r = d.runs[+b.dataset.i]; LAB.sel = +b.dataset.i; labPlay(r.params, r.policy, r.others, r.label, r.hazards); labRender(); }); });
    labPlot(); labReadout();
  }
  function labPlot() {
    var cv = $("labPlot"); if (!cv || !LAB.data || !LAB.data.cloud) { if (cv) cv.getContext("2d").clearRect(0, 0, cv.width, cv.height); return; }
    var c = LAB.data.cloud, x = cv.getContext("2d"), W = cv.width, H = cv.height, m = 34, xi = LAB.xi, yi = LAB.yi;
    var cs = getComputedStyle(document.documentElement), col = function (k) { return cs.getPropertyValue("--" + k).trim(); };
    var xv = c.rows.map(function (r) { return r[xi]; }), yv = c.rows.map(function (r) { return r[yi]; });
    var x0 = Math.min.apply(null, xv), x1 = Math.max.apply(null, xv), y0 = Math.min.apply(null, yv), y1 = Math.max.apply(null, yv);
    if (x1 === x0) x1 = x0 + 1; if (y1 === y0) y1 = y0 + 1;
    var sx = function (v) { return m + (v - x0) / (x1 - x0) * (W - m - 10); }, sy = function (v) { return H - m - (v - y0) / (y1 - y0) * (H - m - 10); };
    LAB.map = { sx: sx, sy: sy };
    x.clearRect(0, 0, W, H); x.font = "600 18px " + cs.getPropertyValue("--f-mono"); x.fillStyle = col("ink-3"); x.strokeStyle = col("line"); x.lineWidth = 1.5;
    x.beginPath(); x.moveTo(m, 6); x.lineTo(m, H - m); x.lineTo(W - 6, H - m); x.stroke();
    x.textAlign = "left"; x.fillText(fmt(x0, 1), m, H - 10); x.textAlign = "right"; x.fillText(fmt(x1, 1) + "  " + c.names[xi], W - 6, H - 10);
    x.save(); x.translate(18, H - m); x.rotate(-Math.PI / 2); x.textAlign = "left"; x.fillText(fmt(y0, 1), 0, 0); x.textAlign = "right"; x.fillText(c.names[yi] + "  " + fmt(y1, 1), H - m - 6, 0); x.restore();
    var itMax = Math.max(1, Math.max.apply(null, c.it || [0])), order = c.rows.map(function (_, i) { return i; });
    var rank = { safe: 0, infeasible: 0, near: 1, crash: 2 };
    order.sort(function (a, b) { return (rank[c.out[a]] || 0) - (rank[c.out[b]] || 0); });
    order.forEach(function (i) {
      var o = c.out[i], it = c.it ? c.it[i] : 0, a = 0.18 + 0.7 * it / itMax;
      x.globalAlpha = o === "crash" ? 0.95 : o === "near" ? 0.85 : a * 0.6;
      x.fillStyle = col(LAB_OUT[o] || "grey");
      x.beginPath(); x.arc(sx(xv[i]), sy(yv[i]), o === "crash" ? 5 : o === "near" ? 4 : 2.6, 0, 7); x.fill();
      if (o === "infeasible") { x.globalAlpha = 0.6; x.strokeStyle = col("ink-3"); x.stroke(); }
    });
    x.globalAlpha = 1;
    if (LAB.cur) { x.strokeStyle = col("ink"); x.lineWidth = 2.5; x.beginPath(); x.arc(sx(LAB.cur.params[c.names[xi]]), sy(LAB.cur.params[c.names[yi]]), 9, 0, 7); x.stroke(); }
  }
  function labBind() {
    var f = $("labFile"), drop = $("labDrop"), cv = $("labPlot");
    function readFile(file) {
      var rd = new FileReader();
      rd.onload = function () { try { labLoad(JSON.parse(rd.result), file.name); } catch (err) { $("labSrc").textContent = "Không đọc được " + file.name + ": " + err.message; } };
      rd.readAsText(file);
    }
    f.addEventListener("change", function () { if (f.files[0]) readFile(f.files[0]); });
    ["dragover", "dragenter"].forEach(function (k) { drop.addEventListener(k, function (e) { e.preventDefault(); drop.classList.add("on"); }); });
    ["dragleave", "drop"].forEach(function (k) { drop.addEventListener(k, function () { drop.classList.remove("on"); }); });
    drop.addEventListener("drop", function (e) { e.preventDefault(); if (e.dataTransfer.files[0]) readFile(e.dataTransfer.files[0]); });
    $("labX").addEventListener("change", function () { LAB.xi = +this.value; labPlot(); });
    $("labY").addEventListener("change", function () { LAB.yi = +this.value; labPlot(); });
    cv.addEventListener("click", function (e) {
      if (!LAB.data || !LAB.data.cloud || !LAB.map) return;
      var r = cv.getBoundingClientRect(), px = (e.clientX - r.left) * cv.width / r.width, py = (e.clientY - r.top) * cv.height / r.height, c = LAB.data.cloud, best = -1, bd = 1e9;
      c.rows.forEach(function (row, i) { var dx = LAB.map.sx(row[LAB.xi]) - px, dy = LAB.map.sy(row[LAB.yi]) - py, dd = dx * dx + dy * dy + (c.out[i] === "crash" ? -40 : 0); if (dd < bd) { bd = dd; best = i; } });
      if (best >= 0 && bd < 400) { LAB.sel = -1; labPlay(labParamsFromCloud(best), null, "reactive", c.mode ? c.mode[best] : null); labRender(); }
    });
    if (!LAB.data && window.LAB_SAMPLE) labLoad(window.LAB_SAMPLE, "mẫu của giảng viên");
    labRender();
  }
  onTheme.push(function () { if (CH[st.ch] && CH[st.ch].key === "lab") labPlot(); });

  // ---------------------------------------------------------------- chapter 10: the scenario studio
  // Students add hazards, tune the world and the planner, or drive the ego themselves; the file they save runs
  // unchanged in the lab's Python port (avsim), which matches this sim-core bit for bit.
  var STU_TYPES = [["ped", "người băng ngang"], ["stalled", "xe chết máy"], ["brake", "phanh gấp"], ["bike", "xe đạp lạng ra"]];
  var STU_FIELDS = {
    ped: [["x", "băng qua tại x", 90, 320, 1, "m"], ["side", "từ lề", ["S", "nam (phải)", "N", "bắc (trái)"]], ["v", "tốc độ", 0.6, 3.5, 0.05, "m/s"],
          ["t0", "sớm nhất lúc", 0, 20, 0.1, "s"], ["trig", "khi xe ta còn cách", 0, 80, 1, "m"]],
    stalled: [["x", "đứng tại x", 100, 320, 1, "m"], ["lane", "làn", ["E1", "làn xe ta", "E2", "làn ngoài"]]],
    brake: [["who", "xe nào", ["L1", "xe phía trước", "C2", "xe cắt làn", "E5", "xe làn bên"]], ["decel", "mức phanh", 2, 9, 0.1, "m/s²"],
            ["dur", "trong", 0.5, 5, 0.1, "s"], ["t0", "sớm nhất lúc", 0, 20, 0.1, "s"], ["trig", "khi xe ta còn cách", 0, 80, 1, "m"]],
    bike: [["x", "xuất phát tại x", -100, 300, 1, "m"], ["lane", "làn", ["E2", "làn ngoài", "E1", "làn xe ta"]], ["d", "lệch phải so với tim làn", -1.5, 1.5, 0.05, "m"],
           ["v", "tốc độ", 2, 9, 0.1, "m/s"], ["swerve", "lạng sang trái", 0, 4, 0.05, "m"], ["swDur", "trong", 0.5, 3, 0.05, "s"],
           ["t0", "sớm nhất lúc", 0, 20, 0.1, "s"], ["trig", "khi xe ta còn cách", 0, 80, 1, "m"]]
  };
  var STU_WORLD = [["cutT0", "xe tím bắt đầu tìm khe", 1, 12, 0.1, "s"], ["cutDur", "thời gian chuyển làn", 1, 8, 0.1, "s"], ["cutV", "tốc độ xe tím", 8, 20, 0.1, "m/s"],
    ["cutDs", "xe tím xuất phát so với xe ta", -12, 12, 0.5, "m"], ["cutGap", "khe xe tím chấp nhận", 1, 40, 0.5, "m"], ["cutBrake", "xe tím phanh sau khi vào làn", 0, 8, 0.1, "m/s²"],
    ["pedT0", "người đi bộ tới lề lúc", 4, 18, 0.1, "s"], ["pedV", "tốc độ người đi bộ", 0.6, 2.6, 0.05, "m/s"], ["pedGap", "khe người đi bộ chấp nhận", 0.5, 10, 0.1, "s"],
    ["busGo", "xe buýt rời bến lúc", 10, 24, 0.1, "s"], ["tgT", "xe bám đuôi: cách xe ta", 0.5, 2, 0.05, "s"]];
  var STU_POL = [["v0", "tốc độ mong muốn", 8, 16, 0.1, "m/s"], ["T", "khoảng cách thời gian", 0.8, 2.6, 0.05, "s"], ["cautionV", "tốc độ thận trọng gần vạch bị che", 3, 12.5, 0.1, "m/s"],
    ["cutinLat", "coi xe lấn làn là xe trước khi cách ngang", 1.5, 3.6, 0.05, "m"], ["pedBuffer", "biên an toàn khi đoán người đi bộ", 0, 5, 0.1, "s"]];
  var STU = { on: false, drive: false, keys: {}, params: null, policy: {}, hazards: [], pedal: null, R: null, ghost: null, timer: 0, msg: "", extra: {} };
  STU.params = Object.assign({}, S.DEFAULTS, { civil: true, cutGap: 12, pedGap: 4.5, pedLook: 1, cutT0: 6.5, pedT0: 11, busGo: 17 });
  STU.pedal = new Float32Array(NF);
  function stuNum(v, d) { return v == null ? "tắt" : fmt(+v, d); }
  function stuSlider(id, f, val, dv) {
    var isTrig = f[0] === "trig", v = val == null ? (isTrig ? 0 : dv) : val, d = f[4] < 0.1 ? 2 : f[4] < 1 ? 1 : 0;
    return '<div class="sl"><span>' + f[1] + '</span><output id="o' + id + '">' + (isTrig && !val ? "theo thời gian" : stuNum(v, d) + " " + f[5]) + '</output>' +
      '<input type="range" id="' + id + '" min="' + f[2] + '" max="' + f[3] + '" step="' + f[4] + '" value="' + v + '"></div>';
  }
  function stuSelect(id, f, val) {
    var o = f[2], h = '<div class="sl"><span>' + f[1] + '</span><select id="' + id + '">';
    for (var i = 0; i < o.length; i += 2) h += '<option value="' + o[i] + '"' + (o[i] === val ? " selected" : "") + ">" + o[i + 1] + "</option>";
    return h + "</select></div>";
  }
  function stuHzHtml() {
    if (!STU.hazards.length) return '<p class="note">Chưa có. Bấm một nút ở trên.</p>';
    return STU.hazards.map(function (h, k) {
      var full = S.hazardFull(h), name = STU_TYPES.filter(function (t) { return t[0] === h.type; })[0][1];
      return '<div class="hzcard"><div class="hzhead"><b>H' + k + " · " + name + '</b><button type="button" class="hzdel" data-del="' + k + '" aria-label="Bỏ">×</button></div>' +
        STU_FIELDS[h.type].map(function (f) { var id = "hz" + k + "_" + f[0]; return Array.isArray(f[2]) ? stuSelect(id, f, full[f[0]]) : stuSlider(id, f, full[f[0]], f[2]); }).join("") + "</div>";
    }).join("");
  }
  function stuDoc() {
    var d = { format: "avsim-studio/1", params: {}, policy: {}, hazards: STU.hazards.map(function (h) { return S.hazardFull(h); }), ego: STU.drive ? "script" : "planner" };
    Object.keys(STU.params).forEach(function (k) { var v = STU.params[k]; d.params[k] = v; });   // full precision: the lab replays this exact scene
    Object.keys(STU.policy).forEach(function (k) { d.policy[k] = STU.policy[k]; });
    if (STU.drive) { var last = NF - 1; while (last > 0 && STU.pedal[last] === STU.pedal[last - 1]) last--; d.egoAcc = Array.from(STU.pedal.slice(0, last + 1)); }
    Object.keys(STU.extra).forEach(function (k) { d[k] = STU.extra[k]; });   // space / exposure / why / world, added by hand: kept
    return d;
  }
  function stuRun() {
    if (!STU.on) return;                       // a debounced run that lands after leaving the chapter must not replace RUN
    var pol = Object.assign({}, S.POLICY, STU.policy), hz = STU.hazards.map(function (h) { return S.hazardFull(h); });
    var o = { params: STU.params, policy: pol, hazards: hz, others: "reactive" };
    STU.R = S.rollout(Object.assign({ ego: STU.drive ? "script" : "planner", egoAcc: STU.pedal }, o));
    STU.ghost = S.rollout(Object.assign({ ego: "planner" }, o, STU.drive ? {} : { policy: S.POLICY }));
    setRun(STU.R);
    var g = $("stuGhostWho"); if (g) g.textContent = STU.drive ? "planner lái cùng thế giới" : "planner gốc";
    var ta = $("stuJson"); if (ta) ta.value = JSON.stringify(stuDoc());
    stuReadout(st.t, true);
  }
  function stuLater() { clearTimeout(STU.timer); STU.timer = setTimeout(stuRun, 60); }
  function stuPedal(a) {
    var f = Math.max(0, Math.min(NF - 1, Math.floor(st.t / DT)));
    for (var i = f; i < NF; i++) STU.pedal[i] = a;
    if (!st.playing) { st.playing = true; syncPlay(); }
    stuRun();
  }
  function stuRender() {
    var hz = $("stuHz"); if (!hz) return;
    hz.innerHTML = stuHzHtml();
    hz.querySelectorAll("[data-del]").forEach(function (b) { b.addEventListener("click", function () { STU.hazards.splice(+b.dataset.del, 1); stuRender(); stuRun(); }); });
    STU.hazards.forEach(function (h, k) {
      STU_FIELDS[h.type].forEach(function (f) {
        var el = $("hz" + k + "_" + f[0]); if (!el) return;
        el.addEventListener(el.tagName === "SELECT" ? "change" : "input", function () {
          var v = el.tagName === "SELECT" ? el.value : +el.value;
          if (f[0] === "trig" && v === 0) v = null;
          h[f[0]] = v;
          var out = $("o" + el.id); if (out) out.textContent = f[0] === "trig" && v == null ? "theo thời gian" : stuNum(v, f[4] < 0.1 ? 2 : f[4] < 1 ? 1 : 0) + " " + f[5];
          stuLater();
        });
      });
    });
  }
  function stuGrid(elId, defs, obj, base) {
    var el = $(elId); if (!el) return;
    el.innerHTML = defs.map(function (f) { var v = obj[f[0]] != null ? obj[f[0]] : base[f[0]]; return stuSlider("stu_" + f[0], f, v, v); }).join("") +
      (elId === "stuWorld" ? '<label class="tog"><input type="checkbox" id="stuWet"' + (STU.params.mu < 0.65 ? " checked" : "") + '><span>Đường ướt (μ 0,5)</span></label>' +
                             '<label class="tog"><input type="checkbox" id="stuLook"' + (STU.params.pedLook ? " checked" : "") + '><span>Người đi bộ ngó qua đầu xe buýt</span></label>' : "");
    defs.forEach(function (f) {
      var inp = $("stu_" + f[0]);
      inp.addEventListener("input", function () { obj[f[0]] = +inp.value; $("ostu_" + f[0]).textContent = stuNum(+inp.value, f[4] < 0.1 ? 2 : f[4] < 1 ? 1 : 0) + " " + f[5]; stuLater(); });
    });
    if (elId === "stuWorld") {
      $("stuWet").addEventListener("change", function () { STU.params.mu = this.checked ? 0.5 : 0.8; stuLater(); });
      $("stuLook").addEventListener("change", function () { STU.params.pedLook = this.checked ? 1 : 0; stuLater(); });
    }
  }
  function stuLoad(d) {
    if (!d || d.format !== "avsim-studio/1") throw new Error("không phải avsim-studio/1");
    STU.params = Object.assign({}, S.DEFAULTS, { civil: true }, d.params || {});
    STU.policy = Object.assign({}, d.policy || {});
    var hz = d.hazards || [], ok = hz.filter(function (h) { return STU_FIELDS[h.type]; });
    if (ok.length !== hz.length) throw new Error("loại rắc rối không biết: " + hz.filter(function (h) { return !STU_FIELDS[h.type]; }).map(function (h) { return h.type; }).join(", "));
    if (ok.length > 4) throw new Error(ok.length + " rắc rối; tối đa 4");
    STU.hazards = ok;
    STU.extra = {}; ["space", "exposure", "why", "world"].forEach(function (k) { if (d[k] !== undefined) STU.extra[k] = d[k]; });
    STU.drive = d.ego === "script" && Array.isArray(d.egoAcc);
    STU.pedal = new Float32Array(NF);
    if (STU.drive) for (var i = 0; i < NF; i++) STU.pedal[i] = d.egoAcc[Math.min(i, d.egoAcc.length - 1)] || 0;
  }
  function stuBind() {
    STU.on = true;
    bodyEl.querySelectorAll("[data-add]").forEach(function (b) { b.addEventListener("click", function () {
      if (STU.hazards.length >= 4) { toast("Tối đa 4 rắc rối trong một kịch bản."); return; }
      var h = { type: b.dataset.add }; Object.assign(h, S.HAZARD_DEFAULTS[h.type]); STU.hazards.push(h); stuRender(); stuRun(); }); });
    bodyEl.querySelectorAll("[data-drive]").forEach(function (b) { b.addEventListener("click", function () {
      var want = b.dataset.drive === "1"; if (want === STU.drive) return;      // same button again: keep the recording
      STU.drive = want; STU.keys = {};
      if (STU.drive) { STU.pedal = new Float32Array(NF); st.t = 0; st.playing = true; syncPlay(); }
      stuSyncDrive(); stuRun(); }); });
    bodyEl.querySelectorAll("[data-pedal]").forEach(function (b) {
      var a = +b.dataset.pedal;
      b.addEventListener("pointerdown", function (e) { e.preventDefault(); stuPedal(a); });
      ["pointerup", "pointerleave", "pointercancel"].forEach(function (k) { b.addEventListener(k, function () { if (STU.pedal[Math.min(NF - 1, Math.floor(st.t / DT))] === a) stuPedal(0); }); });
    });
    $("stuRec").addEventListener("click", function () { STU.pedal = new Float32Array(NF); st.t = 0; st.playing = true; syncPlay(); stuRun(); });
    $("stuSave").addEventListener("click", function () {
      try { var blob = new Blob([JSON.stringify(stuDoc(), null, 1)], { type: "application/json" }), a = document.createElement("a");
        a.href = URL.createObjectURL(blob); a.download = "hazard.json"; document.body.appendChild(a); a.click(); setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 500);
      } catch (e) { toast("Trình duyệt chặn tải file: dùng Sao chép hoặc chép từ ô bên dưới."); } });
    $("stuCopy").addEventListener("click", function () {
      var ta = $("stuJson"); ta.value = JSON.stringify(stuDoc(), null, 1);
      try { navigator.clipboard.writeText(ta.value).then(function () { toast("Đã sao chép. Dán vào hazard.json."); }, function () { ta.select(); toast("Chọn sẵn rồi: nhấn Ctrl/⌘+C."); }); }
      catch (e) { ta.select(); toast("Chọn sẵn rồi: nhấn Ctrl/⌘+C."); } });
    $("stuOpen").addEventListener("change", function () {
      var f = this.files[0]; if (!f) return; var rd = new FileReader();
      rd.onload = function () { try { stuLoad(JSON.parse(rd.result)); stuAll(); toast("Đã mở " + f.name); } catch (err) { toast("Không đọc được " + f.name + ": " + err.message); } };
      rd.readAsText(f);
    });
    stuAll();
  }
  function stuSyncDrive() {
    bodyEl.querySelectorAll("[data-drive]").forEach(function (x) { x.setAttribute("aria-pressed", (x.dataset.drive === "1") === STU.drive); });
    var dv = $("stuDrive"); if (dv) dv.hidden = !STU.drive;
  }
  function stuAll() { stuRender(); stuGrid("stuWorld", STU_WORLD, STU.params, S.DEFAULTS); stuGrid("stuPol", STU_POL, STU.policy, S.POLICY); stuSyncDrive(); stuRun(); }
  function stuEgoEvents(Rr) { return Rr ? Rr.events.filter(function (e) { return e.a === "ego" || e.b === "ego"; }) : []; }
  function stuReadout(t, force) {
    var el = $("stuRead"); if (!el || !STU.R) return;
    function row(k, v) { return "<span>" + k + "</span><b>" + v + "</b>"; }
    var me = stuEgoEvents(STU.R)[0], gh = stuEgoEvents(STU.ghost)[0], others = STU.R.events.filter(function (e) { return e.a !== "ego" && e.b !== "ego"; });
    var ep = pose(STU.R, EGO_I, t), hz = STU.R.actors.filter(function (a) { return a.hz != null; }).length;
    el.innerHTML = row(STU.drive ? "bạn lái" : "planner", me ? '<span class="c-red">' + evWord(me) + ' ' + who(me.a === "ego" ? me.b : me.a) + " · " + fmt(me.t) + " s</span>" : '<span class="c-green">không va chạm</span>') +
      row(STU.drive ? "planner, cùng thế giới" : "planner gốc", gh ? evWord(gh) + " " + who(gh.a === "ego" ? gh.b : gh.a) + " · " + fmt(gh.t) + " s" : "không va chạm") +
      row("va chạm giữa xe khác", others.length ? others.map(function (e) { return who(e.a) + "–" + who(e.b); }).slice(0, 3).join(", ") : "không") +
      row("tốc độ xe ta lúc này", fmt(ep.v * 3.6, 0) + " km/h") +
      row("rắc rối đang có", hz + STU.hazards.filter(function (h) { return h.type === "brake"; }).length + " / 4");
  }
  function stuLabels(t) {
    if (!STU.R) return;
    STU.R.actors.forEach(function (a, i) { if (a.hz == null) return; var p = pose(R, i, t); label(p.x, heightOf(a) + 1.1, p.z, "H" + a.hz + " " + a.name, "pred"); });
    STU.hazards.forEach(function (h, k) { if (h.type !== "brake") return; var i = R.actors.findIndex(function (a) { return a.id === h.who; }); if (i < 0) return; var p = pose(R, i, t);
      label(p.x, 2.9, p.z, "H" + k + " phanh gấp" + (p.acc < -2.5 ? " ●" : ""), "hit"); });
    if (STU.drive) { var e = pose(R, EGO_I, t); label(e.x, 2.9, e.z, "bạn lái · " + fmt(e.v * 3.6, 0) + " km/h", "plan"); }
  }
  // meshes for actors beyond the fixed sixteen (hazards); built on demand, hidden when not in the current run
  var extraObj = {};
  function bikeMesh() {
    var g = new T.Group(), m = mat("car-1"), d = mat("sc-wheel");
    var fr = new T.Mesh(BOX, m); fr.scale.set(1.2, 0.08, 0.08); fr.position.y = 0.62; g.add(fr);
    [0.6, -0.6].forEach(function (x) { var w = new T.Mesh(new T.TorusGeometry(0.33, 0.04, 6, 16), d); w.position.set(x, 0.34, 0); g.add(w); });
    var body = new T.Mesh(new T.CylinderGeometry(0.17, 0.2, 0.75, 8), mat("ped")); body.position.set(-0.05, 1.15, 0); g.add(body);
    var head = new T.Mesh(new T.SphereGeometry(0.15, 10, 8), mat("sc-cab")); head.position.set(0.05, 1.68, 0); g.add(head);
    g.traverse(function (o) { if (o.isMesh) o.castShadow = true; }); g.userData.wheels = []; return g;
  }
  function drawExtras(t) {
    var seen = {};
    for (var i = NA; i < R.actors.length; i++) {
      var a = R.actors[i], key = a.id + ":" + a.kind + ":" + (a.dark ? 1 : 0), o = extraObj[key];
      if (!o) { o = extraObj[key] = a.kind === "ped" ? pedMesh() : a.kind === "bike" ? bikeMesh() : carMesh(a.dark ? "car-dark" : colorKey[a.color], a.kind); scene.add(o); }
      var p = pose(R, i, t); seen[key] = 1; o.visible = true;
      o.position.set(p.x, a.kind === "ped" ? 0.16 : 0, p.z); o.rotation.y = Math.atan2(-p.hz, p.hx);
      if (a.kind === "ped") { var ph = t * 7 + i, mv = p.v > 0.1 ? 1 : 0; o.userData.legs[0].rotation.z = 0.5 * Math.sin(ph) * mv; o.userData.legs[1].rotation.z = -0.5 * Math.sin(ph) * mv; }
    }
    Object.keys(extraObj).forEach(function (k) { if (!seen[k]) extraObj[k].visible = false; });
  }

  // ---------------------------------------------------------------- transport bar
  var playBtn = $("play"), timeEl = $("time"), tread = $("tread"), speedBtn = $("speed");
  function syncPlay() { playBtn.textContent = st.playing ? "❚❚" : "▶"; playBtn.setAttribute("aria-label", st.playing ? "Dừng" : "Chạy"); }
  playBtn.addEventListener("click", function () { st.playing = !st.playing; if (st.playing && st.t >= T_END - 0.01) st.t = 0; syncPlay(); });
  $("restart").addEventListener("click", function () { st.t = 0; TRK = {}; st.playing = true; syncPlay(); lidarClear(); });
  speedBtn.addEventListener("click", function () { var s = [0.25, 0.5, 1, 2]; st.speed = s[(s.indexOf(st.speed) + 1) % s.length]; speedBtn.textContent = (st.speed < 1 ? fmt(st.speed, 2).replace(/0$/, "") : st.speed) + "×"; });
  timeEl.addEventListener("input", function () { st.t = +timeEl.value; });
  $("camBtn").addEventListener("click", function () { setFollow(!rig.follow); });
  function drawTicks() {
    var tk = $("ticks"), h = "";
    R.events.forEach(function (e) { h += '<i style="left:' + (100 * e.t / T_END) + '%;background:var(--' + (e.kind === "block" ? "gold" : "red") + ')" title="' + evWord(e) + ' ' + fmt(e.t) + ' s"></i>'; });
    var p = R.params; h += '<i style="left:' + (100 * p.cutT0 / T_END) + '%;background:var(--purple)" title="bắt đầu cắt làn"></i>';
    h += '<i style="left:' + (100 * p.pedT0 / T_END) + '%;background:var(--gold)" title="người đi bộ bước xuống"></i>';
    tk.innerHTML = h;
  }
  function stuKey(e, down) {
    if (!STU.on || !STU.drive) return false;
    var k = e.key === "ArrowUp" || e.key === "w" || e.key === "W" ? "up" : e.key === "ArrowDown" || e.key === "s" || e.key === "S" ? "down" : null;
    if (!k) return false;
    if (!!STU.keys[k] !== down) { STU.keys[k] = down; stuPedal(STU.keys.down ? -6 : STU.keys.up ? 1.5 : 0); }
    e.preventDefault(); return true;
  }
  document.addEventListener("keyup", function (e) { stuKey(e, false); });
  window.addEventListener("blur", function () { if (STU.on && STU.drive && (STU.keys.up || STU.keys.down)) { STU.keys = {}; stuPedal(0); } });   // a key released in another window
  document.addEventListener("keydown", function (e) {
    if (e.target && (e.target.tagName === "INPUT" && e.target.type !== "checkbox" || e.target.tagName === "SELECT" || e.target.tagName === "TEXTAREA")) return;   // a slider's own arrows are not pedals
    if (stuKey(e, true)) return;
    if (e.key === "ArrowRight") { go(st.ch + 1); e.preventDefault(); }
    else if (e.key === "ArrowLeft") { go(st.ch - 1); e.preventDefault(); }
    else if (e.key === " ") { st.playing = !st.playing; syncPlay(); e.preventDefault(); }
  });
  $("themeBtn").addEventListener("click", function () {
    var cur = document.documentElement.getAttribute("data-theme") || (window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
    document.documentElement.setAttribute("data-theme", cur === "dark" ? "light" : "dark");
    try { localStorage.setItem("vlms-theme", cur === "dark" ? "light" : "dark"); } catch (e) {}
  });
  try { var savedTheme = localStorage.getItem("vlms-theme"); if (savedTheme && !document.documentElement.getAttribute("data-theme")) document.documentElement.setAttribute("data-theme", savedTheme); } catch (e) {}
  new MutationObserver(applyPal).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
  if (window.matchMedia) window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyPal);

  // ---------------------------------------------------------------- toast for collisions during playback
  var toastT = 0;
  var WHO = { ego: "xe ta", C2: "xe cắt làn", TG: "xe bám đuôi", BUS: "xe buýt", P1: "người đi bộ", L1: "xe phía trước", X1: "xe rẽ phải", E5: "xe làn bên" };
  function who(id) { if (WHO[id]) return WHO[id]; var a = R.actors.filter(function (q) { return q.id === id; })[0]; return a ? a.name : "xe " + id; }
  function toast(msg) { var el = $("toast"); el.textContent = msg; el.hidden = false; toastT = performance.now() + 3200; }

  // ================================================================== frame loop
  function resize() { var w = stage.clientWidth, h = stage.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix(); }
  window.addEventListener("resize", resize);
  var last = performance.now(), wheelRot = new Float32Array(NA), lastT = st.t;
  function frame(now) {
    var dtReal = Math.min(0.05, (now - last) / 1000); last = now;
    var prevT = st.t;
    if (st.playing) { st.t += dtReal * st.speed; if (st.t >= T_END) { st.t = 0; TRK = {}; lastPF = -1; } }
    var t = st.t;
    if (document.activeElement !== timeEl) timeEl.value = t;
    tread.textContent = fmt(t) + " s";
    // actors
    for (var i = 0; i < NA; i++) {
      var p = pose(R, i, t), o = actorObj[i], a = ACT[i];
      o.position.set(p.x, a.kind === "ped" ? 0.16 : 0, p.z); o.rotation.y = Math.atan2(-p.hz, p.hx);
      if (a.kind === "ped") { var ph = t * 7 + i; var mv = p.v > 0.1 ? 1 : 0; o.userData.legs[0].rotation.z = 0.5 * Math.sin(ph) * mv; o.userData.legs[1].rotation.z = -0.5 * Math.sin(ph) * mv; }
      else { wheelRot[i] -= p.v * (t - lastT) / 0.34; o.userData.wheels.forEach(function (w) { w.rotation.z = wheelRot[i]; }); }
      o.visible = !(st.splat && st.splatOnly && false);
    }
    lastT = t;
    drawExtras(t);
    var ego = pose(R, EGO_I, t);
    // ghost of the logged ego
    var ghostSrc = st.ghostLog && CH[st.ch].key === "replay" ? LOG : st.labGhost && CH[st.ch].key === "lab" ? LAB.ghost : st.stuGhost && CH[st.ch].key === "studio" ? STU.ghost : null;
    ghost.visible = !!ghostSrc;
    if (ghost.visible) { var gp = pose(ghostSrc, EGO_I, t); ghost.position.set(gp.x, 0.02, gp.z); ghost.rotation.y = Math.atan2(-gp.hz, gp.hx); }
    // sensors
    lidPts.visible = sweep.visible = st.lidar; dropPts.visible = st.lidar && st.lidarReal && st.showDrop;
    if (st.lidar) lidarStep(st.playing ? dtReal : dtReal * 0.6, t, R);
    sensorRig.visible = st.frustums; radarRig.visible = st.radar;
    // perception / prediction / planning
    var needPer = st.det || st.pred || st.plan;
    if (needPer) perceive(t, R); else { DET = []; }
    drawPerception();
    drawPrediction(t, R);
    if (st.plan) plan(t, R); drawPlan();
    stGroup.visible = stGrid.visible = st.st && st.plan;
    if (stGroup.visible) for (var sk = 1; sk <= 6; sk++) label(ego.x + 48, sk * stK + 0.3, ego.z, "t +" + sk + " s", "gt");
    if (stGroup.visible) buildST(t, R);
    // collisions
    hitRings.forEach(function (m) { var ev = m.userData.ev, on = t >= ev.t && t < ev.t + 6; m.visible = on; if (on) { var s = 1 + 0.25 * Math.sin((t - ev.t) * 8); m.scale.set(s, s, s); label(ev.x, 3.2, ev.z, ev.fake ? "va chạm giả (phát lại)" : evWord(ev), "hit"); } });
    R.events.forEach(function (ev) { if (prevT < ev.t && t >= ev.t && st.playing) toast(ev.fake ? "Va chạm giả: xe bám đuôi phát lại theo log, không biết xe ta đã phanh sớm hơn." : ev.kind === "block" ? "Chặn lối: xe ta dừng trên vạch qua đường, người đi bộ phải đứng lại lúc " + fmt(ev.t) + " s" : "Va chạm: " + who(ev.a) + " và " + who(ev.b) + " lúc " + fmt(ev.t) + " s"); });
    if (toastT && performance.now() > toastT) { $("toast").hidden = true; toastT = 0; }
    // variants
    varLines.visible = st.variants && !!VAR; updateGhosts(t);
    // reconstruction view
    var rec = CH[st.ch].key === "real2sim" && REC;
    splat.visible = !!rec; holes.visible = !!rec && st.holes;
    var hideReal = !!rec && st.splatOnly, stMode = st.st && st.plan;
    world.visible = !hideReal; bldMesh.visible = crown.visible = trunk.visible = !stMode; parkedObj.forEach(function (o) { o.visible = !hideReal; });
    if (rec && (st.novel > 0 || st.novelH > 1.7)) {
      var e2 = pose(LOG, EGO_I, t);
      rig.fp = { x: e2.x - 6 * e2.hx, y: st.novelH + 0.4, z: e2.z - st.novel - 0.0, lx: e2.x + 30 * e2.hx, ly: st.novelH * 0.35, lz: e2.z - st.novel * 0.5 };
    } else rig.fp = null;
    // weather
    stepRain(dtReal, rig.tx, rig.tz);
    // camera and light
    stepRig(dtReal, ego);
    sun.position.set(rig.tx + 60 * Math.cos(env.tod * Math.PI) , 40 + 60 * Math.sin(env.tod * Math.PI), rig.tz + 45); sun.target.position.set(rig.tx, 0, rig.tz);
    // labels and readouts
    chapterReadouts(t, ego);
    renderer.render(scene, camera);
    flushLabels();
    requestAnimationFrame(frame);
  }
  var roTick = 0;
  function chapterReadouts(t, ego) {
    var key = CH[st.ch].key;
    if (key === "world") {
      [["ego", "xe ta"], ["C2", "cắt làn"], ["P1", "người đi bộ"], ["BUS", "xe buýt"], ["TG", "bám đuôi"]].forEach(function (q) { var i = ACT.findIndex(function (a) { return a.id === q[0]; }), p = pose(R, i, t); label(p.x, heightOf(ACT[i]) + 1.2, p.z, q[1] + (ACT[i].kind !== "ped" && ACT[i].kind !== "bus" ? " · " + fmt(p.v * 3.6, 0) + " km/h" : ""), q[0] === "ego" ? "plan" : q[0] === "C2" ? "pred" : ""); });
    }
    if (key === "perception" || key === "prediction") {
      DET.filter(function (g) { return g.vis && g.r < 50; }).sort(function (a, b) { return a.r - b.r; }).slice(0, 6).forEach(function (g) {
        if (g.k) label(g.k.x, heightOf(g.a) + 0.9, g.k.z, "#" + g.k.id + " " + (g.a.kind === "ped" ? "người" : g.a.kind === "bus" ? "buýt" : "xe") + " " + fmt(g.m.conf, 2), "det"); });
    }
    if (key === "studio") stuLabels(t);
    if (key === "planning" && st.plan && PLAN.best) { var qb = PLAN.best.pts[14]; label(qb[0], 1.6, qb[1], planText(PLAN.best), "plan"); }
    if ((roTick++ % 6) !== 0) return;
    if (key === "sensors") {
      var n = 0, dr = 0; for (var i = 1; i < LID.pos.length; i += 3) { if (LID.pos[i] > -900) n++; if (LID.drop[i] > -900) dr++; }
      var a = $("rPts"), b = $("rDrop"); if (a) a.textContent = n.toLocaleString("vi-VN"); if (b) b.textContent = st.lidarReal ? Math.round(100 * dr / Math.max(1, n + dr)) + "%" : "0%";
    }
    if (key === "perception") {
      var inR = DET.filter(function (g) { return g.r < 75; }), det = inR.filter(function (g) { return g.vis; });
      var x = $("rIn"), y = $("rDet"), z = $("rMiss"); if (x) { x.textContent = inR.length; y.textContent = det.length; z.textContent = inR.length - det.length; }
    }
    if (key === "prediction") {
      function top(id) { var pr = PRED.find(function (q) { return q.g.a.id === id; }); if (!pr) return "chưa thấy"; var m = pr.modes.slice().sort(function (a, b) { return b.p - a.p; })[0]; return m.name + " " + Math.round(m.p * 100) + "%"; }
      var c = $("rC2"), d = $("rP1"); if (c) { c.textContent = top("C2"); d.textContent = top("P1"); }
    }
    if (key === "planning") {
      var pl = $("rPlan"), bd = $("rBad"); if (pl && PLAN.best) { pl.textContent = planText(PLAN.best); bd.textContent = PLAN.cands.filter(function (c) { return c.bad; }).length + " / " + PLAN.cands.length; }
    }
    if (key === "replay") {
      var ev = $("rEv"), tg = $("rTg"); if (ev) { ev.textContent = R.events.length ? R.events.map(function (e) { return (e.fake ? "giả " : "") + fmt(e.t) + " s"; }).join(", ") : "không có"; var ps = S.pairStats(R, "TG", "ego"); tg.textContent = ps.gapMin < 0 ? "chạm" : fmt(ps.gapMin) + " m"; }
    }
    if (key === "real2sim") { var hs = holeStats(), hb = $("rHole"); if (hb) hb.textContent = !hs ? "–" : hs.holes.toLocaleString("vi-VN"); }
    if (key === "studio") stuReadout(t);
  }

  // ================================================================== go
  applyPal(); applyEnv(); resize(); rebuildHits();
  var start = 0, hashKey = (location.hash || "").slice(1); CH.forEach(function (c, i) { if (c.key === hashKey) start = i; });
  go(start); syncPlay();
  var e0 = pose(R, EGO_I, st.t); rig.tx = e0.x + 18; rig.tz = e0.z;
  if (!REDUCED && start === 0) { rig.dist = 260; rig.pitch = 1.35; rig.yaw = 2.2; camTo({ dist: CH[0].cam.dist, yaw: CH[0].cam.yaw, pitch: CH[0].cam.pitch }, 3600); }
  requestAnimationFrame(frame);
  window.__vlms = { st: st, go: go, rig: rig, env: env, applyEnv: applyEnv, get R() { return R; }, get PLAN() { return PLAN; }, get PRED() { return PRED; } };
})();
