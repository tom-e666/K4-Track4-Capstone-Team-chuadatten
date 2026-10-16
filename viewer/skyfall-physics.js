/* Deterministic 2D flight dynamics with bounded actuation and geometry-based collision.
   This is a hypothetical thruster-equipped vehicle, NOT a model of a production car. */
(function (root) {
  "use strict";
  const G = 9.81, CAR_HALF = 2.25, CAR_H = 2.05;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  function create(c) {
    return {
      t: 0, x: 10, y: 54, vx: c.launchSpeed, vy: c.launchVY,
      theta: c.initialPitch, omega: c.angularVelocity,
      status: "flying", reason: "", control: { ax: 0, thrust: 0, torque: 0 },
      enteredB: false, enteredC: false, disturbed: false, trail: []
    };
  }
  function envelope(s) {
    const half = Math.abs(Math.cos(s.theta)) * CAR_H / 2 +
      Math.abs(Math.sin(s.theta)) * CAR_HALF;
    return { bottom: s.y + CAR_H / 2 - half, top: s.y + CAR_H / 2 + half };
  }
  function step(s, c, mode, dt = 0.02) {
    if (s.status !== "flying") return s;
    dt = clamp(dt, 0.001, 0.05);
    const u = { ax: 0, thrust: 0, torque: 0 };
    if (mode === "controlled") {
      const targetY = s.x > 173 ? c.floor - 0.35 : 54;
      const targetVX = Math.min(28, Math.sqrt(Math.max(0, 14 * (c.goalX - s.x - 10))));
      u.ax = clamp((targetVX - s.vx) * 2.3, -7, 7);
      u.thrust = clamp(G + (targetY - s.y) * 1.65 - s.vy * 2.9, 0, 21);
      u.torque = clamp(-s.theta * 11 - s.omega * 6, -16, 16);
    }
    s.control = u;
    const gust = c.wind * (0.4 + 0.6 * Math.sin(s.t * 1.7));
    s.vx += u.ax * dt;
    s.vy += (u.thrust - G + gust) * dt;
    s.omega += u.torque * dt;
    s.x += s.vx * dt;
    s.y += s.vy * dt;
    s.theta += s.omega * dt;
    s.t += dt;
    // A one-time midair velocity and attitude perturbation in recovery mode.
    if (c.disturbance && !s.disturbed && s.x >= 54) {
      s.vy -= c.turbulence * 0.7;
      s.omega += c.turbulence;
      s.disturbed = true;
    }
    if (s.x >= c.towers[1].x0 - CAR_HALF &&
        s.x <= c.towers[1].x1 + CAR_HALF) s.enteredB = true;
    if (s.x >= c.towers[2].x0 - CAR_HALF) s.enteredC = true;
    const e = envelope(s);
    for (const building of c.towers) {
      if (s.x + CAR_HALF < building.x0 || s.x - CAR_HALF > building.x1) continue;
      if (e.bottom < c.floor || e.top > c.ceiling) {
        if (building.id === "C" && e.bottom <= c.floor &&
            Math.abs(s.x - c.goalX) < 6 && Math.abs(s.vx) < 3.5 &&
            Math.abs(s.vy) < 2.3 && Math.abs(s.theta) < 0.18 && s.enteredB) {
          s.status = "landed";
          s.reason = "Dừng an toàn trong tòa C";
          s.y += c.floor - e.bottom;
        } else {
          s.status = "crashed";
          s.reason = "Va chạm kết cấu tòa " + building.id;
        }
        break;
      }
    }
    if (s.y < 0 && s.status === "flying") {
      s.status = "crashed"; s.reason = "Rơi xuống mặt đất";
    }
    if (s.x > c.towers[2].x1 + 5 && s.status === "flying") {
      s.status = "crashed"; s.reason = "Bay vượt tòa C";
    }
    if (s.t >= c.maxSeconds && s.status === "flying") {
      s.status = "timeout"; s.reason = "Hết thời gian mô phỏng";
    }
    if (s.trail.length < 900 && Math.floor(s.t * 10) > s.trail.length) {
      s.trail.push([s.x, s.y]);
    }
    return s;
  }
  function simulate(c, mode = "controlled", dt = 0.02) {
    const s = create(c);
    while (s.status === "flying") step(s, c, mode, dt);
    return s;
  }
  function benchmark(c, n = 20) {
    const rows = [];
    for (let i = 0; i < n; i++) {
      const sc = { ...c,
        launchSpeed: c.launchSpeed + (i % 5 - 2) * 0.45,
        initialPitch: c.initialPitch + (i % 7 - 3) * 0.018 };
      const baseline = simulate(sc, "baseline");
      const controlled = simulate(sc, "controlled");
      rows.push({ seed: i, baseline: baseline.status, controlled: controlled.status,
        baselineTime: baseline.t, controlledTime: controlled.t });
    }
    return { rows,
      baselineSuccess: rows.filter(r => r.baseline === "landed").length,
      controlledSuccess: rows.filter(r => r.controlled === "landed").length };
  }
  root.SkyfallPhysics = { G, create, envelope, step, simulate, benchmark };
})(typeof window !== "undefined" ? window : globalThis);
