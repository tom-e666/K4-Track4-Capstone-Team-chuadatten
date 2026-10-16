/* SKYFALL scenario presets; x/y are metres, with z=0 for 2D physics. */
(function (root) {
  "use strict";
  const towers = [
    { id: "A", name: "LAUNCH", x0: -32, x1: 18, height: 105 },
    { id: "B", name: "CHECKPOINT", x0: 78, x1: 112, height: 112 },
    { id: "C", name: "LANDING", x0: 170, x1: 214, height: 120 }
  ];
  const floor = 44, ceiling = 64, goalX = 188, safeHalfWidth = 9;
  function make(kind = "jump", options = {}) {
    const base = {
      id: kind, seed: 42, launchSpeed: 26, launchVY: 5,
      initialPitch: 0.1, angularVelocity: 0.1, wind: 0,
      disturbance: kind === "recovery",
      turbulence: kind === "recovery" ? 1.7 : 0,
      floor, ceiling, goalX, safeHalfWidth, maxSeconds: 16
    };
    return { ...base, ...options, towers: towers.map(t => ({ ...t })) };
  }
  root.SkyfallScenarios = { towers, floor, ceiling, goalX, make };
})(typeof window !== "undefined" ? window : globalThis);
