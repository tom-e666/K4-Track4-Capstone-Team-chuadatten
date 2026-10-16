/* Run with: node viewer/skyfall-tests.js */
"use strict";
const assert = require("node:assert/strict");
require("./skyfall-scenarios.js");
require("./skyfall-physics.js");
const S = globalThis.SkyfallScenarios, P = globalThis.SkyfallPhysics;
assert.ok(S && P, "Modules must load in Node");
for (const kind of ["jump", "recovery"]) {
  const sc = S.make(kind);
  const a = P.simulate(sc, "baseline");
  const b = P.simulate(sc, "controlled");
  assert.equal(a.status, "crashed", kind + " baseline should fail");
  assert.equal(b.status, "landed", kind + " controlled should land");
  assert.ok(b.enteredB && b.enteredC, "Must reach tower B and C");
  assert.ok(Math.abs(b.x - sc.goalX) < 6, "Must land near target");
  const v = P.simulate(sc, "controlled");
  assert.equal(v.x, b.x, "Simulation must be deterministic");
  assert.equal(v.y, b.y, "Simulation must be deterministic");
  const batch = P.benchmark(sc, 20);
  assert.equal(batch.rows.length, 20);
  assert.equal(batch.baselineSuccess, 0);
  assert.equal(batch.controlledSuccess, 20);
  console.log(kind + ": baseline=" + a.status +
    ", controlled=" + b.status +
    ", success=" + batch.controlledSuccess + "/20");
}
console.log("SKYFALL smoke tests passed.");
