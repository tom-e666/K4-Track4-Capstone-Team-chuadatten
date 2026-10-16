const test = require('node:test');
const assert = require('node:assert/strict');
let C;
require('node:test').before(async()=>{ C=await import('../viewer/river/core.js'); });

function run(config, planner = C.mockCommand) {
  const e = new C.Engine(config), trace = [e.observation()];
  while (!e.state.terminal) { e.step(planner(e.observation())); trace.push(e.observation()); }
  return { e, trace };
}

test('late braking without flight falls into water before reaching the other bank', () => {
  const { e } = run({ flightEnabled: false });
  assert.equal(e.state.outcome, 'splash');
  assert.ok(e.state.x > 0 && e.state.x < e.config.riverWidth);
  assert.ok(e.events.some(event => event.type === 'left_bank'));
  assert.ok(!e.events.some(event => event.type === 'rescue_activation'));
});

test('mock rescues a skid after its configured delay and lands across the river', () => {
  for (const riverWidth of [24, 48]) for (const initialSpeed of [18, 22]) {
    const { e, trace } = run({ riverWidth, initialSpeed });
    assert.equal(e.state.outcome, 'landed', `width=${riverWidth}, speed=${initialSpeed}`);
    assert.ok(Math.abs(e.state.x - (riverWidth + 22)) <= 10);
    assert.ok(Math.abs(e.state.z) <= 5);
    assert.equal(e.state.y, 0);
    const depart = e.events.find(event => event.type === 'left_bank');
    const activate = e.events.find(event => event.type === 'rescue_activation');
    assert.ok(activate.t-depart.t>=e.config.rescueDelay,'rescue begins after the car is ejected');
    assert.ok(activate.t-depart.t<=e.config.rescueDelay+2*C.DT+1e-6,'rescue uses simulation-time delay');
    assert.ok(trace.every(o => o.state.y > e.config.waterY + 0.25));
    assert.ok(trace.some(o => o.state.y > 3), 'vehicle truly gains altitude');
  }
});

test('a planner can brake early and stop safely; flight is not scripted to occur', () => {
  const { e } = run({}, () => ({ brake: 1 }));
  assert.equal(e.state.outcome, 'stopped');
  assert.ok(e.state.x < -2.3);
  assert.equal(run({ initialSpeed: 12 }).e.state.outcome, 'stopped');
});

test('sampled navigation still makes contact and lands at 50–200 ms update intervals', () => {
  for (const ticks of [3, 5, 10]) {
    let command;
    const { e } = run({}, observation => {
      if (observation.seq % ticks === 0) command = C.mockCommand(observation);
      return command;
    });
    assert.equal(e.state.outcome, 'landed', `command held for ${ticks * C.DT}s`);
  }
});

test('physics is deterministic and an observation cannot mutate the engine', () => {
  const first = run({}), second = run({});
  assert.deepEqual(first.e.state, second.e.state);
  assert.deepEqual(first.e.events, second.e.events);
  const observation = first.e.observation(); observation.state.x = 999; observation.map.farBankX = 999;
  assert.notEqual(first.e.state.x, 999); assert.equal(first.e.config.riverWidth, 24);
});

test('controls clamp valid numbers, reject NaN, and cannot fly a road-only vehicle', () => {
  assert.deepEqual(C.normalizeCommand({ throttle: 2, brake: -4, steer: -2, lift: 3, flight: true }),
    { throttle: 1, brake: 0, steer: -1, lift: 1, flight: true });
  for (const field of ['throttle', 'brake', 'steer', 'lift']) {
    assert.throws(() => C.normalizeCommand({ [field]: NaN }));
    assert.throws(() => C.normalizeCommand({ [field]: '1' }));
  }
  const { e } = run({ flightEnabled: false }, () => ({ throttle: 1, flight: true, lift: 1 }));
  assert.ok(['splash','crashed'].includes(e.state.outcome)); assert.equal(e.state.flightActive, false);
});

test('a poor flight controller can miss the landing or crash; terminal state freezes', () => {
  const { e } = run({}, o => ({ flight: o.state.x < o.map.farBankX, lift: .9 }));
  assert.notEqual(e.state.outcome, 'landed');
  const before = { ...e.state }; e.step({ throttle: 1 }); assert.deepEqual(e.state, before);
});

test('steering affects the physical path and timeout terminates a hovering controller', () => {
  const e = new C.Engine();
  for (let i = 0; i < 50; i++) e.step({ steer: .5 });
  assert.ok(e.state.z > 0); assert.ok(e.state.yaw > 0);
  const { e: hover } = run({}, o => ({ brake: 1, flight: true, lift: 9.81 / o.vehicle.maxLift }));
  assert.equal(hover.state.outcome, 'out_of_bounds');
  assert.ok(hover.state.t >= 35);
});
