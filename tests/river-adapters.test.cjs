const test = require('node:test');
const assert = require('node:assert/strict');
let C, SocketController;
require('node:test').before(async()=>{C=await import('../viewer/river/core.js');({SocketController}=await import('../viewer/river/controllers.js'));});

function setup() {
  let time = 0;
  class Socket {
    static OPEN = 1;
    constructor(url) { this.url = url; this.readyState = 0; this.handlers = {}; this.sent = []; }
    addEventListener(type, handler) { this.handlers[type] = handler; }
    send(data) { this.sent.push(JSON.parse(data)); }
    open() { this.readyState = 1; this.handlers.open(); }
    close() { this.readyState = 3; this.handlers.close(); }
    reply(data) { this.handlers.message({ data: JSON.stringify(data) }); }
  }
  const controller = new SocketController('ws://127.0.0.1:8765',null,{WebSocket:Socket,now:()=>time});
  const engine = new C.Engine();
  controller.connect(); controller.reset(engine.observation()); controller.socket.open();
  return { controller, engine, tick: t => { time = t; },
    reply: controls => controller.socket.reply({ schema: 'river-nav/1', type: 'command',
      runId: engine.runId, seq: controller.pending.seq, controls }) };
}

test('socket waits for matching commands, holds briefly, and pauses on stale/disconnected control', () => {
  const { controller: c, engine: e, tick, reply } = setup();
  assert.equal(c.update(e.observation()), null);
  reply({ throttle: 1, flight: true, lift: .6 });
  assert.equal(c.update(e.observation()).throttle, 1);
  tick(501); assert.equal(c.update(e.observation()), null);
  c.socket.close(); assert.equal(c.update(e.observation()), null);
});

test('socket rejects wrong run/sequence/schema and invalid controls', () => {
  const { controller: c, engine: e, reply } = setup(); c.update(e.observation());
  const valid = { schema: 'river-nav/1', type: 'command', runId: e.runId, seq: 0, controls: { throttle: 1 } };
  for (const changed of [{ runId: 'old-run' }, { seq: 999 }, { schema: 'wrong' }, { type: 'observation' }]) {
    c.socket.reply({ ...valid, ...changed }); assert.equal(c.latest, null);
  }
  reply({ lift: '1' }); assert.equal(c.latest, null);
  reply({ flight: 'true' }); assert.equal(c.latest, null);
  reply({ brake: .8 }); assert.equal(c.update(e.observation()).brake, .8);
});

test('reset drops commands from previous run and clears a pending request', () => {
  const { controller: c, engine: e, reply } = setup(); c.update(e.observation()); reply({ throttle: 1 });
  const oldRun = e.runId; e.reset(); c.reset(e.observation());
  assert.equal(c.latest, null); assert.equal(c.pending, null); c.update(e.observation());
  c.socket.reply({ schema: 'river-nav/1', type: 'command', runId: oldRun, seq: 0, controls: { throttle: 1 } });
  assert.equal(c.latest, null);
});

test('one request in flight, request retries after a second, maximum send rate is 20 Hz', () => {
  const { controller: c, engine: e, tick, reply } = setup(); c.update(e.observation());
  tick(100); c.update(e.observation()); assert.equal(c.socket.sent.length, 1);
  tick(1001); c.update(e.observation()); assert.equal(c.socket.sent.length, 2);
  reply({ brake: .3 }); tick(1020); c.update(e.observation()); assert.equal(c.socket.sent.length, 2);
  tick(1051); c.update(e.observation()); assert.equal(c.socket.sent.length, 3);
});
