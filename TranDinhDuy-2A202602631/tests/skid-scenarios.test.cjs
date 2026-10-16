const test=require('node:test'),assert=require('node:assert/strict');
let C,SCENARIOS,AttitudeThrusters;
test.before(async()=>{C=await import('../viewer/river/core.js');({SCENARIOS}=await import('../viewer/river/src/core/scenarios.js'));({AttitudeThrusters}=await import('../viewer/river/src/vehicle/attitudeThrusters.js'));});
function run(config){const engine=new C.Engine(config),frames=[];while(!engine.state.terminal){engine.step(C.mockCommand(engine.observation()));frames.push(engine.observation());}return {engine,frames};}

test('driver reacts late, tyres slip, car is ejected and rotates before rescue starts',()=>{
  const {engine:e,frames}=run({});
  const names=['driver_inattentive','late_turn','skid_started','ejected_spinning','rescue_activation','touchdown'];
  const times=names.map(name=>{const event=e.events.find(v=>v.type===name);assert.ok(event,name);return event.t;});
  assert.ok(times.every((t,i)=>i===0||t>times[i-1]));
  const spinning=frames.filter(f=>f.state.accidentOccurred&&!f.state.flightActive&&!f.state.terminal);
  assert.ok(spinning.length>10);assert.ok(spinning.some(f=>Math.abs(f.state.yaw)>1));
  assert.ok(spinning.some(f=>Math.abs(f.state.roll)>.25));
  assert.equal(e.state.outcome,'landed');
});

test('body jets actually reduce angular momentum and precede the main lift ramp',()=>{
  const {engine:e,frames}=run({});const active=frames.find(f=>f.state.flightActive),later=frames.find(f=>f.state.t>active.state.t+3.5);
  const rates=f=>Math.hypot(f.state.yawRate,f.state.pitchRate,f.state.rollRate);
  assert.ok(rates(later)<rates(active)*.25);
  assert.ok(frames.some(f=>f.attitudeThrusters.yawPositive>.5));
  const initial=frames.find(f=>f.state.rescueSince!==null&&f.state.t-f.state.rescueSince>.04);
  const lift=frames.find(f=>f.state.t>initial.state.t+.3);
  assert.ok(initial.attitudeThrusters.totalPower>0);assert.ok(lift.thrusters.totalLift>initial.thrusters.totalLift);
  assert.ok(Math.abs(e.state.pitch)<.01&&Math.abs(e.state.roll)<.01);
});

test('all presets land, with distinct touchdown positions, headings and timing',()=>{
  const summaries=[];
  for(const [name,c]of Object.entries(SCENARIOS)){
    const {engine:e,frames}=run(c);assert.equal(e.state.outcome,'landed',name);
    const touch=e.events.find(event=>event.type==='touchdown');
    assert.ok(Math.abs(touch.z-c.landingOffset)<1);
    summaries.push([touch.x,touch.z,Math.atan2(Math.sin(touch.yaw),Math.cos(touch.yaw)),touch.t]);
    const hold=e.events.find(event=>event.type==='hover_before_landing');
    if(name==='spin'){assert.ok(hold);assert.ok(touch.t-hold.t>1.5);assert.ok(frames.some(f=>f.state.mode==='HOLD_BEFORE_LANDING'));}
    else assert.equal(hold,undefined);
    if(name==='wet')assert.ok(Math.min(...frames.map(f=>f.state.z))<c.landingOffset-3,'arc really deviates sideways before approaching the pad');
  }
  assert.equal(new Set(summaries.map(s=>s.map(v=>v.toFixed(1)).join(','))).size,3);
});

test('grip changes slip and excessive rescue delay is allowed to fail honestly',()=>{
  const dry=run({grip:.8}),wet=run({grip:.2});
  assert.notEqual(dry.engine.events.find(e=>e.type==='ejected_spinning').z,wet.engine.events.find(e=>e.type==='ejected_spinning').z);
  const late=run({rescueDelay:1.4,spinIntensity:4});assert.notEqual(late.engine.state.outcome,'landed');
});

test('reaction thrusters produce signed torques and retain zero force when disabled',()=>{
  const rcs=new AttitudeThrusters();let f=rcs.step({yawPositive:1,pitchNegative:.5,rollPositive:.5},true,.02);
  assert.ok(f.yawAcceleration>0&&f.pitchAcceleration<0&&f.rollAcceleration>0);
  for(let i=0;i<100;i++)f=rcs.step({},false,.02);assert.ok(f.totalPower<1e-9);
  assert.throws(()=>C.normalizeCommand({attitude:{yawPositive:NaN}}));assert.throws(()=>C.normalizeCommand({acceleration:{x:NaN,z:0}}));
});
