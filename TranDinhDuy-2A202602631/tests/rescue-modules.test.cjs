const test=require('node:test'),assert=require('node:assert/strict');
let C;test.before(async()=>{C=await import('../viewer/river/core.js');});
function finish(engine){while(!engine.state.terminal)engine.step(C.mockCommand(engine.observation()));return engine;}

test('all sensor modules expose measurements and multiple forward/downward probes',()=>{
  const e=new C.Engine(),o=e.observation();
  for(const field of ['distanceToGroundAhead','roadAheadExists','cliffRiskScore','altitudeAboveGround','groundDetectedBelow',
    'pitch','roll','yaw','verticalVelocity','angularVelocity','forwardSpeed','wheelContactState','grounded',
    'edgeDistance','edgeRiskScore','landingZoneFound','landingZonePosition','landingZoneNormal','landingZoneDistance'])assert.ok(field in o.sensors,field);
  assert.ok(o.sensors.forwardProbes.length>=4);assert.equal(o.sensors.downwardProbes.length,5);
  assert.equal(o.sensors.cliffRiskScore,0);e.teleportToEdge();assert.ok(e.observation().sensors.cliffRiskScore>.8);
  assert.equal(e.observation().sensors.roadAheadExists,false);
});

test('ballistic fall prediction and rescue path are independent of the renderer',()=>{
  const e=new C.Engine();e.teleportToEdge(true);const o=e.observation();
  assert.ok(o.prediction.predictedFallPath.length>10);assert.ok(o.prediction.predictedImpactPoint);
  const [first,second]=o.prediction.predictedFallPath;assert.ok(second.y<first.y);
  assert.equal(o.guidance.rescuePath.at(-1).x,o.map.landing.x);
  const gravity=o.vehicle.gravity,dt=second.t-first.t;
  assert.ok(Math.abs(second.y-(first.y+o.state.vy*dt-.5*gravity*dt*dt))<1e-9);
  assert.equal(o.prediction.predictedImpactPoint.kind,'cliff');
  assert.ok(o.prediction.predictedImpactPoint.y< -1,'impact remains on the canyon wall, not teleported to the road surface');
});

test('four equal forces give pure lift; unequal pairs give opposite pitch/roll torques',()=>{
  const thrusters=new C.ThrusterSystem(),e=new C.Engine(),c=e.config;
  const equal=Object.fromEntries(C.THRUSTER_KEYS.map(k=>[k,.6]));
  const f=thrusters.step(equal,true,c,.02);assert.equal(f.pitchAcceleration,0);assert.equal(f.rollAcceleration,0);
  assert.ok(f.totalLift>0&&f.totalLift<.6,'power is smoothed');
  thrusters.reset();const front=thrusters.step({frontLeft:1,frontRight:1,rearLeft:0,rearRight:0},true,c,.02);
  assert.ok(front.pitchAcceleration>0);assert.equal(front.rollAcceleration,0);
  thrusters.reset();const left=thrusters.step({frontLeft:1,frontRight:0,rearLeft:1,rearRight:0},true,c,.02);
  assert.ok(left.rollAcceleration>0);assert.equal(left.pitchAcceleration,0);
});

test('manual rescue recovers from falling and a disturbed attitude with auto rescue disabled',()=>{
  const e=new C.Engine({autoRescueEnabled:false});e.teleportToEdge(true);
  for(let i=0;i<10;i++)e.step(C.mockCommand(e.observation()));
  assert.equal(e.state.mode,'SPINNING');assert.equal(e.state.flightActive,false);
  e.triggerRescue();let different=false;
  for(let i=0;i<120;i++){e.step(C.mockCommand(e.observation()));const p=e.observation().thrusters;different ||= Math.abs(p.frontLeft-p.rearRight)>.01;}
  assert.ok(different);assert.ok(Math.abs(e.state.pitch)<.03);assert.ok(Math.abs(e.state.roll)<.03);
  assert.equal(finish(e).state.outcome,'landed');
});

test('state machine reports warning, activation, stabilization, movement, and landing',()=>{
  const e=finish(new C.Engine());const events=e.events.map(x=>x.type);
  for(const mode of ['WARNING','RESCUE_ACTIVE','HOVER_STABILIZE','MOVE_TO_SAFE_ZONE','LANDING','LANDED'])assert.ok(events.includes('state:'+mode),mode);
  assert.ok(events.indexOf('state:WARNING')<events.indexOf('state:RESCUE_ACTIVE'));
});

test('hover mode stops horizontal motion and holds altitude, gravity and power sliders affect physics',()=>{
  const e=new C.Engine();e.teleportToEdge(true);e.triggerRescue();e.state.hoverMode=true;
  for(let i=0;i<300;i++)e.step(C.mockCommand(e.observation()));
  assert.ok(e.state.speed<.2);assert.ok(Math.abs(e.state.y-e.config.hoverHeight)<.2);assert.equal(e.state.mode,'HOVER_STABILIZE');
  const weak=new C.Engine({gravity:15,maxLift:12});weak.teleportToEdge(true);weak.triggerRescue();
  assert.notEqual(finish(weak).state.outcome,'landed');
});

test('teammates can replace sensors and landing acceptance without touching the vehicle',()=>{
  const custom={sample(){return {cliffRiskScore:.42,roadAheadExists:false,distanceToGroundAhead:7,forwardProbes:[]};}};
  const e=new C.Engine({}, {sensors:{cliff:custom}});assert.equal(e.observation().sensors.cliffRiskScore,.42);
  const refuse=new C.Engine({}, {landingPolicy:{evaluate(){return false;}}});assert.equal(finish(refuse).state.outcome,'off_target');
});

test('explicit nozzle commands override aggregate lift and malformed nozzle values fail',()=>{
  const valid=C.normalizeCommand({flight:true,thrusters:{frontLeft:.3,frontRight:.4,rearLeft:.5,rearRight:.6}});
  assert.ok(Math.abs(valid.lift-.45)<1e-12);
  assert.throws(()=>C.normalizeCommand({thrusters:{frontLeft:1}}));
  assert.throws(()=>C.normalizeCommand({thrusters:{frontLeft:NaN,frontRight:0,rearLeft:0,rearRight:0}}));
});

test('stabilizer feedback consumes replaceable IMU measurements rather than renderer transforms',()=>{
  const e=new C.Engine();e.teleportToEdge(true);e.triggerRescue();
  e.state.rescueSince=e.state.t-.2;
  const normal=e.observation(),spoofed=structuredClone(normal);spoofed.sensors.pitch+=.25;
  const baseline=C.mockCommand(normal),changed=C.mockCommand(spoofed);
  assert.ok(changed.thrusters.frontLeft<baseline.thrusters.frontLeft);
  assert.ok(changed.thrusters.rearLeft>baseline.thrusters.rearLeft);
});
