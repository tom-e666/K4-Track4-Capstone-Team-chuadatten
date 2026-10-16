"use strict";
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const J=require('../viewer/jet-sim.js');
const base=J.rollout({});
assert.equal(base.metrics.outcome,'safe');assert.equal(base.metrics.gateCount,4);assert(base.metrics.landed);
assert.deepEqual(base.events.map(e=>e.gate),[0,1,2,3]);
assert(base.events[0].t>3.5,'glass time comes from integrated crossing rather than the reference timestamp');
assert.equal(base.hash,J.rollout({}).hash,'deterministic integration');
const slow=J.rollout({}, {maxSpeed:30/3.6});
assert.equal(slow.metrics.gateCount,4);assert(slow.metrics.landed);
assert(slow.duration>base.duration+10,'speed cap retimes the flight instead of only changing playback');
assert(slow.metrics.peakSpeed<30/3.6+0.25,'nominal run tracks the configured speed limit');
assert.notEqual(slow.hash,base.hash);
const weaker=J.rollout({}, {maxPodForce:5000});assert.notEqual(weaker.hash,base.hash,'force capacity changes the integrated trajectory');
for(const s of weaker.states)for(const f of s.jets)assert(Math.hypot(...f)<=5000+1e-6);
assert.equal(J.rollout({thrustScale:0.55,massScale:1.3,podLoss:0.75}).metrics.outcome,'infeasible');
assert.throws(()=>J.rollout({windX:NaN}));assert.throws(()=>J.rollout({unknown:1}));assert.throws(()=>J.rollout({}, {kpY:-1}));
for(const p of [{},{windZ:7,podLoss:0.4,massScale:1.1},{delay:0.2,thrustScale:0.8}]){
  const r=J.rollout(p);if(!r.metrics.feasible)continue;
  for(const s of r.states){assert([s.x,s.y,s.z,s.yaw,s.vx,s.vy,s.vz].every(Number.isFinite));s.jets.forEach((f,i)=>assert(Math.hypot(...f)<=r.policy.maxPodForce*r.params.thrustScale*(i===0?1-r.params.podLoss:1)+1e-6,'per-pod force is bounded'));}
  assert.equal(J.at(r,999).x,r.states.at(-1).x);assert.equal(J.at(r,-1).x,r.states[0].x);
}
const browser={};vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../viewer/jet-sim.js'),'utf8'),{globalThis:browser});
assert.equal(browser.JetSim.rollout({windZ:5,delay:0.15}).hash,J.rollout({windZ:5,delay:0.15}).hash,'browser module and Node use the same core');
if(process.argv[2]){
  const d=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
  assert.equal(d.modelVersion,J.VERSION);const counts={};
  for(let i=0;i<d.cloud.rows.length;i++){
    const p={};d.cloud.names.forEach((n,k)=>p[n]=d.cloud.rows[i][k]);const r=J.rollout(p,d.policy,{record:false});
    assert.equal(r.hash,d.cloud.hash[i],`trajectory hash at cloud row ${i}`);assert.equal(r.metrics.outcome,d.cloud.out[i],`outcome at row ${i}`);counts[r.metrics.outcome]=(counts[r.metrics.outcome]||0)+1;
  }
  assert.deepEqual(counts,d.summary.counts);
  for(const r of d.runs){const replay=J.rollout(r.params,d.policy);assert.equal(replay.hash,r.trajectoryHash);assert.equal(replay.metrics.outcome,r.outcome);assert.deepEqual(replay.metrics,r.metrics);}
  console.log(`PASS: all ${d.cloud.rows.length} exported labels/hashes and ${d.runs.length} saved cases replay exactly.`);
}
console.log('PASS: baseline, speed regulator, force limits, physical feasibility, parameter validation, repeatability, browser/Node core parity.');
