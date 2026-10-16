// node tests/js_dump.js <sim-core.js> <in.json> <out.json>: run SimCore on a list of {params, policy, ego, others, hazards, egoAcc} and dump F/events
const [core, inp, outp] = process.argv.slice(2);
const S = require(core), fs = require('fs');
const jobs = JSON.parse(fs.readFileSync(inp, 'utf8')), out = [];
for (const j of jobs) {
  let log = null;
  if (j.others === 'replay') log = S.rollout({ ego: 'human', others: 'reactive', params: j.params, hazards: j.hazards });
  const R = S.rollout({ ego: j.ego, others: j.others, params: j.params, policy: j.policy, log, hazards: j.hazards, egoAcc: j.egoAcc });
  out.push({ F: Array.from(R.F), EGO: Array.from(R.EGO), events: R.events });
}
fs.writeFileSync(outp, JSON.stringify(out));
