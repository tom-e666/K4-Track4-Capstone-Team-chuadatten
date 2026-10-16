#!/usr/bin/env node
/* Random exploration + cross-entropy failure search; no third-party packages. */
"use strict";
const fs = require('node:fs'), path = require('node:path');
const J = require('../viewer/jet-sim.js');
const ROOT = path.resolve(__dirname, '..');
function usage() {
  console.log(`Generate measured jet-car scenarios for the 3D viewer.

node scripts/generate-jet-scenarios.cjs [options]
  --seed N             Reproducible seed (default 20261016)
  --samples N          Initial uniform samples, including baseline (default 600)
  --rounds N           CEM rounds (default 6)
  --population N       Candidates per CEM round (default 500)
  --elite X            Elite fraction (default 0.1)
  --policy FILE        JSON overrides for the ground-truth PD controller
  --out DIR            Output directory (default evidence/jet-stunt)
  --sample-js FILE     Also write an offline browser sample, e.g. viewer/jet-lab-sample.js
  --help               Show this help

Outputs: viewer.json, summary.json, cases.json. Load viewer.json in Lab phản lực.
The shared model integrates 3D translation and yaw with four vector thrusters.
Roll/pitch are visual only. Collision checks are a vehicle-envelope approximation.
The controller reads true state; camera, radar and LiDAR are not required or simulated.
Safe/near/crash are measured on this model. Infeasible and incomplete are separate.
CEM searches failures: its sample counts are NOT estimates of real-world risk.`);
}
function main(argv) {
  const o={seed:20261016,samples:600,rounds:6,population:500,elite:0.1,out:path.join(ROOT,'evidence','jet-stunt'),sampleJs:null,policy:null};
  const flags={'--seed':'seed','--samples':'samples','--rounds':'rounds','--population':'population','--elite':'elite','--out':'out','--sample-js':'sampleJs','--policy':'policy'};
  for(let i=0;i<argv.length;i++) {
    if(argv[i]==='--help'){usage();return;}
    const key=flags[argv[i]];if(!key || i+1>=argv.length)throw new Error('Unknown or incomplete argument: '+argv[i]);
    const v=argv[++i];o[key]=['out','sampleJs','policy'].includes(key)?path.resolve(v):Number(v);
  }
  for(const key of ['seed','samples','rounds','population'])if(!Number.isSafeInteger(o[key]) || o[key]<(key==='rounds'?0:1))throw new Error('Invalid '+key);
  if(o.samples>100000 || o.rounds>100 || o.population>10000 || !(o.elite>0&&o.elite<=0.5))throw new Error('Search budget or elite fraction out of bounds');
  const policy=J.policy(o.policy?JSON.parse(fs.readFileSync(o.policy,'utf8')):{}),names=Object.keys(J.SPACE);
  let seed=o.seed>>>0;
  function rand(){seed=(seed+0x6D2B79F5)>>>0;let a=seed;a=Math.imul(a^(a>>>15),a|1);a^=a+Math.imul(a^(a>>>7),a|61);return((a^(a>>>14))>>>0)/4294967296;}
  function normal(){return Math.sqrt(-2*Math.log(Math.max(1e-12,rand())))*Math.cos(2*Math.PI*rand());}
  const proposal=names.map(n=>{const b=J.SPACE[n];return{mean:(b[0]+b[1])/2,std:(b[1]-b[0])/Math.sqrt(12)};});
  function sample(adaptive){const p={};names.forEach((n,i)=>{const b=J.SPACE[n],q=proposal[i];
    const v=adaptive?q.mean+q.std*normal():b[0]+rand()*(b[1]-b[0]);p[n]=Number(Math.max(b[0],Math.min(b[1],v)).toFixed(8));});return p;}
  const cloud={names:names,x:'windZ',y:'podLoss',rows:[],out:[],it:[],mode:[],score:[],hash:[]},evaluations=[],history=[];
  function evaluate(p,iteration,source){
    const r=J.rollout(p,policy,{record:false}),row=names.map(n=>r.params[n]),id=evaluations.length;
    cloud.rows.push(row);cloud.out.push(r.metrics.outcome);cloud.it.push(iteration);cloud.mode.push(r.metrics.reason);cloud.score.push(r.metrics.score);cloud.hash.push(r.hash);
    const e={index:id,params:r.params,metrics:r.metrics,events:r.events,hash:r.hash,duration:r.duration,source:source};evaluations.push(e);return e;
  }
  function counts(rows){return rows.reduce((a,e)=>{const k=e.metrics.outcome;a[k]=(a[k]||0)+1;return a;},{});}
  const baseline=evaluate({},0,'baseline');
  for(let i=1;i<o.samples;i++)evaluate(sample(false),0,'uniform');
  console.log('Uniform exploration: '+o.samples+' '+JSON.stringify(counts(evaluations)));
  for(let round=1;round<=o.rounds;round++) {
    const batch=[];for(let i=0;i<o.population;i++)batch.push(evaluate(sample(rand()>=0.2),round,'cem'));
    const feasible=batch.filter(e=>e.metrics.feasible).sort((a,b)=>a.metrics.score-b.metrics.score);
    const elite=feasible.slice(0,Math.max(2,Math.ceil(feasible.length*o.elite)));
    if(elite.length>=2)names.forEach((n,i)=>{
      const mean=elite.reduce((s,e)=>s+e.params[n],0)/elite.length;
      const variance=elite.reduce((s,e)=>s+(e.params[n]-mean)**2,0)/elite.length,b=J.SPACE[n],q=proposal[i];
      q.mean=0.35*q.mean+0.65*mean;q.std=Math.max((b[1]-b[0])*0.06,Math.sqrt(0.35*q.std*q.std+0.65*variance));
    });
    history.push({round:round,counts:counts(batch),bestScore:feasible.length?feasible[0].metrics.score:null,proposal:JSON.parse(JSON.stringify(proposal))});
    console.log('CEM '+round+'/'+o.rounds+': '+JSON.stringify(counts(batch))+' best='+history[history.length-1].bestScore);
  }
  const selected=[baseline],seen=new Set([baseline.index]);
  function add(e){if(e&&!seen.has(e.index)){selected.push(e);seen.add(e.index);}}
  for(const outcome of ['crash','near','safe','incomplete']) {
    const list=evaluations.filter(e=>e.metrics.outcome===outcome).sort((a,b)=>a.metrics.score-b.metrics.score);
    const reasons=new Set();for(const e of list){if(reasons.has(e.metrics.reason))continue;add(e);reasons.add(e.metrics.reason);if(selected.length>=10)break;}
  }
  const runs=selected.slice(0,10).map((e,i)=>({id:'JET-'+String(i+1).padStart(2,'0'),cloudIndex:e.index,params:e.params,outcome:e.metrics.outcome,label:e.metrics.reason,metrics:e.metrics,events:e.events,duration:e.duration,trajectoryHash:e.hash}));
  const summary={seed:o.seed,total:evaluations.length,counts:counts(evaluations),uniformCounts:counts(evaluations.filter(e=>e.source!=='cem')),cemCounts:counts(evaluations.filter(e=>e.source==='cem')),
    baseline:baseline.metrics,modelVersion:J.VERSION,controller:'ground-truth PD with four vector thrusters',sensors:[],
    caveats:['CEM samples are biased toward failures; counts are not risk probabilities.','3D point-mass plus dynamic yaw; roll/pitch are not rigid-body dynamics.','Facade crossing and vehicle envelope are collision approximations.']};
  const data={format:'jet-stunt-viewer/1',modelVersion:J.VERSION,title:'Xe phản lực: uniform + CEM, seed '+o.seed,seed:o.seed,policy:policy,space:J.SPACE,
    search:{method:'uniform + defensive CEM',uniformSamples:o.samples,rounds:o.rounds,population:o.population,eliteFraction:o.elite,uniformMixture:0.2,history:history},summary:summary,runs:runs,cloud:cloud};
  fs.mkdirSync(o.out,{recursive:true});
  fs.writeFileSync(path.join(o.out,'viewer.json'),JSON.stringify(data));
  fs.writeFileSync(path.join(o.out,'summary.json'),JSON.stringify(summary,null,2)+'\n');
  fs.writeFileSync(path.join(o.out,'cases.json'),JSON.stringify({format:data.format,modelVersion:J.VERSION,policy:policy,runs:runs},null,2)+'\n');
  if(o.sampleJs){fs.mkdirSync(path.dirname(o.sampleJs),{recursive:true});fs.writeFileSync(o.sampleJs,'/* Generated by scripts/generate-jet-scenarios.cjs; measured shared-model runs. */\nwindow.JET_LAB_SAMPLE = '+JSON.stringify(data)+';\n');}
  console.log(JSON.stringify(summary,null,2));console.log('Viewer data: '+path.join(o.out,'viewer.json'));
}
try{main(process.argv.slice(2));}catch(e){console.error('Error: '+e.message);process.exitCode=1;}
