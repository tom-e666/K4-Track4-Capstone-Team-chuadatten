#!/usr/bin/env node
"use strict";
const http=require('node:http'),fs=require('node:fs'),path=require('node:path');
const root=path.resolve(__dirname,'../viewer');
if(process.argv.includes('--help')){console.log('node scripts/serve.cjs [port]  (default 8766, localhost only)');process.exit(0);}
const port=Number(process.argv[2]||8766);
if(!Number.isInteger(port)||port<1024||port>65535)throw new Error('Port must be an integer from 1024 to 65535');
const allowed=new Set(['index.html','app.js','sim-core.js','lab-sample.js','jet-sim.js','jet-lab.js','jet-lab-sample.js','vendor/three.min.js']);
const server=http.createServer((req,res)=>{
  let route;try{route=new URL(req.url,'http://127.0.0.1').pathname;}catch(e){res.writeHead(400);res.end();return;}
  const name=route==='/'?'index.html':route.slice(1);
  if(!allowed.has(name)){res.writeHead(404);res.end('Not found');return;}
  res.setHeader('Content-Type',name.endsWith('.html')?'text/html; charset=utf-8':'text/javascript; charset=utf-8');
  res.setHeader('Cache-Control','no-store');
  const stream=fs.createReadStream(path.join(root,name));stream.on('error',()=>{if(!res.headersSent)res.writeHead(404);res.end('File unavailable');});stream.pipe(res);
});
server.on('error',err=>{console.error(err.message);process.exitCode=1;});
server.listen(port,'127.0.0.1',()=>console.log('JET3 · TowerFlight Lab: http://127.0.0.1:'+port+'/#jetlab'));
