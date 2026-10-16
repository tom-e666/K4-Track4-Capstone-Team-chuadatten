/* Shared deterministic jet-car dynamics for Node search and browser replay.
   Model: 3D point mass + dynamic yaw, four ideal vector thrusters, floor contact,
   swept centre-plane facade checks with an oriented vehicle clearance envelope.
   The PD controller reads ground-truth state. No camera/radar/LiDAR is simulated. */
(function (root, factory) {
  var api = factory(); if (typeof module === "object" && module.exports) module.exports = api; else root.JetSim = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";
  var VERSION = "jet-pointmass-yaw/2", DT = 0.025, CG = 0.85, MASS = 1400, G = 9.81;
  var TOWERS = [
    { x:49,z:-31,w:36,d:34,h:68,floor:32,corner:"TÂY BẮC",exit:{face:"east",offset:0} },
    { x:109,z:-31,w:26,d:34,h:62,floor:28,corner:"ĐÔNG BẮC",entry:{face:"west",offset:0},exit:{face:"south",offset:8} },
    { x:117,z:31,w:42,d:34,h:56,floor:24,corner:"ĐÔNG NAM",entry:{face:"north",offset:0} }
  ];
  var GATES = [{x:67,y:32,z:-31,axis:"x",yaw:0,tower:0,face:"east"}, {x:96,y:28,z:-31,axis:"x",yaw:0,tower:1,face:"west"},
    {x:117,y:28,z:-14,axis:"z",yaw:-Math.PI/2,tower:1,face:"south"}, {x:117,y:24,z:14,axis:"z",yaw:-Math.PI/2,tower:2,face:"north"}];
  var SPACE = {
    windX:[-9,9,0,"gió X (m/s)"], windZ:[-9,9,0,"gió Z (m/s)"], windGust:[0,6,0,"gió giật (m/s)"],
    thrustScale:[0.55,1.2,1,"hệ số sức đẩy"], massScale:[0.85,1.3,1,"hệ số khối lượng"],
    delay:[0,0.22,0.025,"trễ cơ cấu (s)"], podLoss:[0,0.75,0,"mất lực vòi số 1"],
    yawBias:[-0.2,0.2,0,"nhiễu quay (rad/s²)"], offsetZ:[-0.9,0.9,0,"lệch ngang ban đầu (m)"], speedScale:[0.85,1.2,1,"hệ số tốc độ đặt"]
  };
  var POLICY = { kpXY:4.5,kdXY:4.2,kpY:9,kdY:5.5,kpYaw:12,kdYaw:5.5,maxPodForce:9000,maxSpeed:80/3.6,forceSlew:90000,drag:0.35 };
  var POLICY_LIMITS = { kpXY:[0.1,30],kdXY:[0.1,20],kpY:[0.1,40],kdY:[0.1,25],kpYaw:[0.1,40],kdYaw:[0.1,25],maxPodForce:[5000,14000],maxSpeed:[5,40],forceSlew:[10000,200000],drag:[0,1] };
  var OFFSETS = [[1.55,-1.48],[1.55,1.48],[-1.55,-1.48],[-1.55,1.48]];
  function clamp(x,a,b) { return Math.max(a,Math.min(b,x)); }
  function angle(a) { return Math.atan2(Math.sin(a),Math.cos(a)); }
  function params(input) {
    var out = {}; Object.keys(input || {}).forEach(function(k) { if (!SPACE[k]) throw new Error("Unknown scenario parameter: " + k); });
    Object.keys(SPACE).forEach(function(k) { var s=SPACE[k],v=input && input[k] != null ? input[k] : s[2];
      if (typeof v!=="number" || !Number.isFinite(v) || v<s[0] || v>s[1]) throw new Error("Out of range parameter: " + k); out[k]=v; }); return out;
  }
  function policy(input) {
    var out=Object.assign({},POLICY); Object.keys(input || {}).forEach(function(k) {
      var b=POLICY_LIMITS[k],v=input[k]; if (!b || typeof v!=="number" || !Number.isFinite(v) || v<b[0] || v>b[1]) throw new Error("Invalid controller field: " + k); out[k]=v;
    }); return out;
  }
  function point(t,p) {
    var q=Math.max(0,t*p.speedScale),u,x=34,z=-31,y=32,yaw=0,phase="boost";
    if(q<3.5) {u=q/3.5;x=34+33*u*u;}
    else if(q<5.2) {u=(q-3.5)/1.7;x=67+29*u;y=32-4*(3*u*u-2*u*u*u)+1.6*u*(1-u);phase="flight1";}
    else if(q<7.6) {u=(q-5.2)/2.4;x=96+21*(2*u-u*u);z=-31+17*u*u;y=28;yaw=-Math.atan2(34*u,42*(1-u));phase="turn";}
    else if(q<9.4) {u=(q-7.6)/1.8;x=117;z=-14+28*u;y=28-4*(3*u*u-2*u*u*u)+1.6*u*(1-u);yaw=-Math.PI/2;phase="flight2";}
    else if(q<12.5) {u=(q-9.4)/3.1;var slope=(28/1.8)*3.1/30;
      x=117+1.5*Math.pow(Math.sin(Math.PI*u),2);z=14+30*(-2*u*u*u+3*u*u+slope*(u*u*u-2*u*u+u));y=24;
      yaw=-Math.PI/2+(3*u*u-2*u*u*u)*110*Math.PI/180;phase="brake";
    } else {x=117;z=44;y=24;yaw=-Math.PI/2+110*Math.PI/180;phase="landed";}
    return {x:x,y:y+CG,z:z,yaw:yaw,phase:phase};
  }
  function reference(t,p) {
    var h=0.02,a=point(t-h,p),b=point(t,p),c=point(t+h,p);
    ["x","y","z"].forEach(function(k) { b["v"+k]=(c[k]-a[k])/(2*h);b["a"+k]=clamp((c[k]-2*b[k]+a[k])/(h*h),-8,8); });
    b.omega=angle(c.yaw-a.yaw)/(2*h);return b;
  }
  function phaseLabel(phase) { return {boost:"Tăng tốc trong tháp 1",flight1:"Bay sang tháp 2",turn:"Đổi hướng trong tháp 2",flight2:"Bay sang tháp 3",brake:"Phản lực ngược / hãm",landed:"Hạ cánh"}[phase] || phase; }
  function rollout(input,policyInput,options) {
    var p=params(input),pol=policy(policyInput),record=!(options && options.record===false),mass=MASS*p.massScale,events=[],states=[];
    // Retiming the reference avoids requesting an impossible speed. Feedback brakes
    // overspeed through bounded forces; the integrated velocity is never clipped.
    var refParams=Object.assign({},p,{speedScale:Math.min(p.speedScale,pol.maxSpeed/19.5)});
    var available=pol.maxPodForce*p.thrustScale*(4-p.podLoss),hover=mass*G;
    if(available<hover*1.05) return {params:p,policy:pol,dt:DT,states:[],events:[],duration:0,hash:"0",metrics:{outcome:"infeasible",reason:"insufficient_hover_reserve",feasible:false,gateCount:0,minClearance:null,score:1000000}};
    var target=point(0,refParams),s={x:target.x,y:target.y,z:target.z+p.offsetZ,vx:0,vy:0,vz:0,yaw:0,omega:0};
    var forces=OFFSETS.map(function(_,i){return [0,Math.min(MASS*G/4,pol.maxPodForce*p.thrustScale*(i===0?1-p.podLoss:1)),0];}),queue=[],delaySteps=Math.round(p.delay/DT),gateSeen=[false,false,false,false];
    var failed=null,minClear=10,peakError=0,peakSpeed=0,saturated=0,steps=0,hash=2166136261,lastT=0;
    var end=15/refParams.speedScale,landed=false,I=MASS*p.massScale*2.8;
    function snapshot(t,ref) {
      var out={t:t,x:s.x,y:s.y,z:s.z,vx:s.vx,vy:s.vy,vz:s.vz,yaw:s.yaw,omega:s.omega,phase:ref.phase,jets:forces.map(function(v){return v.slice();})};
      if(record) states.push(out);
      [s.x,s.y,s.z,s.yaw].forEach(function(v){hash=Math.imul((hash ^ Math.round(v*1e5))>>>0,16777619)>>>0;});
    }
    snapshot(0,target);
    for(var step=1;step<=Math.ceil(end/DT);step++) {
      var t=step*DT,ref=reference(t,refParams),old={x:s.x,y:s.y,z:s.z},c=Math.cos(s.yaw),sn=Math.sin(s.yaw);
      var acc=[ref.ax+pol.kpXY*(ref.x-s.x)+pol.kdXY*(ref.vx-s.vx),
        ref.ay+pol.kpY*(ref.y-s.y)+pol.kdY*(ref.vy-s.vy),
        ref.az+pol.kpXY*(ref.z-s.z)+pol.kdXY*(ref.vz-s.vz)];
      var speed=Math.hypot(s.vx,s.vy,s.vz),velocity=[s.vx,s.vy,s.vz];
      if(speed>pol.maxSpeed*0.95){
        var along=acc.reduce(function(sum,a,k){return sum+a*velocity[k]/speed;},0),allowed=(pol.maxSpeed-speed)*4;
        if(along>allowed)acc=acc.map(function(a,k){return a-(along-allowed)*velocity[k]/speed;});
      }
      var total=[MASS*acc[0],MASS*(G+acc[1]),MASS*acc[2]];
      var torque=MASS*2.8*clamp(pol.kpYaw*angle(ref.yaw-s.yaw)+pol.kdYaw*(ref.omega-s.omega),-12,12);
      var radii=OFFSETS.map(function(o){return [c*o[0]+sn*o[1],-sn*o[0]+c*o[1]];});
      var denom=radii.reduce(function(a,r){return a+r[0]*r[0]+r[1]*r[1];},0);
      var command=radii.map(function(r){return [total[0]/4+torque*r[1]/denom,total[1]/4,total[2]/4-torque*r[0]/denom];});
      queue.push(command);var delayed=queue.length>delaySteps ? queue.shift() : forces;
      var sum=[0,0,0],yawTorque=0;
      forces.forEach(function(f,i){
        var desired=delayed[i].slice(),limit=pol.maxPodForce*p.thrustScale*(i===0?1-p.podLoss:1),length=Math.hypot.apply(null,desired);
        if(length>limit){saturated++;desired=desired.map(function(v){return v*limit/length;});}
        var diff=desired.map(function(v,k){return v-f[k];}),change=Math.hypot.apply(null,diff),amount=Math.min(1,pol.forceSlew*DT/Math.max(1e-9,change));
        for(var k=0;k<3;k++){f[k]+=diff[k]*amount;sum[k]+=f[k];}
        yawTorque+=radii[i][1]*f[0]-radii[i][0]*f[2];
      });
      var gust=p.windGust*Math.sin(t*2.1+0.4)*Math.sin(t*0.7);
      s.vx+=(sum[0]/mass+pol.drag*MASS/mass*(p.windX+gust-s.vx))*DT;
      s.vy+=(sum[1]/mass-G-pol.drag*MASS/mass*s.vy)*DT;
      s.vz+=(sum[2]/mass+pol.drag*MASS/mass*(p.windZ+gust*0.6-s.vz))*DT;
      s.omega+=(yawTorque/I+p.yawBias-0.5*s.omega)*DT;
      s.yaw=angle(s.yaw+s.omega*DT);s.x+=s.vx*DT;s.y+=s.vy*DT;s.z+=s.vz*DT;
      var ex=2.5*Math.abs(Math.cos(s.yaw))+1.5*Math.abs(Math.sin(s.yaw));
      var ez=2.5*Math.abs(Math.sin(s.yaw))+1.5*Math.abs(Math.cos(s.yaw));
      // Check all four facade planes of every tower. Passing the intended glass is permitted.
      TOWERS.forEach(function(b,bi){
        if(failed)return;
        ["west","east","north","south"].forEach(function(face){
          if(failed)return;var axis=face==="west"||face==="east"?"x":"z",other=axis==="x"?"z":"x";
          var plane=b[axis]+(face==="west"||face==="north"?-1:1)*(axis==="x"?b.w:b.d)/2,da=old[axis]-plane,db=s[axis]-plane;
          if(da*db>0 || Math.abs(s[axis]-old[axis])<1e-10)return;
          var alpha=clamp((plane-old[axis])/(s[axis]-old[axis]),0,1),lateral=old[other]+alpha*(s[other]-old[other]),y=old.y+alpha*(s.y-old.y);
          if(Math.abs(lateral-b[other])>(axis==="x"?b.d:b.w)/2 || y-CG>b.h || y+CG<0)return;
          var gi=GATES.findIndex(function(g){return g.tower===bi&&g.face===face;}),gate=GATES[gi];
          var sideMargin=gate?3.25-Math.abs(lateral-gate[other])-(axis==="x"?ez:ex):-10;
          var topMargin=b.floor+5.5-y-CG,bottomMargin=y-CG-b.floor;
          var margin=Math.min(sideMargin,topMargin);minClear=Math.min(minClear,margin);
          if(!gate || sideMargin<0 || topMargin<0 || bottomMargin< -0.2){if(bottomMargin< -0.2)minClear=Math.min(minClear,bottomMargin);failed={reason:"wall_tower_"+(bi+1),t:t,x:s.x,y:s.y,z:s.z,clearance:Math.min(margin,bottomMargin)};return;}
          if(!gateSeen[gi]){gateSeen[gi]=true;events.push({kind:"glass",gate:gi,t:t,x:gate.x,y:gate.y,z:gate.z});s.vx*=0.97;s.vz*=0.97;}
        });
      });
      // Landing on a passage floor is a support reaction, rather than a facade collision.
      TOWERS.forEach(function(b){
        if(failed || Math.abs(s.x-b.x)>b.w/2 || Math.abs(s.z-b.z)>b.d/2)return;
        if(s.y+CG>b.floor+5.8 && s.y-CG<b.h){failed={reason:"ceiling",t:t,x:s.x,y:s.y,z:s.z};return;}
        if(s.y< b.floor+CG){
          if(s.vy< -5){failed={reason:"hard_landing",t:t,x:s.x,y:s.y,z:s.z};return;}
          s.y=b.floor+CG;s.vy=Math.max(0,s.vy);
        }
      });
      if(s.y<CG && !failed)failed={reason:"ground_contact",t:t,x:s.x,y:s.y,z:s.z};
      if(![s.x,s.y,s.z,s.vx,s.vy,s.vz,s.yaw,s.omega].every(Number.isFinite))throw new Error("Non-finite integration state");
      peakError=Math.max(peakError,Math.hypot(s.x-ref.x,s.y-ref.y,s.z-ref.z));peakSpeed=Math.max(peakSpeed,Math.hypot(s.vx,s.vy,s.vz));steps++;
      snapshot(t,ref);lastT=t;if(failed)break;
      if(ref.phase==="landed" && gateSeen.every(Boolean) && Math.hypot(s.x-117,s.z-44)<1.5 && Math.abs(s.y-(24+CG))<0.3 && Math.hypot(s.vx,s.vy,s.vz)<0.8){landed=true;break;}
    }
    if(failed)events.push(Object.assign({kind:"collision"},failed));
    var outcome=failed?"crash":!landed?"incomplete":minClear<0.45?"near":"safe";
    var reason=failed?failed.reason:!landed?"route_incomplete":outcome==="near"?"tight_gate":"completed";
    var score=failed?-2-Math.max(0,-(failed.clearance||0)):!landed?-1: minClear;
    return {params:p,referenceParams:refParams,policy:pol,dt:DT,states:states,events:events,duration:lastT,hash:hash.toString(16),metrics:{outcome:outcome,reason:reason,feasible:true,gateCount:gateSeen.filter(Boolean).length,minClearance:minClear,peakTrackingError:peakError,peakSpeed:peakSpeed,speedLimit:pol.maxSpeed,peakOverspeed:Math.max(0,peakSpeed-pol.maxSpeed),saturationFraction:saturated/Math.max(1,steps*4),landed:landed,score:score}};
  }
  function at(run,t) {
    if(!run.states.length)return null;var i=Math.min(run.states.length-1,Math.max(0,Math.floor(t/run.dt))),a=run.states[i],b=run.states[Math.min(i+1,run.states.length-1)],u=clamp((t-a.t)/run.dt,0,1),p={t:t,phase:a.phase,jets:a.jets};
    ["x","y","z","vx","vy","vz","omega"].forEach(function(k){p[k]=a[k]+(b[k]-a[k])*u;});p.yaw=a.yaw+angle(b.yaw-a.yaw)*u;return p;
  }
  return {VERSION:VERSION,DT:DT,CG:CG,TOWERS:TOWERS,GATES:GATES,SPACE:SPACE,POLICY:POLICY,params:params,policy:policy,reference:reference,rollout:rollout,at:at,phaseLabel:phaseLabel};
});
