import { DT, VEHICLE, THRUSTER_KEYS } from '../core/constants.js';
import { makeConfig } from '../core/config.js';
import { clamp, copy, finite } from '../core/utils.js';
import { MockTerrain } from '../scene/road.js';
import { createLandingZone, defaultLandingPolicy } from '../scene/landingZone.js';
import { SensorSuite } from '../sensors/sensorSuite.js';
import { TrajectoryPredictor } from '../guidance/trajectoryPredictor.js';
import { RescueGuidance } from '../guidance/rescueGuidance.js';
import { StabilizerController } from '../guidance/stabilizerController.js';
import { ThrusterSystem } from './thrusterSystem.js';
import { VehicleStateMachine } from './vehicleStateMachine.js';
import { AttitudeThrusters, ATTITUDE_KEYS, attitudeCommand } from './attitudeThrusters.js';
import { driveStep } from './drivingDynamics.js';

export function normalizeCommand(command) {
  if(!command||typeof command!=='object'||Array.isArray(command)||typeof command.then==='function')throw new Error('Controller must return a synchronous command object');
  const out={};
  for(const key of ['throttle','brake','steer','lift'])out[key]=clamp(finite(command[key]??0,key),key==='steer'?-1:0,1);
  if(command.flight!==undefined&&typeof command.flight!=='boolean')throw new Error('flight must be boolean');
  out.flight=command.flight===true;
  if(command.acceleration!==undefined){
    if(!command.acceleration||typeof command.acceleration!=='object')throw new Error('acceleration requires x and z');
    out.acceleration={x:clamp(finite(command.acceleration.x,'acceleration.x'),-10,8),z:clamp(finite(command.acceleration.z,'acceleration.z'),-8,8)};
  }
  if(command.attitude!==undefined){
    if(!command.attitude||typeof command.attitude!=='object')throw new Error('attitude requires an object');
    out.attitude=Object.fromEntries(ATTITUDE_KEYS.map(key=>[key,clamp(finite(command.attitude[key]??0,key),0,1)]));
  }
  if(command.thrusters!==undefined){
    if(!command.thrusters||typeof command.thrusters!=='object')throw new Error('thrusters must contain four powers');
    out.thrusters=Object.fromEntries(THRUSTER_KEYS.map(key=>[key,clamp(finite(command.thrusters[key],key),0,1)]));
    out.lift=THRUSTER_KEYS.reduce((sum,key)=>sum+out.thrusters[key],0)/4;
  }
  return out;
}

export class Engine {
  constructor(options={},dependencies={}) {
    this.terrain=dependencies.terrain??new MockTerrain();this.sensorSuite=new SensorSuite(dependencies.sensors);
    this.predictor=dependencies.predictor??new TrajectoryPredictor();this.guidance=dependencies.guidance??new RescueGuidance();
    this.landingPolicy=dependencies.landingPolicy??defaultLandingPolicy;this.zoneProvider=dependencies.landingZoneProvider??createLandingZone;
    this.thrusters=new ThrusterSystem();this.rcs=new AttitudeThrusters();this.stabilizer=new StabilizerController();this.machine=new VehicleStateMachine();this.reset(options);
  }
  reset(options={}) {
    this.config=makeConfig(options);this.runId=`rescue-${Date.now()}-${Math.random().toString(36).slice(2,8)}`;
    this.seq=0;this.events=[];this.departed=false;this.command=normalizeCommand({});this.thrusters.reset();this.rcs.reset();this.machine.reset();
    this.state={t:0,x:-95,y:0,z:0,vx:this.config.initialSpeed,vy:0,vz:0,yaw:0,speed:this.config.initialSpeed,
      pitch:0,roll:0,pitchRate:0,rollRate:0,yawRate:0,grounded:true,phase:'approach',mode:'DRIVING',
      flightActive:false,rescueTriggered:false,autoRescueEnabled:this.config.autoRescueEnabled,
      hoverMode:false,forceLanding:false,terminal:false,outcome:null,driverAttentionSince:null,driverReacted:false,
      skidding:false,slipAngle:0,accidentOccurred:false,airborneSince:null,rescueSince:null,hoverSince:null};
    this.refresh();return this.observation();
  }
  refresh() {
    this.landingZone=this.zoneProvider(this.config);
    const context={state:copy(this.state),config:{...this.config},terrain:this.terrain,landingZone:copy(this.landingZone)};
    this.sensors=this.sensorSuite.sample(context);this.prediction=this.predictor.predict(context);this.flightGuidance=this.guidance.generate(context);
  }
  observation() {
    const c=this.config,s=this.state,zone=this.landingZone;
    return copy({schema:'river-nav/1',type:'observation',runId:this.runId,seq:this.seq,dt:DT,state:s,
      sensors:this.sensors,prediction:this.prediction,guidance:this.flightGuidance,thrusters:this.thrusters.frame(c),attitudeThrusters:this.rcs.frame(),
      map:{nearBankX:0,farBankX:c.riverWidth,waterY:c.waterY,junctionX:-12,roadHalfWidth:5.5,
        landing:zone?{x:zone.position.x,y:zone.position.y,z:zone.position.z,halfX:zone.halfX,halfZ:zone.halfZ,heading:zone.heading}:null},
      vehicle:{...VEHICLE,maxBrake:c.maxBrake,maxLift:c.maxLift,gravity:c.gravity,flightAvailable:c.flightEnabled},
      scenario:{...c},
      metrics:{distanceToEdge:this.sensors.edgeDistance,stoppingDistance:s.speed*s.speed/(2*c.maxBrake)}});
  }
  emit(type){const s=this.state;this.events.push({type,t:s.t,x:s.x,y:s.y,z:s.z,yaw:s.yaw,pitch:s.pitch,roll:s.roll,yawRate:s.yawRate,pitchRate:s.pitchRate,rollRate:s.rollRate,slipAngle:s.slipAngle,vx:s.vx,vy:s.vy,vz:s.vz,speed:s.speed});}
  triggerRescue(){if(this.config.flightEnabled&&!this.state.terminal){this.state.rescueTriggered=true;this.emit('manual_rescue');}}
  teleportToEdge(forceFall=false) {
    const s=this.state;
    Object.assign(s,{x:forceFall?2:-5,y:forceFall?-.5:0,z:0,vx:18,vz:0,vy:forceFall?-3:0,speed:18,
      yaw:0,pitch:forceFall?.22:0,roll:forceFall?-.18:0,pitchRate:0,rollRate:0,grounded:!forceFall,
      terminal:false,outcome:null,flightActive:false,rescueTriggered:false,hoverMode:false,forceLanding:false,
      airborneSince:forceFall?s.t:null,rescueSince:null,accidentOccurred:forceFall,driverReacted:true,driverAttentionSince:s.t-this.config.driverDelay,hoverSince:null});
    this.departed=forceFall;this.thrusters.reset();this.rcs.reset();this.machine.reset();this.emit(forceFall?'force_fall':'teleport_to_edge');this.refresh();
  }
  restore(frame) {
    this.state=copy(frame.state);this.seq=frame.seq;this.thrusters.power=Object.fromEntries(THRUSTER_KEYS.map(key=>[key,frame.thrusters?.[key]??0]));
    this.rcs.power=Object.fromEntries(ATTITUDE_KEYS.map(key=>[key,frame.attitudeThrusters?.[key]??0]));this.departed=this.state.airborneSince!==null;
    this.machine.activeSince=this.state.flightActive?this.state.t-1:null;this.machine.stableSince=this.state.flightActive?this.state.t-1:null;this.refresh();
  }
  finish(outcome){this.state.outcome=outcome;this.state.terminal=true;this.state.phase=outcome;this.state.flightActive=false;this.emit(outcome);}
  step(input) {
    if(this.state.terminal)return this.observation();
    const u=normalizeCommand(input),s=this.state,c=this.config,dt=DT;this.command=u;s.t=+(s.t+dt).toFixed(6);this.seq++;
    const active=c.flightEnabled&&u.flight;
    if(active&&!s.flightActive){s.rescueTriggered=true;s.rescueSince??=s.t;this.emit('rescue_activation');}
    s.flightActive=active;
    const desired=u.thrusters??this.stabilizer.mix(s,u.lift,c),force=this.thrusters.step(desired,active,c,dt);
    const attitude=this.rcs.step(u.attitude??attitudeCommand(s,0),active,dt);
    // Differential nozzle forces produce pitch/roll torque in this simplified body model.
    if(s.grounded){
      if(s.x>=-35&&s.driverAttentionSince===null){s.driverAttentionSince=s.t;this.emit('driver_inattentive');}
      if(!s.driverReacted&&Math.abs(u.steer)>.05){s.driverReacted=true;this.emit('late_turn');}
      const wasSkidding=s.skidding;driveStep(s,u,c,dt);if(s.skidding&&!wasSkidding)this.emit('skid_started');
    }else{
      s.pitchRate+=(force.pitchAcceleration+attitude.pitchAcceleration-.6*s.pitchRate)*dt;
      s.rollRate+=(force.rollAcceleration+attitude.rollAcceleration-.6*s.rollRate)*dt;
      s.yawRate+=(attitude.yawAcceleration-.12*s.yawRate)*dt;
      s.pitch+=s.pitchRate*dt;s.roll+=s.rollRate*dt;s.yaw+=s.yawRate*dt;
      if(active){const a=u.acceleration??{x:Math.cos(s.yaw)*(3.8*u.throttle-7*u.brake),z:Math.sin(s.yaw)*(3.8*u.throttle-7*u.brake)};s.vx+=a.x*dt;s.vz+=a.z*dt;}
      s.vx*=Math.exp(-.025*dt);s.vz*=Math.exp(-.025*dt);s.speed=Math.hypot(s.vx,s.vz);
    }
    s.x+=s.vx*dt;s.z+=s.vz*dt;let horizontal=Math.hypot(s.vx,s.vz);
    const support=this.terrain.supportAt(s,c);
    if(s.grounded&&!support)s.grounded=false;
    if(!this.departed&&s.x> -2.3){
      this.departed=true;s.airborneSince??=s.t;this.emit('left_bank');
      if(!active){s.accidentOccurred=true;s.pitchRate+=c.spinIntensity*.45;s.rollRate-=c.spinIntensity*.6;s.yawRate-=c.spinIntensity*.7;this.emit('ejected_spinning');}
    }
    const accelerationY=force.totalLift*c.maxLift*Math.cos(s.pitch)*Math.cos(s.roll)-c.gravity;
    if(s.grounded&&accelerationY>.1){s.grounded=false;s.airborneSince??=s.t;this.emit('takeoff');}
    if(!s.grounded){
      s.vy+=accelerationY*dt;s.y+=s.vy*dt;const surface=this.terrain.surfaceAt(s.x,s.z,c);
      if(surface?.kind==='ground'&&s.x>c.riverWidth&&s.y<-.3)this.finish('crashed');
      else if(support&&s.y<=0&&s.vy<=0){
        if(Math.abs(s.vy)>4.5||horizontal>13||Math.abs(s.pitch)>.6||Math.abs(s.roll)>.6){s.y=0;this.finish('crashed');}
        else{s.y=0;s.pitch=0;s.roll=0;s.pitchRate=0;s.rollRate=0;s.yawRate=0;s.grounded=true;this.emit('touchdown');s.vy=0;}
      }else if(surface?.kind==='water'&&s.y<=surface.y+.25){s.y=surface.y+.25;this.finish('splash');}
    }
    s.speed=horizontal;
    if(c.landingStyle==='hover'&&!s.grounded&&s.x>c.riverWidth+3&&Math.hypot(s.x-(this.landingZone?.position.x-5),s.z-this.landingZone?.position.z)<4&&horizontal<2&&s.hoverSince===null){s.hoverSince=s.t;this.emit('hover_before_landing');}
    if(!s.terminal){
      if(s.grounded&&s.x>c.riverWidth&&horizontal<.4)this.finish(this.landingPolicy.evaluate(s,this.landingZone)?'landed':'off_target');
      else if(s.grounded&&horizontal<.05&&s.x<0&&!active)this.finish('stopped');
      else if(Math.abs(s.z)>65||s.x>c.riverWidth+85||s.x< -140||s.y>65||s.y<c.waterY-3||s.t>=40)this.finish('out_of_bounds');
      else if(s.grounded)s.phase=u.brake>.1?'braking':'approach';
      else if(!active)s.phase='falling';
      else s.phase=s.vy<-.35&&s.x>c.riverWidth?'landing':'flying';
    }
    this.refresh();const previous=s.mode;this.machine.update(s,this.sensors,c);if(previous!==s.mode)this.emit(`state:${s.mode}`);
    return this.observation();
  }
}
