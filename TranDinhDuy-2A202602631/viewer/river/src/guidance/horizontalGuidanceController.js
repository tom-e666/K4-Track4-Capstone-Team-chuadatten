import { clamp, wrapAngle } from '../core/utils.js';
export function horizontalGuidance(state,target,hover=false) {
  const remaining=target.x-state.x;
  const desiredSpeed=hover?0:Math.min(10,Math.sqrt(Math.max(0,2*2.6*(remaining-1))));
  const desiredYaw=Math.atan2((target.z-state.z)*.45,Math.max(3,remaining));
  return {throttle:clamp((desiredSpeed-state.speed)*.5,0,1),brake:clamp((state.speed-desiredSpeed)*.55,0,1),
    steer:clamp(wrapAngle(desiredYaw-state.yaw)*3,-1,1)};
}

// Flight translation is independent of body yaw while the car is still spinning.
export function worldGuidance(state,target,{hover=false,stabilizing=false}={}){
  const dx=target.x-state.x,dz=target.z-state.z,distance=Math.hypot(dx,dz);
  const desiredSpeed=hover||stabilizing?0:Math.min(10,Math.sqrt(Math.max(0,2*2.6*(distance-.65))));
  const vx=distance>.01?dx/distance*desiredSpeed:0,vz=distance>.01?dz/distance*desiredSpeed:0;
  return {acceleration:{x:clamp((vx-state.vx)*1.8,-10,5),z:clamp((vz-state.vz)*1.8,-6,6)},
    throttle:clamp((desiredSpeed-state.speed)*.5,0,1),brake:clamp((state.speed-desiredSpeed)*.55,0,1),steer:0};
}
