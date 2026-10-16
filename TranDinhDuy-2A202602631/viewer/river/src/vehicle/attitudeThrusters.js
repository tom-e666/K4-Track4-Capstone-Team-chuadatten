import { clamp, finite, wrapAngle } from '../core/utils.js';
export const ATTITUDE_KEYS=Object.freeze(['yawPositive','yawNegative','pitchPositive','pitchNegative','rollPositive','rollNegative']);
export function attitudeCommand(measured,targetYaw=0){
  // Brake fast rotation before choosing the shortest heading correction at the ±PI branch cut.
  const yaw=clamp(Math.abs(measured.yawRate)>1?-4.5*measured.yawRate:5*wrapAngle(targetYaw-measured.yaw)-3.8*measured.yawRate,-6,6)/6;
  const pitch=clamp(-3*measured.pitch-2.5*measured.pitchRate,-4,4)/4;
  const roll=clamp(-3*measured.roll-2.5*measured.rollRate,-4,4)/4;
  return {yawPositive:Math.max(0,yaw),yawNegative:Math.max(0,-yaw),pitchPositive:Math.max(0,pitch),pitchNegative:Math.max(0,-pitch),
    rollPositive:Math.max(0,roll),rollNegative:Math.max(0,-roll)};
}
export class AttitudeThrusters {
  constructor(){this.reset();}
  reset(){this.power=Object.fromEntries(ATTITUDE_KEYS.map(key=>[key,0]));}
  step(command,active,dt){
    const blend=1-Math.exp(-dt/.045);
    for(const key of ATTITUDE_KEYS)this.power[key]+=((active?clamp(finite(command[key]??0,key),0,1):0)-this.power[key])*blend;
    return this.frame();
  }
  frame(){const p=this.power;return {...p,yawAcceleration:6*(p.yawPositive-p.yawNegative),pitchAcceleration:4*(p.pitchPositive-p.pitchNegative),
    rollAcceleration:4*(p.rollPositive-p.rollNegative),totalPower:ATTITUDE_KEYS.reduce((sum,key)=>sum+p[key],0)/6};}
}
