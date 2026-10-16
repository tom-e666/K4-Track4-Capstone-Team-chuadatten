import { THRUSTER_KEYS, VEHICLE } from '../core/constants.js';
import { clamp, finite } from '../core/utils.js';
export class ThrusterSystem {
  constructor(){this.reset();}
  reset(){this.power=Object.fromEntries(THRUSTER_KEYS.map(k=>[k,0]));}
  step(command,active,config,dt) {
    const blend=1-Math.exp(-dt/.07);
    for(const key of THRUSTER_KEYS) this.power[key]+=(active?clamp(finite(command[key],key),0,1)-this.power[key]:-this.power[key])*blend;
    return this.frame(config);
  }
  frame(c) {
    const p=this.power,mean=THRUSTER_KEYS.reduce((sum,k)=>sum+p[k],0)/4;
    return {...p,totalLift:mean,totalForceN:mean*c.maxLift*VEHICLE.mass,
      pitchAcceleration:(p.frontLeft+p.frontRight-p.rearLeft-p.rearRight)*c.maxLift/4*VEHICLE.leverX/VEHICLE.pitchInertiaPerMass,
      rollAcceleration:(p.frontLeft+p.rearLeft-p.frontRight-p.rearRight)*c.maxLift/4*VEHICLE.leverZ/VEHICLE.rollInertiaPerMass};
  }
}
