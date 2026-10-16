import { clamp } from '../core/utils.js';
import { VEHICLE } from '../core/constants.js';
export class StabilizerController {
  altitudeController(y,vy,target,c) {return clamp((c.gravity+clamp((target-y)*2.2-vy*2.8,-4,9))/c.maxLift,0,1);}
  pitchStabilizer(s,c) {return (-8*s.pitch-4*s.pitchRate)*VEHICLE.pitchInertiaPerMass/(c.maxLift*VEHICLE.leverX);}
  rollStabilizer(s,c) {return (-9*s.roll-4.5*s.rollRate)*VEHICLE.rollInertiaPerMass/(c.maxLift*VEHICLE.leverZ);}
  mix(state,base,config) {
    const p=this.pitchStabilizer(state,config),r=this.rollStabilizer(state,config);
    return {frontLeft:clamp(base+p+r,0,1),frontRight:clamp(base+p-r,0,1),
      rearLeft:clamp(base-p+r,0,1),rearRight:clamp(base-p-r,0,1)};
  }
}
