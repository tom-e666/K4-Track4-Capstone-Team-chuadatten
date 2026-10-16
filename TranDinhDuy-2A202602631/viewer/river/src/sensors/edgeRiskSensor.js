import { clamp } from '../core/utils.js';
export class EdgeRiskSensor {
  sample({state:s,config:c,terrain}) {
    const distance=terrain.edgeDistance(s,c);
    return {edgeDistance:distance,edgeRiskScore:clamp(1-Math.max(0,distance)/12,0,1)};
  }
}
