import { clamp } from '../core/utils.js';
export class ForwardCliffSensor {
  sample({state:s, config:c, terrain}) {
    const lookAhead = clamp(Math.max(s.speed, 5)*1.8, 12, 45), probes=[];
    for (let distance=3; distance<=lookAhead; distance+=3) {
      const x=s.x+Math.cos(s.yaw)*distance, z=s.z+Math.sin(s.yaw)*distance;
      const hit=terrain.surfaceAt(x,z,c);
      probes.push({from:{x,y:s.y+1.1,z},to:{x,y:hit?.y??c.waterY,z},
        hit:!!hit, safe:hit?.kind==='ground' && Math.abs(hit.y)<=.5, distance});
    }
    const danger=probes.find(p=>!p.safe), edge=terrain.edgeDistance(s,c);
    return {distanceToGroundAhead:danger?Math.max(0,edge):lookAhead,
      roadAheadExists:!danger, cliffRiskScore:danger?clamp(1-Math.max(0,edge)/lookAhead,0,1):0, forwardProbes:probes};
  }
}
