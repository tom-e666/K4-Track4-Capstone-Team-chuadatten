export class AltitudeSensor {
  sample({state:s,config:c,terrain}) {
    const probes=[[0,0], [1.45,-.72],[1.45,.72],[-1.45,-.72],[-1.45,.72]].map(([dx,dz])=>{
      const x=s.x+Math.cos(s.yaw)*dx-Math.sin(s.yaw)*dz,z=s.z+Math.sin(s.yaw)*dx+Math.cos(s.yaw)*dz;
      const hit=terrain.surfaceAt(x,z,c);
      return {from:{x,y:s.y+.1,z},to:{x,y:hit?.y??c.waterY,z},hit:!!hit};
    });
    const hit=terrain.surfaceAt(s.x,s.z,c);
    return {altitudeAboveGround:hit?Math.max(0,s.y-hit.y):null,
      groundDetectedBelow:hit?.kind==='ground', surfaceDetectedBelow:!!hit, surfaceBelow:hit?.kind??null, downwardProbes:probes};
  }
}
