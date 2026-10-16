// Ballistic counterfactual: integrate current velocity + gravity with NO thrust or braking.
export class TrajectoryPredictor {
  predict({state:s,config:c,terrain}) {
    const path=[],velocity={x:s.vx,y:s.vy,z:s.vz};
    let x=s.x,y=s.y,z=s.z,delay=0,impact=null;
    if(s.grounded && s.x<0) {
      if(s.vx<=.1)return {predictedFallPath:[],predictedImpactPoint:null,timeToImpact:null};
      delay=Math.max(0,(-s.x-2.3)/s.vx);path.push({x,y:y+.15,z,t:0});
      x=-2.3;z+=s.vz*delay;velocity.y=0;
    }
    for(let t=0;t<=8;t+=.08) {
      const point={x:x+velocity.x*t,y:y+velocity.y*t-.5*c.gravity*t*t,z:z+velocity.z*t,t:delay+t};
      const surface=terrain.surfaceAt(point.x,point.z,c);
      path.push(point);
      // Ignore launch-bank support while the centre passes its edge.
      if(t>.08 && surface && point.x>0 && point.y<=surface.y) {
        const previous=path.at(-2),wall=surface.kind==='ground'&&previous.x<c.riverWidth&&point.y<0;
        const alpha=wall?(c.riverWidth-previous.x)/(point.x-previous.x):(surface.y-previous.y)/(point.y-previous.y);
        const u=Math.max(0,Math.min(1,alpha));
        impact={x:previous.x+(point.x-previous.x)*u,y:previous.y+(point.y-previous.y)*u,
          z:previous.z+(point.z-previous.z)*u,kind:wall?'cliff':surface.kind};
        path[path.length-1]={...impact,t:previous.t+(point.t-previous.t)*u};break;
      }
    }
    return {predictedFallPath:path,predictedImpactPoint:impact,timeToImpact:impact?path.at(-1).t:null};
  }
}
