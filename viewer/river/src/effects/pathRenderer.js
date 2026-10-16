export function createPathRenderer(T,scene){
  const group=new T.Group();scene.add(group);
  function path(color,dashed=false,segments=false){const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.BufferAttribute(new Float32Array(2400*3),3));geometry.setDrawRange(0,0);
    const material=dashed?new T.LineDashedMaterial({color,dashSize:.75,gapSize:.45,depthTest:false,toneMapped:false}):new T.LineBasicMaterial({color,transparent:true,opacity:.95,depthTest:false,toneMapped:false});
    const line=segments?new T.LineSegments(geometry,material):new T.Line(geometry,material);line.frustumCulled=false;line.renderOrder=2;group.add(line);return line;}
  const fall=path(0xff9a4c,true),rescue=path(0x4edbff),trail=path(0x9bf5e4),rays=path(0xf4d46c,false,true);
  const skid=path(0x243138,false,true);
  const marker=new T.Mesh(new T.SphereGeometry(.38,12,8),new T.MeshBasicMaterial({color:0xff9a4c,depthTest:false}));group.add(marker);
  function write(line,points){const attr=line.geometry.attributes.position;const count=Math.min(points.length,attr.count);
    for(let i=0;i<count;i++)attr.setXYZ(i,points[i].x,points[i].y+.1,points[i].z);attr.needsUpdate=true;line.geometry.setDrawRange(0,count);if(line===fall)line.computeLineDistances();}
  return {group,update(observation,history,toggles){
    write(fall,observation.prediction.predictedFallPath);fall.visible=toggles.fall;
    write(rescue,observation.guidance.rescuePath);rescue.visible=toggles.rescue&&observation.vehicle.flightAvailable;
    write(trail,history.map(f=>({x:f.state.x,y:f.state.y+.2,z:f.state.z})));trail.visible=toggles.rescue;
    const marks=[];
    for(let i=1;i<history.length;i++)if(history[i].state.grounded&&history[i-1].state.grounded&&history[i].state.skidding){
      for(const side of [-.82,.82])for(const index of [i-1,i]){
        const s=history[index].state;marks.push({x:s.x-1.45*Math.cos(s.yaw)-side*Math.sin(s.yaw),y:-.01,z:s.z-1.45*Math.sin(s.yaw)+side*Math.cos(s.yaw)});
      }
    }
    write(skid,marks);
    const probes=[...(observation.sensors.forwardProbes??[]),...(observation.sensors.downwardProbes??[])];
    write(rays,probes.flatMap(p=>[p.from,p.to]));rays.visible=toggles.rays;
    const impact=observation.prediction.predictedImpactPoint;marker.visible=toggles.fall&&!!impact;if(impact)marker.position.set(impact.x,impact.y+.25,impact.z);
  }};
}
