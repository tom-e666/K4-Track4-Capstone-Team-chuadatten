import { createThrusterEffects } from '../effects/thrusterEffects.js';
import { createAttitudeEffects } from '../effects/attitudeEffects.js';
import { createSkidEffects } from '../effects/skidEffects.js';
export function createVehicleModel(T) {
  const group=new T.Group();group.name='RescueCar';
  const paint=new T.MeshStandardMaterial({color:0xd9dedf,metalness:.65,roughness:.33});
  const dark=new T.MeshStandardMaterial({color:0x142630,metalness:.5,roughness:.3});
  const trim=new T.MeshStandardMaterial({color:0x426a78,metalness:.8,roughness:.3});
  function box(material,x,y,z,sx,sy,sz){const mesh=new T.Mesh(new T.BoxGeometry(sx,sy,sz),material);mesh.position.set(x,y,z);group.add(mesh);return mesh;}
  box(paint,0,.66,0,4.5,.6,1.9).name='RescueCarBody';
  box(dark,-.35,1.15,0,2.3,.65,1.67);
  box(paint,-.38,1.51,0,1.95,.08,1.72);
  const windshield=box(dark,.85,1.19,0,.12,.65,1.68);windshield.rotation.z=.4;
  box(trim,0,.43,0,4.1,.14,1.75);
  const light=new T.MeshBasicMaterial({color:0xc7f0ff}),rearLight=new T.MeshBasicMaterial({color:0xeb644e});
  box(light,2.26,.72,0,.04,.07,1.65);box(rearLight,-2.26,.72,0,.04,.07,1.5);
  for(const x of [-1.48,1.48])for(const z of [-.94,.94]){
    const wheel=new T.Mesh(new T.CylinderGeometry(.36,.36,.26,18),dark);wheel.rotation.x=Math.PI/2;wheel.position.set(x,.36,z);group.add(wheel);
    const hub=new T.Mesh(new T.CylinderGeometry(.19,.19,.28,12),paint);hub.rotation.x=Math.PI/2;hub.position.copy(wheel.position);group.add(hub);
  }
  const mounts=[];
  // Embedded underbody emitters, no outboard arms or spinning hardware.
  for(const [name,x,z] of [['FrontLeftThruster',1.45,-.72],['FrontRightThruster',1.45,.72],['RearLeftThruster',-1.45,-.72],['RearRightThruster',-1.45,.72]]){
    const mount=new T.Group();mount.name=name;mount.position.set(x,.35,z);
    const housing=new T.Mesh(new T.CylinderGeometry(.22,.29,.30,16),trim);
    const opening=new T.Mesh(new T.CylinderGeometry(.17,.23,.08,16),dark);opening.position.y=-.17;
    const core=new T.Mesh(new T.CircleGeometry(.17,20),light);core.rotation.x=Math.PI/2;core.position.y=-.215;
    mount.add(housing,opening,core);group.add(mount);mounts.push(mount);
  }
  group.traverse(object=>{if(object.isMesh)object.castShadow=true;});
  const effects=createThrusterEffects(T,mounts),attitude=createAttitudeEffects(T,group),skid=createSkidEffects(T,group);
  return {group,mounts,update(state,thrusters,dt,attitudePowers){
    group.position.set(state.x,state.y,state.z);group.rotation.set(state.roll,-state.yaw,state.pitch,'YXZ');
    effects.update(state,thrusters,dt);attitude.update(state,attitudePowers,dt);skid.update(state,dt);
  },dispose(){group.traverse(o=>{o.geometry?.dispose();if(o.material){o.material.map?.dispose();o.material.dispose();}});}};
}
