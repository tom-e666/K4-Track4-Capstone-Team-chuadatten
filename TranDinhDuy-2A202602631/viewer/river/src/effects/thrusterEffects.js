import { THRUSTER_KEYS } from '../core/constants.js';
export function createThrusterEffects(T,mounts) {
  const effects=mounts.map(mount=>{
    const outer=new T.Mesh(new T.ConeGeometry(.2,1,16),new T.MeshBasicMaterial({color:0x1689ff,transparent:true,opacity:.8,depthWrite:false,toneMapped:false}));
    const inner=new T.Mesh(new T.ConeGeometry(.10,1,12),new T.MeshBasicMaterial({color:0xe5f9ff,transparent:true,opacity:.95,depthWrite:false,toneMapped:false,blending:T.AdditiveBlending}));
    outer.rotation.z=inner.rotation.z=Math.PI;mount.add(outer,inner);
    const halo=new T.Mesh(new T.SphereGeometry(.2,12,8),new T.MeshBasicMaterial({color:0x60cfff,transparent:true,opacity:.25,depthWrite:false,blending:T.AdditiveBlending}));
    halo.scale.y=.45;halo.position.y=-.13;mount.add(halo);
    const particlePositions=new Float32Array(30*3),geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.BufferAttribute(particlePositions,3));
    const particles=new T.Points(geometry,new T.PointsMaterial({color:0x8fd9f5,size:.055,transparent:true,opacity:.65,depthWrite:false,blending:T.AdditiveBlending}));
    mount.add(particles);return {outer,inner,halo,particles,power:0,mount};
  });
  let time=0;
  return {update(state,powers,dt){
    time+=dt;
    effects.forEach((fx,i)=>{
      const target=state.flightActive?(powers[THRUSTER_KEYS[i]]??0):0;
      fx.power+=(target-fx.power)*(1-Math.exp(-dt/.12));
      fx.mount.position.y=.35-fx.power*.20;
      const length=fx.power*(2.8+.18*Math.sin(time*49+i));
      fx.outer.scale.y=length;fx.outer.position.y=-.16-length/2;
      fx.inner.scale.y=length*.66;fx.inner.position.y=-.16-length*.33;
      fx.outer.visible=fx.inner.visible=fx.halo.visible=fx.particles.visible=fx.power>.015;
      fx.halo.material.opacity=fx.power*.6;
      const attribute=fx.particles.geometry.attributes.position;
      for(let j=0;j<30;j++){const age=(time*2+j/30+i*.13)%1;
        attribute.setXYZ(j,Math.sin(j*7+i)*age*.3,-.2-age*(length+1.2),Math.cos(j*9+i)*age*.3);}
      attribute.needsUpdate=true;fx.particles.material.opacity=fx.power*.55;
    });
  }};
}
