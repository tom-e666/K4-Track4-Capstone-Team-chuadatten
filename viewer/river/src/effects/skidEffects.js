export function createSkidEffects(T,car){
  const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.BufferAttribute(new Float32Array(60*3),3));
  const material=new T.PointsMaterial({color:0xb9bdba,size:.32,transparent:true,opacity:.35,depthWrite:false});
  const smoke=new T.Points(geometry,material);car.add(smoke);let clock=0;
  return {update(state,dt){clock+=dt;smoke.visible=state.grounded&&state.skidding;if(!smoke.visible)return;
    const attr=geometry.attributes.position;
    for(let i=0;i<60;i++){const age=(clock*1.3+i/60)%1;attr.setXYZ(i,-1.5-age*3,.1+age*.9,(i%2?1:-1)*(.9+age*.4)+Math.sin(i*11)*.2);}
    attr.needsUpdate=true;material.opacity=Math.min(.55,Math.abs(state.slipAngle));
  }};
}
