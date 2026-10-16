export function createWarningEffects(T,scene){
  const marker=new T.Mesh(new T.PlaneGeometry(2.4,11),new T.MeshBasicMaterial({color:0xff8549,transparent:true,opacity:.2,side:T.DoubleSide,depthWrite:false}));
  marker.rotation.x=-Math.PI/2;marker.position.set(-1.2,.13,0);scene.add(marker);
  return {update(frame){marker.visible=frame.cliffRiskScore>.25;marker.material.opacity=.15+frame.cliffRiskScore*.25;}};
}
