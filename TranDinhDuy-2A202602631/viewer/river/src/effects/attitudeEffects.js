// Reaction-control jets around the body. Directions are exhaust directions, not thrust.
export function createAttitudeEffects(T,car){
  const definitions=[
    ['FrontLeftYawJet','yawPositive',1.85,.76,-1,[0,0,-1]],['RearRightYawJet','yawPositive',-1.85,.76,1,[0,0,1]],
    ['FrontRightYawJet','yawNegative',1.85,.76,1,[0,0,1]],['RearLeftYawJet','yawNegative',-1.85,.76,-1,[0,0,-1]],
    ['RearPitchJet','pitchPositive',-1.9,.98,0,[0,1,0]],['FrontPitchJet','pitchNegative',1.9,.98,0,[0,1,0]],
    ['RightRollJet','rollPositive',0,1.54,.81,[0,1,0]],['LeftRollJet','rollNegative',0,1.54,-.81,[0,1,0]]
  ];
  const effects=definitions.map(([name,key,x,y,z,direction])=>{
    const mount=new T.Group();mount.name=name;mount.position.set(x,y,z);
    mount.quaternion.setFromUnitVectors(new T.Vector3(0,-1,0),new T.Vector3(...direction));
    const housing=new T.Mesh(new T.CylinderGeometry(.10,.14,.15,12),new T.MeshStandardMaterial({color:0x253d4d,metalness:.8,roughness:.3}));
    const core=new T.Mesh(new T.CircleGeometry(.09,16),new T.MeshBasicMaterial({color:0x9ee8ff,toneMapped:false}));core.rotation.x=Math.PI/2;core.position.y=-.08;
    const flame=new T.Mesh(new T.ConeGeometry(.15,1,12),new T.MeshBasicMaterial({color:0x4bc8ff,transparent:true,opacity:.8,toneMapped:false,depthWrite:false}));
    const white=new T.Mesh(new T.ConeGeometry(.065,1,12),new T.MeshBasicMaterial({color:0xeeffff,transparent:true,opacity:.9,toneMapped:false,depthWrite:false}));
    flame.rotation.z=white.rotation.z=Math.PI;mount.add(housing,core,flame,white);car.add(mount);
    return {mount,key,flame,white,power:0};
  });
  let time=0;
  return {update(state,powers={},dt){time+=dt;effects.forEach((fx,i)=>{
    const demand=state.flightActive?(powers[fx.key]??0):0;
    fx.power+=(demand-fx.power)*(1-Math.exp(-dt/.055));
    // A visible pulse still represents the same mean force used by the attitude model.
    const length=fx.power*(1.4+.2*Math.sin(time*43+i));
    fx.flame.visible=fx.white.visible=fx.power>.025;fx.flame.scale.y=length;fx.flame.position.y=-.08-length/2;
    fx.white.scale.y=length*.7;fx.white.position.y=-.08-length*.35;
  });}};
}
