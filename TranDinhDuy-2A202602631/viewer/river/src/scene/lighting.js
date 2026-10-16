export function createLighting(T,scene) {
  scene.background=new T.Color(0x76919c);scene.fog=new T.Fog(0x76919c,190,440);
  scene.add(new T.HemisphereLight(0xd6edf1,0x425640,.65));
  const sun=new T.DirectionalLight(0xffe7c4,1);sun.position.set(-45,90,-65);sun.castShadow=true;
  sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-130,right:130,top:110,bottom:-110,near:1,far:300});
  sun.shadow.bias=-.0005;scene.add(sun);scene.add(sun.target);
}
