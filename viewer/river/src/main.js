import * as C from '../core.js';
import { MockController, SocketController } from '../controllers.js';
import { copy } from './core/utils.js';
import { createInput } from './core/input.js';
import { createLighting } from './scene/lighting.js';
import { createEnvironment } from './scene/environment.js';
import { createVehicleModel } from './vehicle/vehicleModel.js';
import { createPathRenderer } from './effects/pathRenderer.js';
import { createWarningEffects } from './effects/warningEffects.js';
import { createHUD, $ } from './ui/hud.js';
import { bindDebugPanel } from './ui/debugPanel.js';
import { bindScenarioPanel } from './ui/scenarioPanel.js';

const T=window.THREE;
const showError=message=>{$('error').textContent=message;$('error').hidden=false;};
function boot(){
  if(!T)throw new Error('Không tải được thư viện 3D.');
  const renderer=new T.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));
  renderer.shadowMap.enabled=true;renderer.shadowMap.type=T.PCFSoftShadowMap;renderer.outputEncoding=T.sRGBEncoding;
  renderer.toneMapping=T.ACESFilmicToneMapping;renderer.toneMappingExposure=.92;renderer.domElement.tabIndex=0;renderer.domElement.setAttribute('aria-label','Khung cảnh 3D tương tác');$('viewport').appendChild(renderer.domElement);
  const scene=new T.Scene(),camera=new T.PerspectiveCamera(42,1,.2,800);createLighting(T,scene);
  let environment=createEnvironment(T,scene);
  const paths=createPathRenderer(T,scene),warnings=createWarningEffects(T,scene),hud=createHUD();
  let vehicle=createVehicleModel(T);scene.add(vehicle.group);
  let dependencies={},engine=new C.Engine(),controller=new MockController(),mode='mock';
  let frames=[],history=[],events=[],current,playTime=0,accumulator=0,flightEnabled=true,waiting=false;
  let playing=!matchMedia('(prefers-reduced-motion: reduce)').matches,speed=1,last=performance.now();
  let cameraMode='follow',yaw=1.05,pitch=.85,distance=170;const cameraTarget=new T.Vector3(-95,0,0);
  const input=createInput(action),debug=bindDebugPanel(()=>reset(true),action),scenarioPanel=bindScenarioPanel(()=>reset(true));
  const config=()=>({riverWidth:+$('riverWidth').value,initialSpeed:+$('initialSpeed').value,brakeDistance:+$('brakeDistance').value,flightEnabled,...debug.read(),...scenarioPanel.read()});
  function snapshot(){return {...engine.observation(),command:copy(engine.command)};}
  function syncPlay(){$('play').textContent=playing?'Ⅱ':'▶';$('play').setAttribute('aria-label',playing?'Tạm dừng':'Chạy');}
  function status(text,error=false){$('connectionStatus').textContent=text;$('connectionDot').classList.toggle('error-state',error);}
  function labelMode(){
    $('controllerLabel').textContent=mode==='socket'?'ĐIỀU HƯỚNG WEBSOCKET':mode==='custom'?'ĐIỀU HƯỚNG TÙY CHỈNH':mode==='interactive'?'LÁI & CỨU HỘ THỦ CÔNG':'CỨU HỘ MOCK';
    $('navMode').textContent=mode==='mock'?'MOCK':mode==='interactive'?'PLAY':mode==='socket'?'LIVE':'CUSTOM';
  }
  function reset(start=false){
    engine=new C.Engine(config(),dependencies);controller.reset(engine.observation());frames=[];history=[];events=[];
    accumulator=0;playTime=0;waiting=false;input.reset();$('result').hidden=true;$('error').hidden=true;
    environment.build(engine.config);
    $('widthValue').textContent=`${engine.config.riverWidth} m`;$('speedValue').textContent=`${Math.round(engine.config.initialSpeed*3.6)} km/h`;$('brakeValue').textContent=`${engine.config.brakeDistance} m`;
    if(mode==='interactive')mode='mock';
    if(mode==='mock'){
      frames.push(snapshot());while(!engine.state.terminal){engine.step(controller.update(engine.observation()));frames.push(snapshot());}
      events=copy(engine.events);current=frames[0];$('timeline').max=engine.state.t;$('timeline').disabled=false;
    }else{current=snapshot();history=[current];$('timeline').max=40;$('timeline').disabled=true;}
    cameraTarget.set(cameraMode==='follow'?current.state.x:engine.config.riverWidth*.5-14,current.state.y,cameraMode==='follow'?current.state.z:0);
    if(start)playing=true;labelMode();syncPlay();
  }
  // Branch a prerecorded demo into live simulation at the frame the user is watching.
  function beginLive(){
    if(current.state.terminal)reset(true);
    if(mode==='mock'){
      const branch=copy(current),prior=frames.filter(f=>f.state.t<=branch.state.t);
      const priorEvents=events.filter(e=>e.t<=branch.state.t);
      engine=new C.Engine(config(),dependencies);engine.restore(branch);engine.events=copy(priorEvents);
      controller.reset(engine.observation());history=prior;mode='interactive';$('timeline').disabled=true;$('timeline').max=40;
      labelMode();status('Chế độ tương tác · điều khiển đang hoạt động');
    }
    playing=true;syncPlay();
  }
  function action(name){
    if(name==='reset'){reset(true);return;}
    if(name==='pause'){$('play').click();return;}
    beginLive();
    if(name==='rescue')engine.triggerRescue();
    if(name==='hover'){engine.triggerRescue();engine.state.hoverMode=!engine.state.hoverMode;engine.state.forceLanding=false;}
    if(name==='land'){engine.triggerRescue();engine.state.forceLanding=true;engine.state.hoverMode=false;}
    if(name==='toggleAuto'||name==='setAuto'){
      const enabled=name==='setAuto'?$('autoRescue').checked:!engine.state.autoRescueEnabled;
      engine.state.autoRescueEnabled=engine.config.autoRescueEnabled=enabled;$('autoRescue').checked=enabled;
    }
    if(name==='fall'||name==='edge'){
      engine.teleportToEdge(name==='fall');history=[];controller.reset(engine.observation());
    }
    engine.refresh();current=snapshot();events=copy(engine.events);
    if(mode!=='mock'&&!history.length)history.push(current);
  }
  function setController(next,kind='custom'){
    if(!next||typeof next.update!=='function'||typeof next.reset!=='function')throw new Error('Controller requires reset(observation) and update(observation)');
    controller.dispose?.();controller=next;mode=kind;reset(true);
  }
  $('connect').addEventListener('click',()=>{try{const adapter=new SocketController($('socketUrl').value,text=>status(text,/Mất|Không|không hợp lệ/.test(text)));adapter.connect();setController(adapter,'socket');}catch(e){showError(e.message);}});
  $('useMock').addEventListener('click',()=>{setController(new MockController(),'mock');status('Mock đang sẵn sàng');});
  for(const id of ['restart','resultReplay'])$(id).addEventListener('click',()=>reset(true));
  $('play').addEventListener('click',()=>{if(current.state.terminal){reset(true);return;}playing=!playing;accumulator=0;syncPlay();});
  $('playbackSpeed').addEventListener('click',()=>{const values=[.5,1,2];speed=values[(values.indexOf(speed)+1)%values.length];$('playbackSpeed').textContent=`${speed}×`;});
  $('timeline').addEventListener('input',()=>{if(mode!=='mock')return;playing=false;playTime=+$('timeline').value;current=frames[Math.min(frames.length-1,Math.round(playTime/C.DT))];if(cameraMode==='follow')cameraTarget.set(current.state.x,current.state.y,current.state.z);syncPlay();});
  for(const [id,output,format]of [['riverWidth','widthValue',n=>`${n} m`],['initialSpeed','speedValue',n=>`${Math.round(n*3.6)} km/h`],['brakeDistance','brakeValue',n=>`${n} m`]]){
    $(id).addEventListener('input',()=>$(output).textContent=format(+$(id).value));$(id).addEventListener('change',()=>{scenarioPanel.markCustom();reset(true);});
  }
  function setFlight(enabled){flightEnabled=enabled;for(const [id,on]of [['flightOn',enabled],['flightOff',!enabled]]){$(id).classList.toggle('selected',on);$(id).setAttribute('aria-pressed',on);}reset(true);}
  $('flightOn').addEventListener('click',()=>setFlight(true));$('flightOff').addEventListener('click',()=>setFlight(false));
  document.querySelectorAll('[data-camera]').forEach(btn=>btn.addEventListener('click',()=>{cameraMode=btn.dataset.camera;document.querySelectorAll('[data-camera]').forEach(b=>{b.classList.toggle('selected',b===btn);b.setAttribute('aria-pressed',b===btn);});}));
  let drag=null;renderer.domElement.addEventListener('pointerdown',e=>{renderer.domElement.focus({preventScroll:true});drag={x:e.clientX,y:e.clientY};renderer.domElement.setPointerCapture(e.pointerId);});
  renderer.domElement.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-drag.x)*.005;pitch=Math.min(1.4,Math.max(.15,pitch+(e.clientY-drag.y)*.005));drag={x:e.clientX,y:e.clientY};});
  for(const event of ['pointerup','pointercancel'])renderer.domElement.addEventListener(event,()=>drag=null);
  renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();distance=Math.min(240,Math.max(65,distance*Math.exp(e.deltaY*.001)));},{passive:false});
  function resize(){const rect=$('viewport').getBoundingClientRect();renderer.setSize(rect.width,rect.height);camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();}
  new ResizeObserver(resize).observe($('viewport'));resize();
  function visibleHistory(){return mode==='mock'?frames.slice(0,Math.floor(current.state.t/C.DT)+1):history;}
  function draw(dt){
    const o=current,s=o.state,visible=visibleHistory();
    vehicle.update(s,o.thrusters,dt,o.attitudeThrusters);environment.update(s.t);warnings.update(o.sensors);paths.update(o,visible,debug.toggles());
    const follow=cameraMode==='follow',target=new T.Vector3(follow?s.x:engine.config.riverWidth*.5-14,follow?s.y:0,follow?s.z:0);
    const departureBlend=s.airborneSince===null?0:Math.min(1,(s.t-s.airborneSince)/1.2);
    cameraTarget.lerp(target,Math.min(1,dt*4+.02));const d=follow?40*distance/170:cameraMode==='top'?132:distance,p=cameraMode==='top'?1.49:follow?Math.max(.2,Math.min(1.2,.48+(pitch-.85)*.7)):pitch,a=follow?2.45-departureBlend*1.7+(yaw-1.05):yaw;
    camera.position.set(cameraTarget.x+d*Math.cos(p)*Math.cos(a),cameraTarget.y+d*Math.sin(p),cameraTarget.z+d*Math.cos(p)*Math.sin(a));camera.lookAt(cameraTarget.x+(follow?5:0),cameraTarget.y+1,cameraTarget.z);
    const zone=o.map.landing,point=zone?new T.Vector3(zone.x,3,zone.z).project(camera):null,rect=$('viewport').getBoundingClientRect();
    $('worldLabel').hidden=!point||point.z>1||follow||innerWidth<=760;
    if(point){$('worldLabel').style.left=`${rect.left+(point.x*.5+.5)*rect.width}px`;$('worldLabel').style.top=`${rect.top+(-point.y*.5+.5)*rect.height}px`;}
    hud.update(o,visible,mode==='mock'?frames.at(-1).state.t:40,mode);renderer.render(scene,camera);
  }
  function animate(now){
    const dt=Math.max(0,Math.min(.08,(now-last)/1000));last=now;
    if(playing){
      if(mode==='mock'){
        playTime=Math.min(frames.at(-1).state.t,playTime+dt*speed);current=frames[Math.min(frames.length-1,Math.floor(playTime/C.DT))];
        if(playTime>=frames.at(-1).state.t){current=frames.at(-1);playing=false;syncPlay();}
      }else if(!engine.state.terminal){
        accumulator=Math.min(.16,accumulator+dt*speed);
        try{
          let command=controller.update(engine.observation());waiting=!command;
          if(!command)accumulator=0;
          while(command&&accumulator>=C.DT&&!engine.state.terminal){
            engine.step(mode==='interactive'?input.override(command):command);current=snapshot();history.push(current);accumulator-=C.DT;
            command=controller.update(engine.observation());if(!command){accumulator=0;break;}
          }
          events=copy(engine.events);if(engine.state.terminal){playing=false;syncPlay();}
        }catch(error){playing=false;syncPlay();showError(`Bộ điều hướng lỗi: ${error.message}`);status('Lỗi điều hướng · đã dừng',true);}
      }
    }
    draw(dt);requestAnimationFrame(animate);
  }
  function exportRun(){const visible=events.filter(e=>e.t<=current.state.t);return {schema:'river-run/2',controller:controller.name??mode,mode,config:{...engine.config},outcome:current.state.outcome,
    landingSummary:{style:engine.config.landingStyle,touchdown:copy(visible.find(e=>e.type==='touchdown')??null),
      holdStartedAt:visible.find(e=>e.type==='hover_before_landing')?.t??null},
    completed:current.state.terminal,dt:C.DT,events:visible,frames:copy(visibleHistory())};}
  $('export').addEventListener('click',()=>{const url=URL.createObjectURL(new Blob([JSON.stringify(exportRun(),null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download=`cliff-rescue-${Date.now()}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
  window.RiverCore=C;window.RiverControllers={MockController,SocketController};
  window.RiverDemo={setController:next=>{setController(next);status('Bộ điều hướng tùy chỉnh đã nối');},useMock:()=>$('useMock').click(),
    restart:()=>reset(true),exportRun,getObservation:()=>copy(current),getStatus:()=>({mode,playing,waiting,state:copy(current.state)}),
    activateRescue:()=>action('rescue'),forceFall:()=>action('fall'),teleportToEdge:()=>action('edge'),
    setSensors:overrides=>{dependencies={...dependencies,sensors:overrides};reset(true);},
    setLandingPolicy:policy=>{if(typeof policy?.evaluate!=='function')throw new Error('Landing policy requires evaluate(state,zone)');dependencies={...dependencies,landingPolicy:policy};reset(true);},
    setLandingZoneProvider:provider=>{if(typeof provider!=='function')throw new Error('Landing zone provider must be a function');dependencies={...dependencies,landingZoneProvider:provider};reset(true);},
    setGuidance:guidance=>{if(typeof guidance?.generate!=='function')throw new Error('Guidance requires generate(context)');dependencies={...dependencies,guidance};reset(true);},
    setPredictor:predictor=>{if(typeof predictor?.predict!=='function')throw new Error('Predictor requires predict(context)');dependencies={...dependencies,predictor};reset(true);},
    setTerrain:terrain=>{if(typeof terrain?.surfaceAt!=='function'||typeof terrain?.supportAt!=='function'||typeof terrain?.edgeDistance!=='function')throw new Error('Terrain requires surfaceAt, supportAt, edgeDistance');dependencies={...dependencies,terrain};reset(true);},
    setVehicleModel:factory=>{const next=factory(T);if(!next?.group||typeof next.update!=='function')throw new Error('Vehicle model requires group and update(state,thrusters,dt)');scene.remove(vehicle.group);vehicle.dispose?.();vehicle=next;scene.add(vehicle.group);},
    setEnvironmentFactory:factory=>{const next=factory(T,scene);if(!next?.world||typeof next.build!=='function'||typeof next.update!=='function')throw new Error('Environment requires world, build(config), update(time)');scene.remove(environment.world);environment.dispose?.();environment=next;scene.add(environment.world);environment.build(engine.config);},
    inspectVehicle:()=>({name:vehicle.group.name,thrusters:vehicle.group.children.filter(child=>child.name.endsWith('Thruster')).map(child=>({name:child.name,x:child.position.x,y:child.position.y,z:child.position.z})),
      attitudeJets:vehicle.group.children.filter(child=>child.name.endsWith('Jet')).map(child=>child.name)})};
  reset();requestAnimationFrame(animate);
}
try{boot();}catch(error){showError(`Không khởi động được mô phỏng: ${error.message}`);console.error(error);}
