/* SKYFALL mode reuses the existing Lab 11 renderer, scene, camera, vehicle mesh,
   ground and surrounding city. Three tall towers and flight mechanics are layered on top.
   Open viewer/index.html?skyfall=1 */
(function (root) {
  "use strict";
  if (!new URLSearchParams(root.location.search).has("skyfall")) return;
  const b=root.__skyfallBridge, P=root.SkyfallPhysics, S=root.SkyfallScenarios;
  if (!b || !P || !S) { alert("SKYFALL dependencies unavailable."); return; }
  const T=root.THREE, C=()=>document.createElement("canvas");
  // Keep the existing city, lighting and ground as background; hide original traffic.
  b.actors.forEach(o=>o.visible=false);
  b.parked.forEach(o=>o.visible=false);
  b.ghost.visible=false;
  document.body.classList.add("skyfall-mode");
  document.title="SKYFALL | Three Tower Challenge";
  const style=document.createElement("style");
  style.textContent=`
    body.skyfall-mode .rail,body.skyfall-mode .bar,body.skyfall-mode .hint {display:none!important;}
    body.skyfall-mode #labels {display:none!important;}
    .skyfall-panel {position:absolute;top:16px;left:16px;width:338px;max-height:calc(100vh - 32px);
      z-index:4;overflow:auto;background:rgba(13,23,36,.91);color:#eff9ff;border:1px solid rgba(132,204,228,.26);
      box-shadow:0 18px 60px #0313268c;border-radius:16px;padding:18px 20px;font:13px/1.5 system-ui,sans-serif;}
    .skyfall-panel h1 {font-size:27px;letter-spacing:.13em;margin:0;font-weight:800;}
    .skyfall-panel .subtitle {font-size:11px;color:#9bc2d6;letter-spacing:.09em;margin-bottom:16px;}
    .skyfall-panel label {display:block;margin:12px 0 3px;color:#b8d0e2;font-size:12px;}
    .skyfall-panel select,.skyfall-panel input {box-sizing:border-box;width:100%;padding:8px;border:1px solid #3d6478;border-radius:8px;background:#14293b;color:#fff;}
    .skyfall-panel input[type=range] {padding:0;accent-color:#4cddb5;}
    .skyfall-panel .row {display:flex;gap:7px;margin-top:12px;}
    .skyfall-panel button {border:1px solid #3e7180;border-radius:8px;padding:9px 10px;background:#1a3948;color:#efffff;cursor:pointer;flex:1;font-weight:650;}
    .skyfall-panel button.primary {background:#1f907e;border-color:#40b9a5;color:white;}
    .skyfall-panel .kv {display:grid;grid-template-columns:1fr auto;gap:4px 8px;margin-top:14px;padding:12px;background:#081b2a9c;border-radius:10px;}
    .skyfall-panel .kv strong {font-variant-numeric:tabular-nums;}
    .skyfall-panel #sf-status {font-weight:800;color:#60dfb8;}
    .skyfall-panel .small {color:#acc1d0;font-size:11px;margin-top:10px;}
    .skyfall-panel #sf-benchmark {white-space:pre-line;line-height:1.6;color:#c5e9ef;margin-top:10px;}
    .skyfall-help {position:fixed;right:18px;bottom:16px;z-index:4;background:#0e243acb;
       color:#dfecf5;padding:9px 14px;border-radius:10px;font:12px system-ui,sans-serif;}
    @media(max-width:720px){.skyfall-panel{width:calc(100% - 32px);max-height:51vh}.skyfall-help{display:none}}
  `;
  document.head.appendChild(style);
  const panel=document.createElement("section");panel.className="skyfall-panel";
  panel.innerHTML=`
    <h1>SKYFALL</h1><div class="subtitle">THREE-TOWER · AUTONOMOUS RECOVERY LAB</div>
    <label for="sf-scenario">Tình huống</label>
    <select id="sf-scenario"><option value="jump">01 · Precision Tower Jump</option>
    <option value="recovery">02 · Mid-Air Disturbance</option></select>
    <label for="sf-mode">Bộ điều khiển</label>
    <select id="sf-mode"><option value="controlled">PD/PID · Thrusters ON</option>
    <option value="baseline">Baseline · Không điều khiển</option></select>
    <label for="sf-speed">Vận tốc phóng: <output id="sf-speed-val">26</output> m/s</label>
    <input type="range" id="sf-speed" min="20" max="32" step="1" value="26">
    <label for="sf-wind">Gia tốc nhiễu dọc: <output id="sf-wind-val">0</output> m/s²</label>
    <input type="range" id="sf-wind" min="-4" max="4" step=".5" value="0">
    <div class="row"><button class="primary" id="sf-restart">↺ Chạy lại</button>
    <button id="sf-pause">Tạm dừng</button><button id="sf-camera">Camera: Tổng</button></div>
    <div class="kv">
      <span>Trạng thái</span><strong id="sf-status">FLYING</strong>
      <span>Thời gian</span><strong id="sf-time">0.0 s</strong>
      <span>Vị trí (x, h)</span><strong id="sf-pos">0, 0 m</strong>
      <span>Vận tốc ngang</span><strong id="sf-vx">0 m/s</strong>
      <span>Góc pitch</span><strong id="sf-pitch">0°</strong>
      <span>Lực đẩy dọc</span><strong id="sf-thrust">0 m/s²</strong>
    </div>
    <div class="small" id="sf-reason">Bay qua hai khoảng trống và đáp trong tòa C.</div>
    <div class="row"><button id="sf-bench">So sánh 20 lượt</button>
    <button id="sf-download">Xuất kết quả CSV</button></div>
    <div id="sf-benchmark" aria-live="polite"></div>
    <div class="small">Mô hình giả định: xe có thruster, physics 2D, view 3D.
    Kết quả không chứng nhận an toàn phương tiện thật.</div>
  `;
  document.body.appendChild(panel);
  const help=document.createElement("div");help.className="skyfall-help";
  help.textContent="A → B → C  •  Cửa xanh = vùng xuyên qua  •  Kết cấu = va chạm";
  document.body.appendChild(help);
  const $=id=>document.getElementById(id);
  const geometry=new T.BoxGeometry(1,1,1);
  const towerMaterial=new T.MeshLambertMaterial({color:0x34495f});
  const sideMaterial=new T.MeshLambertMaterial({color:0x4d6680});
  const glass=new T.MeshBasicMaterial({color:0x70cdea,transparent:true,opacity:0.13,depthWrite:false,side:T.DoubleSide});
  const rim=new T.LineBasicMaterial({color:0x53f2cb,transparent:true,opacity:0.92});
  const deckMat=new T.MeshLambertMaterial({color:0x2a9687});
  function cuboid(parent,cx,cy,cz,w,h,d,mat) {
    const o=new T.Mesh(geometry,mat);o.position.set(cx,cy,cz);o.scale.set(w,h,d);
    o.castShadow=true;o.receiveShadow=true;parent.add(o);return o;
  }
  function outline(parent,x,y) {
    const pts=[new T.Vector3(x,y,-9),new T.Vector3(x,y,9),
      new T.Vector3(x,y+20,9),new T.Vector3(x,y+20,-9),new T.Vector3(x,y,-9)];
    parent.add(new T.Line(new T.BufferGeometry().setFromPoints(pts),rim));
  }
  function labelSprite(text,x,y) {
    const canvas=C();canvas.width=512;canvas.height=128;
    const ctx=canvas.getContext("2d");ctx.fillStyle="rgba(5,20,35,.85)";
    ctx.fillRect(0,0,512,128);ctx.strokeStyle="#4fdbc8";ctx.lineWidth=4;
    ctx.strokeRect(2,2,508,124);ctx.fillStyle="#edffff";ctx.font="bold 46px Arial";
    ctx.textAlign="center";ctx.fillText(text,256,77);
    const sprite=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas),transparent:true}));
    sprite.position.set(x,y,0);sprite.scale.set(33,8.25,1);b.scene.add(sprite);
  }
  // The original city/ground remains visible. Add ONLY three new high-rise towers.
  S.towers.forEach(t=>{
    const w=t.x1-t.x0,mid=(t.x0+t.x1)/2,g=new T.Group();b.scene.add(g);
    cuboid(g,mid,S.floor/2,0,w,S.floor,44,towerMaterial);
    cuboid(g,mid,(S.ceiling+t.height)/2,0,w,t.height-S.ceiling,44,towerMaterial);
    cuboid(g,mid,(S.floor+S.ceiling)/2,-(44+18)/4,w,20,(44-18)/2,sideMaterial);
    cuboid(g,mid,(S.floor+S.ceiling)/2, (44+18)/4,w,20,(44-18)/2,sideMaterial);
    [t.x0,t.x1].forEach(x=>{
      const pane=new T.Mesh(new T.PlaneGeometry(18,20),glass);
      pane.rotation.y=Math.PI/2;pane.position.set(x,S.floor+10,0);g.add(pane);
      outline(g,x,S.floor);
    });
    if(t.id==="C"){
      cuboid(g,S.goalX,S.floor+.035,0,14,.07,16,deckMat);
    }
    labelSprite(t.id+" · "+t.name,mid,t.height+8);
  });
  const car=b.carMesh("ego","car");b.scene.add(car);
  const trajectoryGeo=new T.BufferGeometry();
  const trailArray=new Float32Array(1500*3);
  trajectoryGeo.setAttribute("position",new T.BufferAttribute(trailArray,3));
  trajectoryGeo.setDrawRange(0,0);
  const trail=new T.Line(trajectoryGeo,new T.LineBasicMaterial({color:0xf4ae59}));
  trail.frustumCulled=false;b.scene.add(trail);
  const pathPts=[new T.Vector3(10,54,-1),new T.Vector3(60,54,-1),
    new T.Vector3(140,54,-1),new T.Vector3(S.goalX,44,-1)];
  const path=new T.Line(new T.BufferGeometry().setFromPoints(pathPts),
    new T.LineDashedMaterial({color:0x46d8bd,dashSize:4,gapSize:3,transparent:true,opacity:.45}));
  path.computeLineDistances();b.scene.add(path);
  let current,model,paused=false,acc=0,follow=false,csv="";
  function settings(){
    return S.make($("sf-scenario").value,{launchSpeed:Number($("sf-speed").value),
      wind:Number($("sf-wind").value)});
  }
  function restart(){
    model=settings();current=P.create(model);paused=false;acc=0;
    $("sf-pause").textContent="Tạm dừng";$("sf-benchmark").textContent="";
    csv="t,x,y,vx,vy,pitch,thrust,status\n";trail.geometry.setDrawRange(0,0);
    updateView();updateText();
  }
  function updateText(){
    $("sf-status").textContent=current.status.toUpperCase();
    $("sf-status").style.color=current.status==="crashed"?"#ff7468":"#60dfb8";
    $("sf-time").textContent=current.t.toFixed(2)+" s";
    $("sf-pos").textContent=current.x.toFixed(1)+", "+current.y.toFixed(1)+" m";
    $("sf-vx").textContent=current.vx.toFixed(2)+" m/s";
    $("sf-pitch").textContent=(current.theta*180/Math.PI).toFixed(1)+"°";
    $("sf-thrust").textContent=current.control.thrust.toFixed(1)+" m/s²";
    $("sf-reason").textContent=current.reason||
      (current.disturbed?"Nhiễu đã xảy ra · đang phục hồi tư thế":"Đang vượt các tòa tháp");
  }
  function updateView(){
    car.position.set(current.x,current.y,0);car.rotation.set(0,0,current.theta);
    // Fixed overview or following chase camera. Reuses existing perspective camera.
    if(follow) {
      b.camera.position.set(current.x-30,current.y+22,77);
      b.camera.lookAt(current.x+14,current.y,0);
    } else {
      b.camera.position.set(92,112,320);
      b.camera.lookAt(92,65,0);
    }
    const n=Math.min(500,current.trail.length);
    for(let i=0;i<n;i++){
      trailArray[i*3]=current.trail[i][0];
      trailArray[i*3+1]=current.trail[i][1]+1;
      trailArray[i*3+2]=1.2;
    }
    trajectoryGeo.setDrawRange(0,n);
    trajectoryGeo.attributes.position.needsUpdate=true;
  }
  $("sf-scenario").onchange=restart;$("sf-mode").onchange=restart;
  ["speed","wind"].forEach(key=>{
    $("sf-"+key).oninput=()=>{ $("sf-"+key+"-val").textContent=$("sf-"+key).value; };
    $("sf-"+key).onchange=restart;
  });
  $("sf-restart").onclick=restart;
  $("sf-pause").onclick=()=>{
    paused=!paused;$("sf-pause").textContent=paused?"Tiếp tục":"Tạm dừng";
  };
  $("sf-camera").onclick=()=>{
    follow=!follow;$("sf-camera").textContent=follow?"Camera: Theo xe":"Camera: Tổng";
  };
  $("sf-bench").onclick=()=>{
    const data=P.benchmark(settings(),20);
    $("sf-benchmark").textContent="Baseline: "+data.baselineSuccess+"/20 đáp thành công\n"+
      "Controlled: "+data.controlledSuccess+"/20 đáp thành công\n"+
      "20 phép thử / cấu hình, thay đổi vận tốc và góc phóng.";
  };
  $("sf-download").onclick=()=>{
    const blob=new Blob([csv],{type:"text/csv;charset=utf-8"});
    const url=URL.createObjectURL(blob),a=document.createElement("a");
    a.href=url;a.download="skyfall-trajectory.csv";a.click();
    setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  restart();
  root.Skyfall={
    active:true,
    tick:function(dt){
      if(!paused&&current.status==="flying"){
        acc+=Math.min(dt,.05);
        while(acc>=.02&&current.status==="flying"){
          P.step(current,model,$("sf-mode").value,.02);acc-=.02;
          if(Math.floor(current.t*10)%2===0){
            csv+=[current.t.toFixed(2),current.x.toFixed(3),current.y.toFixed(3),
              current.vx.toFixed(3),current.vy.toFixed(3),current.theta.toFixed(4),
              current.control.thrust.toFixed(3),current.status].join(",")+"\n";
          }
        }
      }
      updateView();updateText();
    }
  };
})(window);
