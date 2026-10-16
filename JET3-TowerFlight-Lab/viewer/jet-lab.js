/* Offline scatter plot + measured scenario replay. No network requests. */
(function(root){
  "use strict";
  var J=root.JetSim;
  var OUT={safe:"an toàn",near:"suýt va",crash:"va chạm",infeasible:"không khả thi",incomplete:"chưa hoàn tất"};
  var REASON={completed:"hoàn tất",tight_gate:"qua khe hẹp",ceiling:"chạm trần",hard_landing:"tiếp đất mạnh",ground_contact:"rơi xuống đất",route_incomplete:"chưa đến đích",insufficient_hover_reserve:"thiếu lực nâng",wall_tower_1:"va tháp 1",wall_tower_2:"va tháp 2",wall_tower_3:"va tháp 3"};
  function html(){return '<div class="eyebrow">LAB PHẢN LỰC · PD + TRẠNG THÁI THẬT</div>'+
    '<p class="note" id="jetSource"></p><details><summary>Nạp viewer.json khác</summary><label class="drop"><input id="jetFile" type="file" accept=".json,application/json">Chọn dữ liệu do script sinh</label></details>'+
    '<div class="jet-limits"><label for="jetLift">Lực nâng tối đa · 4 vòi <output id="jetLiftValue" for="jetLift">36 kN</output></label><input id="jetLift" type="range" min="20" max="56" step="1" value="36" aria-label="Lực nâng phản lực tối đa">'+
    '<label for="jetSpeed">Tốc độ tối đa đặt <output id="jetSpeedValue" for="jetSpeed">80 km/h</output></label><input id="jetSpeed" type="range" min="18" max="144" step="1" value="80" aria-label="Tốc độ tối đa của xe">'+
    '<p class="note" id="jetLimitStatus">Thả thanh trượt để chạy lại ca đang chọn. Hỏng vòi sẽ làm giảm lực nâng khả dụng.</p><div class="jet-limit-actions"><button type="button" id="jetResetLimits">Giới hạn gốc</button><button type="button" id="jetExportPolicy">Xuất policy.json</button></div></div>'+
    '<div class="plotbox"><div class="plothead"><select id="jetX" aria-label="Trục ngang lab phản lực"></select><span>×</span><select id="jetY" aria-label="Trục dọc lab phản lực"></select></div><canvas id="jetPlot" width="640" height="440" class="scenario-plot" aria-label="Các kịch bản xe phản lực đã chạy thử"></canvas></div>'+
    '<div class="legend"><span><i style="background:var(--red)"></i>va chạm</span><span><i style="background:var(--gold)"></i>suýt va</span><span><i style="background:var(--grey)"></i>an toàn</span><span><i style="background:var(--purple)"></i>chưa hoàn tất</span></div>'+
    '<p class="note">Chấm đậm dần theo vòng tìm kiếm. Chấm rỗng: không đủ lực nâng. Bấm chấm để phát lại.</p><div class="runs" id="jetCases"></div><div class="read" id="jetRead"></div><p class="note">CEM tìm lỗi có chủ đích; tỷ lệ chấm đỏ không phải xác suất tai nạn. Mô hình 3D + yaw, không có nhận thức cảm biến.</p>';}
  function validate(d){
    if(!d || d.format!=="jet-stunt-viewer/1" || d.modelVersion!==J.VERSION)throw new Error("Sai định dạng hoặc phiên bản mô hình phản lực");
    var c=d.cloud;if(!c || !Array.isArray(c.names) || !c.names.length || !Array.isArray(c.rows) || !c.rows.length || c.rows.length>100000 || !Array.isArray(d.runs))throw new Error("Thiếu cloud/runs hợp lệ");
    if(new Set(c.names).size!==c.names.length || c.names.some(function(n){return !J.SPACE[n];}))throw new Error("Tên tham số không hợp lệ");
    ["out","it","mode","hash"].forEach(function(k){if(!Array.isArray(c[k]) || c[k].length!==c.rows.length)throw new Error("Độ dài cloud."+k+" không khớp");});
    c.rows.forEach(function(row,i){if(!Array.isArray(row) || row.length!==c.names.length || !OUT[c.out[i]] || !Number.isInteger(c.it[i]) || c.it[i]<0 || typeof c.hash[i]!=="string")throw new Error("Dòng dữ liệu không hợp lệ: "+i);
      var p={};c.names.forEach(function(n,j){p[n]=row[j];});J.params(p);});
    J.policy(d.policy);d.runs.forEach(function(r){J.params(r.params);if(!OUT[r.outcome] || !Number.isInteger(r.cloudIndex) || r.cloudIndex<0 || r.cloudIndex>=c.rows.length)throw new Error("Kịch bản lưu không hợp lệ");});return d;
  }
  function create(play){
    var data=null,source="",selected=-1,xi=1,yi=6,map=null,lastRun=null,verified=false,tuningPolicy=J.policy({});
    function el(id){return document.getElementById(id);}
    function paramsAt(i){var p={};data.cloud.names.forEach(function(n,j){p[n]=data.cloud.rows[i][j];});return p;}
    function syncLimits(){
      el("jetLift").value=tuningPolicy.maxPodForce*4/1000;el("jetSpeed").value=tuningPolicy.maxSpeed*3.6;
      el("jetLiftValue").textContent=(tuningPolicy.maxPodForce*4/1000).toFixed(1).replace(".0","")+" kN";
      el("jetSpeedValue").textContent=(tuningPolicy.maxSpeed*3.6).toFixed(0)+" km/h";
    }
    function applyLimits(){
      try{
        tuningPolicy=J.policy(Object.assign({},tuningPolicy,{maxPodForce:+el("jetLift").value*1000/4,maxSpeed:+el("jetSpeed").value/3.6}));
        lastRun=J.rollout(selected>=0?paramsAt(selected):{},tuningPolicy);verified=false;syncLimits();renderRead();play(lastRun);
        el("jetLimitStatus").textContent="Đã chạy lại ca với giới hạn mới. Biểu đồ vẫn dùng policy gốc; xuất policy để script sinh lại toàn bộ.";
      }catch(e){el("jetLimitStatus").textContent=e.message;}
    }
    function choose(i){
      selected=i;var c=data.cloud,info=el("jetRead");
      try{
        var result=J.rollout(paramsAt(i),data.policy);
        if(result.hash!==c.hash[i] || result.metrics.outcome!==c.out[i])throw new Error("Kết quả phát lại không khớp file. Hãy chạy lại script với mô hình hiện tại.");
        lastRun=result;verified=true;tuningPolicy=J.policy(data.policy);syncLimits();play(result);renderRead();renderCases();plot();
        el("jetLimitStatus").textContent="Đang dùng giới hạn gốc của điểm đã chọn. Thả thanh trượt để chạy lại ca này.";
      }catch(e){info.textContent=e.message;}
    }
    function renderRead(){
      var box=el("jetRead");if(!box)return;box.replaceChildren();if(!lastRun){box.textContent="Chọn chấm hoặc kịch bản lưu để xem kết quả.";return;}
      var m=lastRun.metrics;
      var rows=[["kết quả",OUT[m.outcome]+" · "+(REASON[m.reason]||m.reason)]];
      if(m.feasible)rows=rows.concat([["kính đã vượt",m.gateCount+"/4"],["biên cửa nhỏ nhất",m.minClearance.toFixed(3)+" m"],["sai số bám lớn nhất",m.peakTrackingError.toFixed(3)+" m"],["tốc độ cao nhất",(m.peakSpeed*3.6).toFixed(1)+" km/h"],["vượt tốc độ đặt",(m.peakOverspeed*3.6).toFixed(1)+" km/h"],["bão hòa phản lực",(100*m.saturationFraction).toFixed(1)+"%"]]);
      else rows.push(["phát lại","Không đủ lực nâng để bắt đầu"]);
      rows.push(verified?["Node ↔ trình duyệt","khớp mã quỹ đạo "+lastRun.hash]:["giới hạn tùy chỉnh","đã tính lại · mã "+lastRun.hash]);
      rows.forEach(function(row){var k=document.createElement("span"),v=document.createElement("b");k.textContent=row[0];v.textContent=row[1];box.append(k,v);});
    }
    function renderCases(){var box=el("jetCases");box.replaceChildren();data.runs.forEach(function(r){var button=document.createElement("button"),dot=document.createElement("i");button.type="button";button.setAttribute("aria-pressed",String(r.cloudIndex===selected));dot.style.background="var(--"+({crash:"red",near:"gold",safe:"grey",incomplete:"purple"}[r.outcome]||"grey")+")";button.append(dot,document.createTextNode(r.id+" · "+(REASON[r.label]||r.label)));button.addEventListener("click",function(){choose(r.cloudIndex);});box.appendChild(button);});}
    function render(){
      if(!el("jetSource"))return;if(!data){el("jetSource").textContent="Chưa có dữ liệu. Chạy scripts/generate-jet-scenarios.cjs rồi nạp viewer.json.";return;}
      var c=data.cloud,counts={};c.out.forEach(function(o){counts[o]=(counts[o]||0)+1;});
      el("jetSource").textContent=source+" · "+c.rows.length.toLocaleString("vi-VN")+" lần thử · "+data.runs.length+" ca lưu · "+Object.keys(counts).map(function(k){return counts[k]+" "+OUT[k];}).join(", ");
      [el("jetX"),el("jetY")].forEach(function(select,k){select.replaceChildren();c.names.forEach(function(n,i){var option=document.createElement("option");option.value=i;option.textContent=n;select.appendChild(option);});select.value=k===0?xi:yi;});syncLimits();renderCases();renderRead();plot();
    }
    function plot(){
      var cv=el("jetPlot");if(!cv || !data)return;var c=data.cloud,ctx=cv.getContext("2d"),W=cv.width,H=cv.height,m=36,cs=getComputedStyle(document.documentElement);
      var color=function(k){return cs.getPropertyValue("--"+k).trim();},xb=J.SPACE[c.names[xi]],yb=J.SPACE[c.names[yi]];
      var sx=function(v){return m+(v-xb[0])/(xb[1]-xb[0])*(W-m-12);},sy=function(v){return H-m-(v-yb[0])/(yb[1]-yb[0])*(H-m-12);};map={sx:sx,sy:sy};
      ctx.clearRect(0,0,W,H);ctx.strokeStyle=color("line");ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(m,8);ctx.lineTo(m,H-m);ctx.lineTo(W-8,H-m);ctx.stroke();
      ctx.fillStyle=color("ink-3");ctx.font="600 16px "+cs.getPropertyValue("--f-mono");ctx.textAlign="left";ctx.fillText(xb[0].toFixed(1),m,H-10);ctx.textAlign="right";ctx.fillText(xb[1].toFixed(1)+" "+c.names[xi],W-8,H-10);
      ctx.save();ctx.translate(20,H-m);ctx.rotate(-Math.PI/2);ctx.textAlign="left";ctx.fillText(yb[0].toFixed(2),0,0);ctx.textAlign="right";ctx.fillText(c.names[yi]+" "+yb[1].toFixed(2),H-m-8,0);ctx.restore();
      var maxRound=Math.max(1,Math.max.apply(null,c.it)),rank={safe:0,infeasible:0,incomplete:1,near:2,crash:3};
      c.rows.map(function(_,i){return i;}).sort(function(a,b){return rank[c.out[a]]-rank[c.out[b]];}).forEach(function(i){
        var o=c.out[i];ctx.globalAlpha=o==="crash"?0.78:o==="near"?0.9:0.12+0.45*c.it[i]/maxRound;ctx.fillStyle=color(({crash:"red",near:"gold",incomplete:"purple"}[o]||"grey"));
        ctx.beginPath();ctx.arc(sx(c.rows[i][xi]),sy(c.rows[i][yi]),o==="crash"?4:o==="near"?3.5:2.3,0,Math.PI*2);
        if(o==="infeasible"){ctx.strokeStyle=color("ink-3");ctx.stroke();}else ctx.fill();
      });ctx.globalAlpha=1;
      if(selected>=0){ctx.strokeStyle=color("teal");ctx.lineWidth=3;ctx.beginPath();ctx.arc(sx(c.rows[selected][xi]),sy(c.rows[selected][yi]),10,0,Math.PI*2);ctx.stroke();}
    }
    function bind(){
      if(!data&&root.JET_LAB_SAMPLE){try{data=validate(root.JET_LAB_SAMPLE);source="Kết quả script · seed "+data.seed;xi=Math.max(0,data.cloud.names.indexOf(data.cloud.x));yi=Math.max(0,data.cloud.names.indexOf(data.cloud.y));}catch(e){el("jetSource").textContent=e.message;}}
      el("jetFile").addEventListener("change",function(e){var file=e.target.files[0];if(!file)return;var reader=new FileReader();reader.onload=function(){try{data=validate(JSON.parse(reader.result));source=file.name;selected=-1;lastRun=null;tuningPolicy=J.policy(data.policy);xi=Math.max(0,data.cloud.names.indexOf(data.cloud.x));yi=Math.max(0,data.cloud.names.indexOf(data.cloud.y));render();}catch(err){el("jetSource").textContent=err.message;}};reader.readAsText(file);});
      ["jetLift","jetSpeed"].forEach(function(id){el(id).addEventListener("input",function(){el("jetLiftValue").textContent=el("jetLift").value+" kN";el("jetSpeedValue").textContent=el("jetSpeed").value+" km/h";});el(id).addEventListener("change",applyLimits);});
      el("jetResetLimits").addEventListener("click",function(){if(selected>=0)choose(selected);else{tuningPolicy=J.policy(data?data.policy:{});syncLimits();lastRun=J.rollout({},tuningPolicy);verified=false;renderRead();play(lastRun);el("jetLimitStatus").textContent="Đã khôi phục giới hạn gốc.";}});
      el("jetExportPolicy").addEventListener("click",function(){var url=URL.createObjectURL(new Blob([JSON.stringify(tuningPolicy,null,2)+"\n"],{type:"application/json"})),a=document.createElement("a");a.href=url;a.download="jet-policy.json";a.click();setTimeout(function(){URL.revokeObjectURL(url);},1000);});
      el("jetX").addEventListener("change",function(){xi=+this.value;plot();});el("jetY").addEventListener("change",function(){yi=+this.value;plot();});
      el("jetPlot").addEventListener("click",function(e){if(!map||!data)return;var cv=e.currentTarget,b=cv.getBoundingClientRect(),x=(e.clientX-b.left)*cv.width/b.width,y=(e.clientY-b.top)*cv.height/b.height,best=-1,dist=196;
        data.cloud.rows.forEach(function(r,i){var d=Math.pow(map.sx(r[xi])-x,2)+Math.pow(map.sy(r[yi])-y,2);if(d<dist){best=i;dist=d;}});if(best>=0)choose(best);});render();
    }
    return{bind:bind,plot:plot};
  }
  root.JetLabUI={html:html,create:create,validate:validate,reasonLabel:function(reason){return REASON[reason]||reason;}};
})(window);
