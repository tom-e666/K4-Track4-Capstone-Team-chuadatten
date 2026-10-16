import { THRUSTER_KEYS } from '../core/constants.js';
export const $=id=>document.getElementById(id);
const modes={
  IDLE:['✓','Xe đã dừng','Tốc độ và phản ứng hiện tại đủ để tránh văng khỏi bờ.'],DRIVING:['01','Tài xế chưa để ý','Xe giữ tốc độ khi tiến đến ngã ba sát sông.'],
  WARNING:['02','Ngã ba phía trước','Tài xế chưa phản ứng kịp với đoạn cua.'],RESCUE_ARMED:['03','Cứu hộ sẵn sàng','Dự đoán đường rơi và chờ xe mất hỗ trợ mặt đường.'],
  LATE_TURN:['02','Phanh và cua quá muộn','Đã đánh lái nhưng quán tính xe vẫn hướng ra bờ sông.'],
  SKIDDING:['03','Xe đang trượt ngang','Lốp mất bám; hướng thân xe lệch khỏi hướng chuyển động.'],
  SPINNING:['04','Văng khỏi đường · xe xoay','Thân xe xoay trên ba trục, đang chờ cứu hộ kích hoạt.'],
  RCS_STABILIZE:['05','Vòi quanh thân đang cân bằng','Các cặp vòi phun đối nhau để giảm tốc độ xoay.'],
  HOLD_BEFORE_LANDING:['07','Giữ cao độ trước khi đáp','Dừng ngang, giữ 1,5 giây rồi hạ nhẹ xuống khu đáp.'],
  FALLING:['!','Xe đang rơi','Nhấn Space để kích hoạt cứu hộ thủ công.'],RESCUE_ACTIVE:['04','Bật vòi quanh thân','Hãm xoay trước khi tăng lực nâng dưới gầm.'],
  HOVER_STABILIZE:['05','Nâng và ổn định thân xe','Vòi dưới gầm phối hợp vòi quanh thân để giữ xe cân bằng.'],
  MOVE_TO_SAFE_ZONE:['06','Di chuyển đến khu đáp','Giữ cao độ và điều hướng qua vực.'],LANDING:['07','Đang hạ cánh','Giảm tốc ngang, hạ độ cao và giảm lực đẩy.'],
  LANDED:['✓','Hạ cánh an toàn','Xe đã dừng bên trong khu đáp.'],FAILURE:['!','Cứu hộ chưa thành công','Thử lại hoặc chỉnh thông số trong bảng debug.']
};
export function createHUD(){
  const canvas=$('minimap'),ctx=canvas.getContext('2d');
  function metric(id,value,unit=''){ $(id).innerHTML=value+(unit?`<small> ${unit}</small>`:''); }
  function drawMap(o,history){
    const s=o.state,width=o.map.farBankX;
    const X=x=>24+(x+100)/(width+160)*472,Z=z=>150+z*2.1;
    ctx.fillStyle='#142a32';ctx.fillRect(0,0,520,300);ctx.fillStyle='#244b55';ctx.fillRect(X(0),0,X(width)-X(0),300);
    ctx.strokeStyle='#38505a';ctx.lineWidth=19;ctx.beginPath();ctx.moveTo(X(-100),Z(0));ctx.lineTo(X(-12),Z(0));ctx.moveTo(X(-12),Z(-66));ctx.lineTo(X(-12),Z(66));ctx.moveTo(X(width+4),Z(0));ctx.lineTo(X(width+60),Z(0));ctx.stroke();
    for(const [points,color,dashed]of [[o.prediction.predictedFallPath,'#ff9a4c',true],[o.guidance.rescuePath,'#4edbff',false]]){
      ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dashed?[5,5]:[]);ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(X(p.x),Z(p.z)):ctx.moveTo(X(p.x),Z(p.z)));ctx.stroke();ctx.setLineDash([]);
    }
    const zone=o.map.landing;
    if(zone){ctx.strokeStyle='#87e4c7';ctx.strokeRect(X(zone.x-zone.halfX),Z(zone.z-zone.halfZ),X(zone.x+zone.halfX)-X(zone.x-zone.halfX),zone.halfZ*4.2);ctx.fillStyle='#87e4c7';ctx.font='20px Segoe UI';ctx.fillText('KHU ĐÁP',X(zone.x-13),Z(zone.z-15));}
    ctx.fillStyle='#8fbdc2';ctx.save();ctx.translate(X(width/2),35);ctx.rotate(Math.PI/2);ctx.font='19px Segoe UI';ctx.fillText(`VỰC · ${width} m`,0,0);ctx.restore();
    ctx.strokeStyle='#87e4c7';ctx.beginPath();history.forEach((f,i)=>i?ctx.lineTo(X(f.state.x),Z(f.state.z)):ctx.moveTo(X(f.state.x),Z(f.state.z)));ctx.stroke();
    ctx.save();ctx.translate(X(s.x),Z(s.z));ctx.rotate(s.yaw);ctx.fillStyle=s.mode==='FAILURE'?'#ff9280':'#e9f3e9';ctx.beginPath();ctx.moveTo(10,0);ctx.lineTo(-7,-6);ctx.lineTo(-4,0);ctx.lineTo(-7,6);ctx.closePath();ctx.fill();ctx.restore();
  }
  return {update(o,history,duration,mode){
    const s=o.state,sensor=o.sensors,p=modes[s.mode]??modes.DRIVING;
    $('phaseIcon').textContent=p[0];$('phaseText').textContent=p[1];$('phaseDetail').textContent=p[2];
    metric('metricSpeed',Math.round(s.speed*3.6),'km/h');metric('metricAltitude',sensor.altitudeAboveGround?.toFixed(1)??'—','m');
    metric('metricEdge',s.x<0?Math.max(0,sensor.edgeDistance).toFixed(1):'—','m');metric('metricStop',o.metrics.stoppingDistance.toFixed(1),'m');
    metric('metricVertical',s.vy.toFixed(1),'m/s');metric('metricAttitude',`${(s.pitch*180/Math.PI).toFixed(0)}° / ${(s.roll*180/Math.PI).toFixed(0)}°`);
    metric('metricRisk',Math.round(sensor.cliffRiskScore*100),'%');metric('metricContact',s.grounded?'Mặt đất':'Trên không');
    $('metricRisk').style.color=sensor.cliffRiskScore>.5?'var(--gold)':'';
    $('destinationDistance').textContent=sensor.landingZoneFound?`${Math.round(sensor.landingZoneDistance)} m đến khu đáp`:'Không tìm thấy khu đáp';
    $('zoneStatus').textContent=sensor.landingZoneFound?'◎ Khu đáp hợp lệ':'! Không có khu đáp';
    const styleNames={direct:'Đáp thẳng',arc:'Tiếp cận vòng',hover:'Giữ cao độ rồi đáp'};
    const style=styleNames[o.scenario.landingStyle]??styleNames.direct;
    $('landingPlan').textContent=`${style} · khu đáp z = ${o.map.landing?.z??'—'} m`;
    const rcs=o.attitudeThrusters??{};
    $('rcsPower').textContent=`${Math.round(Math.max(...['yawPositive','yawNegative','pitchPositive','pitchNegative','rollPositive','rollNegative'].map(key=>rcs[key]??0))*100)}%`;
    $('rotationRate').textContent=`${Math.round(Math.hypot(s.yawRate,s.pitchRate,s.rollRate)*180/Math.PI)}°/s`;
    $('sensorRoad').textContent=sensor.roadAheadExists?'↗ Bề mặt phía trước liên tục':'⚠ Phát hiện bề mặt bị gián đoạn';
    for(const key of THRUSTER_KEYS){const power=s.terminal?0:o.thrusters[key];$(key+'Bar').style.width=`${power*100}%`;$(key+'Value').textContent=`${Math.round(power*100)}%`;}
    const stage=['DRIVING','WARNING','LATE_TURN'].includes(s.mode)?0:['SKIDDING','SPINNING','FALLING'].includes(s.mode)?1:['LANDING','LANDED','HOLD_BEFORE_LANDING'].includes(s.mode)?3:2;
    document.querySelectorAll('.steps li').forEach((el,i)=>el.classList.toggle('active',i<=stage));
    $('clock').innerHTML=`${s.t.toFixed(2).padStart(5,'0')} <span>/ ${duration.toFixed(2)} s</span>`;
    $('timeline').value=s.t;$('result').hidden=!s.terminal;
    if(s.terminal){const heading=Math.atan2(Math.sin(s.yaw),Math.cos(s.yaw))*180/Math.PI;$('result').classList.toggle('failure',s.mode==='FAILURE');$('resultTitle').textContent=s.outcome==='stopped'?'Xe dừng trước vực':p[1];$('resultDetail').textContent=`${style} · vị trí (${s.x.toFixed(1)}, ${s.z.toFixed(1)}) m · hướng ${heading.toFixed(0)}° · ${s.t.toFixed(2)} s. Kết quả: ${s.outcome}.`;}
    $('danger').hidden=(sensor.cliffRiskScore<=.25&&!s.accidentOccurred&&!s.skidding)||s.terminal||s.x>o.map.farBankX;
    $('danger').textContent=s.flightActive?'CÂN BẰNG QUANH THÂN + NÂNG DƯỚI GẦM':s.skidding&&s.grounded?'⚠ CUA MUỘN · XE MẤT BÁM':s.accidentOccurred?'⚠ XE VĂNG VÀ XOAY TRÊN KHÔNG':'⚠ TÀI XẾ CHƯA PHẢN ỨNG KỊP';
    drawMap(o,history);
  }};
}
