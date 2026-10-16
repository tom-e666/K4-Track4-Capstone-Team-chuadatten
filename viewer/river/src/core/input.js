// UI input emits intentions. It never writes directly to vehicle transforms.
export function createInput(onAction){
  const keys=new Set();let driving=false;
  const actions={KeyR:'reset',KeyE:'toggleAuto',Space:'rescue',KeyH:'hover',KeyL:'land',KeyP:'pause'};
  function editable(target){return /INPUT|TEXTAREA|SELECT/.test(target?.tagName??'');}
  window.addEventListener('keydown',event=>{
    if(editable(event.target)||(event.target?.tagName==='BUTTON'&&event.code==='Space'))return;
    if(['KeyW','KeyA','KeyS','KeyD'].includes(event.code)){event.preventDefault();keys.add(event.code);if(!driving){driving=true;onAction('drive');}}
    else if(actions[event.code]&&!event.repeat){event.preventDefault();onAction(actions[event.code]);}
  });
  window.addEventListener('keyup',event=>keys.delete(event.code));
  window.addEventListener('blur',()=>keys.clear());
  return {reset(){keys.clear();driving=false;},override(command){
    if(!driving)return command;
    const next={...command,throttle:keys.has('KeyW')?1:0,brake:keys.has('KeyS')?1:0,steer:(keys.has('KeyD')?1:0)-(keys.has('KeyA')?1:0)};
    if(command.flight&&command.acceleration){next.acceleration={...command.acceleration};
      if(keys.has('KeyW')||keys.has('KeyS'))next.acceleration.x=keys.has('KeyW')?4:-7;
      if(keys.has('KeyA')||keys.has('KeyD'))next.acceleration.z=next.steer*5;
    }
    return next;
  }};
}
