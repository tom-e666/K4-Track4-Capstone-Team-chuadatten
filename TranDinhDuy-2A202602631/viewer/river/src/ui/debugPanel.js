import { $ } from './hud.js';
export function bindDebugPanel(onReset,onAction){
  for(const [id,output,format]of [['gravity','gravityValue',n=>`${n.toFixed(2)} m/s²`],['maxLift','maxLiftValue',n=>`${n} m/s²`],['rescueThreshold','thresholdValue',n=>`${Math.round(n*100)}%`]]){
    $(id).addEventListener('input',()=>$(output).textContent=format(+$(id).value));$(id).addEventListener('change',onReset);
  }
  $('autoRescue').addEventListener('change',()=>onAction('setAuto'));
  for(const [id,action]of [['activateRescue','rescue'],['hover','hover'],['land','land'],['forceFall','fall'],['teleportEdge','edge'],['debugReset','reset']])$(id).addEventListener('click',()=>onAction(action));
  return {read(){return {gravity:+$('gravity').value,maxLift:+$('maxLift').value,rescueThreshold:+$('rescueThreshold').value,autoRescueEnabled:$('autoRescue').checked};},
    toggles(){return {rays:$('showRays').checked,fall:$('showFall').checked,rescue:$('showRescue').checked};}};
}
