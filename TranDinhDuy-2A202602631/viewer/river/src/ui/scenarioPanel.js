import { SCENARIOS } from '../core/scenarios.js';
import { DEFAULTS } from '../core/config.js';
import { $ } from './hud.js';
const formats={driverDelay:n=>`${n.toFixed(2)} s`,grip:n=>`${Math.round(n*100)}%`,spinIntensity:n=>`${n.toFixed(1)} rad/s`,
  rescueDelay:n=>`${n.toFixed(2)} s`,landingOffset:n=>`${n>0?'+':''}${n} m`};
export function bindScenarioPanel(onReset){
  const markCustom=()=>{$('scenarioPreset').value='custom';};
  for(const [key,format]of Object.entries(formats)){
    $(key).addEventListener('input',()=>$(key+'Value').textContent=format(+$(key).value));
    $(key).addEventListener('change',()=>{markCustom();onReset();});
  }
  $('landingStyle').addEventListener('change',()=>{markCustom();onReset();});
  $('scenarioPreset').addEventListener('change',()=>{
    const preset=SCENARIOS[$('scenarioPreset').value];if(!preset)return;
    const values={...DEFAULTS,...preset};
    for(const key of ['riverWidth','initialSpeed','brakeDistance','maxLift','gravity','rescueThreshold',...Object.keys(formats),'landingStyle']){
      $(key).value=values[key];$(key).dispatchEvent(new Event('input'));
    }
    onReset();
  });
  return {markCustom,read(){return {...Object.fromEntries(Object.keys(formats).map(key=>[key,+$(key).value])),landingStyle:$('landingStyle').value};}};
}
