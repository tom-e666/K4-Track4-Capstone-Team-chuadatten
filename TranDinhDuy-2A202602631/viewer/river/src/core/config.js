import { clamp, finite } from './utils.js';
export const DEFAULTS = Object.freeze({ riverWidth: 24, initialSpeed: 22, brakeDistance: 9,
  maxBrake: 5.5, maxLift: 24, gravity: 9.81, flightEnabled: true,
  autoRescueEnabled: true, rescueThreshold: 0.92, waterY: -16, hoverHeight: 6,
  driverDelay: 1.05, grip: 0.45, spinIntensity: 1.8, rescueDelay: 0.55,
  landingStyle: 'direct', landingOffset: 0 });
export function makeConfig(input = {}) {
  const c = { ...DEFAULTS, ...input };
  for (const [key, low, high] of [['riverWidth',12,48],['initialSpeed',0,30],['brakeDistance',6,25],
    ['maxBrake',3,10],['maxLift',12,36],['gravity',3,15],['rescueThreshold',.4,.99],['waterY',-30,-8],['hoverHeight',3,12],
    ['driverDelay',0,1.8],['grip',.15,1],['spinIntensity',0,4],['rescueDelay',0,1.4],['landingOffset',-12,12]]) {
    c[key] = clamp(finite(c[key], key), low, high);
  }
  c.flightEnabled = c.flightEnabled !== false;
  c.autoRescueEnabled = c.autoRescueEnabled !== false;
  if(!['direct','arc','hover'].includes(c.landingStyle))throw new Error('Invalid landingStyle');
  return c;
}
