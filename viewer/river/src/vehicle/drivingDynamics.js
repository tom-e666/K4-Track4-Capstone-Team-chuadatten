import { clamp, wrapAngle } from '../core/utils.js';
// Bicycle-like steering with grip-limited velocity direction: body heading can outrun its tyres.
export function driveStep(s,u,c,dt){
  const speed=Math.max(0,s.speed+(3.8*u.throttle-c.maxBrake*u.brake-.035*s.speed)*dt);
  const requestedRate=u.steer*Math.min(1.4,speed/11);
  s.yawRate+=(requestedRate-s.yawRate)*(1-Math.exp(-dt/.22));s.yaw+=s.yawRate*dt;
  let velocityHeading=Math.atan2(s.vz,s.vx);
  const maxTurn=c.grip*c.gravity/Math.max(speed,2)*dt;
  velocityHeading+=clamp(wrapAngle(s.yaw-velocityHeading),-maxTurn,maxTurn);
  s.vx=Math.cos(velocityHeading)*speed;s.vz=Math.sin(velocityHeading)*speed;s.speed=speed;
  s.slipAngle=wrapAngle(s.yaw-velocityHeading);s.skidding=Math.abs(s.slipAngle)>.12&&speed>5;
  return s.skidding;
}
