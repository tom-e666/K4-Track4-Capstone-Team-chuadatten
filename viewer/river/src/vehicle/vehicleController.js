import { StabilizerController } from '../guidance/stabilizerController.js';
import { worldGuidance } from '../guidance/horizontalGuidanceController.js';
import { attitudeCommand } from './attitudeThrusters.js';
import { clamp, wrapAngle } from '../core/utils.js';
const stabilizer=new StabilizerController();
export function mockCommand(o) {
  const s=o.state,c={gravity:o.vehicle.gravity,maxLift:o.vehicle.maxLift},target=o.guidance.targetLandingPoint;
  const activate=s.rescueTriggered || (s.autoRescueEnabled && s.airborneSince!==null && s.t-s.airborneSince>=o.scenario.rescueDelay);
  if(!o.vehicle.flightAvailable || !activate || !target){
    const reacted=s.driverAttentionSince!==null&&s.t-s.driverAttentionSince>=o.scenario.driverDelay;
    return {brake:reacted||o.metrics.distanceToEdge<=o.scenario.brakeDistance?1:0,
      steer:reacted&&s.grounded?clamp(wrapAngle(-Math.PI/2-s.yaw)*2.5,-1,1):0};
  }
  if(s.grounded && s.x>o.map.farBankX)return {brake:1,flight:false};
  const height=o.guidance.targetAltitude??o.scenario.hoverHeight;
  // Controllers consume the sensor frame, so replacing an IMU/altimeter changes feedback.
  const sensor=o.sensors,measured={...s,pitch:sensor.pitch,roll:sensor.roll,yaw:sensor.yaw,
    pitchRate:sensor.angularVelocity.pitch,rollRate:sensor.angularVelocity.roll,yawRate:sensor.angularVelocity.yaw};
  const measuredY=sensor.altitudeAboveGround===null?s.y:sensor.altitudeAboveGround+(sensor.surfaceBelow==='water'?o.map.waterY:0);
  const rcsFirst=s.rescueSince===null||s.t-s.rescueSince<.18;
  const lift=rcsFirst?0:stabilizer.altitudeController(measuredY,sensor.verticalVelocity,height,c);
  const stabilizing=s.y<1||Math.abs(s.pitch)>.45||Math.abs(s.roll)>.45;
  return {...worldGuidance(s,o.guidance.targetWaypoint??target,{hover:s.hoverMode,stabilizing}),flight:true,lift,
    thrusters:rcsFirst?{frontLeft:0,frontRight:0,rearLeft:0,rearRight:0}:stabilizer.mix(measured,lift,c),attitude:attitudeCommand(measured,o.guidance.targetYaw??0)};
}
export class MockController {
  name='Cứu hộ mock';
  reset(){}
  update(o){return mockCommand(o);}
}
