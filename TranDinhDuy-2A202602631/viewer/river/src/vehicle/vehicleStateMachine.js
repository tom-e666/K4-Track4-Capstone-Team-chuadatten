export class VehicleStateMachine {
  reset(){this.activeSince=null;this.stableSince=null;}
  constructor(){this.reset();}
  update(s,sensors,c) {
    if(s.terminal){s.mode=s.outcome==='landed'?'LANDED':s.outcome==='stopped'?'IDLE':'FAILURE';return;}
    if(s.flightActive){
      this.activeSince??=s.t;
      const stable=Math.abs(s.pitch)<.12&&Math.abs(s.roll)<.12&&Math.abs(s.yawRate)<.4&&s.vy>-.7;
      this.stableSince=stable?(this.stableSince??s.t):null;
      if(s.t-this.activeSince<.2)s.mode='RESCUE_ACTIVE';
      else if(s.t-this.activeSince<.55 && !stable)s.mode='RCS_STABILIZE';
      else if(c.landingStyle==='hover'&&s.hoverSince!==null&&s.t-s.hoverSince<1.5)s.mode='HOLD_BEFORE_LANDING';
      else if(s.forceLanding||(s.x>c.riverWidth+3&&!(c.landingStyle==='hover'&&s.hoverSince===null)))s.mode='LANDING';
      else if(s.hoverMode||!stable||s.y<1||this.stableSince===null||s.t-this.stableSince<.25)s.mode='HOVER_STABILIZE';
      else s.mode='MOVE_TO_SAFE_ZONE';
    }else if(!s.grounded)s.mode=s.accidentOccurred?'SPINNING':'FALLING';
    else if(s.skidding)s.mode='SKIDDING';
    else if(s.driverReacted&&s.x>-35)s.mode='LATE_TURN';
    else if(s.rescueTriggered && s.x>c.riverWidth)s.mode='LANDING';
    else if(sensors.cliffRiskScore>.25)s.mode=sensors.edgeRiskScore>c.rescueThreshold*.85&&s.autoRescueEnabled&&c.flightEnabled?'RESCUE_ARMED':'WARNING';
    else s.mode='DRIVING';
  }
}
