export class SpeedSensor {
  sample({state:s}) {return {forwardSpeed:s.vx*Math.cos(s.yaw)+s.vz*Math.sin(s.yaw),grounded:s.grounded,
    wheelContactState:{frontLeft:s.grounded,frontRight:s.grounded,rearLeft:s.grounded,rearRight:s.grounded}};}
}
