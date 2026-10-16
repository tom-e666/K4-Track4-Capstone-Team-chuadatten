export class IMUSensor {
  sample({state:s}) {return {pitch:s.pitch,roll:s.roll,yaw:s.yaw,verticalVelocity:s.vy,
    angularVelocity:{pitch:s.pitchRate,roll:s.rollRate,yaw:s.yawRate}};}
}
