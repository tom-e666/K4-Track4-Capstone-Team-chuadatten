export class LandingZoneSensor {
  sample({state:s,landingZone:zone}) {
    const found=!!zone?.enabled;
    return {landingZoneFound:found,landingZonePosition:found?{...zone.position}:null,
      landingZoneNormal:found?{...zone.normal}:null,
      landingZoneDistance:found?Math.hypot(zone.position.x-s.x,zone.position.y-s.y,zone.position.z-s.z):null};
  }
}
