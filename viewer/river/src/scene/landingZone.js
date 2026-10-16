export function createLandingZone(config) {
  return { id:'opposite-bank-pad', enabled:true, position:{x:config.riverWidth+22,y:0,z:config.landingOffset},
    normal:{x:0,y:1,z:0}, halfX:10, halfZ:5,
    heading:config.landingStyle==='arc'?-.3:config.landingStyle==='hover'?.25:0 };
}
// Replace this policy for another touchdown envelope or tagged landing surface.
export const defaultLandingPolicy = {
  evaluate(state, zone) {
    return !!zone?.enabled && Math.abs(state.x-zone.position.x)<=zone.halfX &&
      Math.abs(state.z-zone.position.z)<=zone.halfZ && Math.abs(state.pitch)<.22 && Math.abs(state.roll)<.22;
  }
};
