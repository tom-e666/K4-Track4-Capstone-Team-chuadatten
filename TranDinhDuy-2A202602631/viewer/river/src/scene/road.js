// MOCK terrain query. Both rendering and sensors use this contract, not mesh names.
// Replace with raycasts/heightfields from your engine without changing the sensors.
export class MockTerrain {
  surfaceAt(x, z, config) {
    if (Math.abs(z) > 100 || x < -190 || x > config.riverWidth + 190) return null;
    if (x > 0 && x < config.riverWidth) return { y: config.waterY, normal: {x:0,y:1,z:0}, kind:'water', road:false };
    const road = x <= 0 ? (x <= -6.5 && Math.abs(z) <= 5.5) || (Math.abs(x+12)<=5.5 && Math.abs(z)<=63)
      : x >= config.riverWidth + 5 && Math.abs(z)<=5.5;
    return { y:0, normal:{x:0,y:1,z:0}, kind:'ground', road };
  }
  supportAt(state, config) {
    // Conservative footprint: start falling when the front loses bank support.
    if (state.x > -2.3 && state.x < config.riverWidth + 2.3) return false;
    return this.surfaceAt(state.x, state.z, config)?.kind === 'ground';
  }
  edgeDistance(state, config) {
    if (state.x <= 0) return -state.x - 2.3;
    if (state.x < config.riverWidth) return -Math.min(state.x, config.riverWidth-state.x);
    return state.x-config.riverWidth-2.3;
  }
}
