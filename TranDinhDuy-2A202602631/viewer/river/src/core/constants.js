export const DT = 0.02;
export const THRUSTER_KEYS = Object.freeze(['frontLeft', 'frontRight', 'rearLeft', 'rearRight']);
// SI units. +x = forward/east; +y = up; +z = right/south. Positive pitch lifts the nose.
export const VEHICLE = Object.freeze({ length: 4.6, width: 1.9, mass: 1600,
  leverX: 1.45, leverZ: 0.72, pitchInertiaPerMass: 2, rollInertiaPerMass: 0.55 });
