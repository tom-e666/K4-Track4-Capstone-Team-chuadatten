// Public entry point shared by browser, Node tests, and teammate adapters.
export { Engine, normalizeCommand } from './src/vehicle/engine.js';
export { DT, VEHICLE, THRUSTER_KEYS } from './src/core/constants.js';
export { DEFAULTS } from './src/core/config.js';
export { mockCommand, MockController } from './src/vehicle/vehicleController.js';
export { SensorSuite } from './src/sensors/sensorSuite.js';
export { MockTerrain } from './src/scene/road.js';
export { StabilizerController } from './src/guidance/stabilizerController.js';
export { ThrusterSystem } from './src/vehicle/thrusterSystem.js';
export { AttitudeThrusters, ATTITUDE_KEYS } from './src/vehicle/attitudeThrusters.js';
export { SCENARIOS } from './src/core/scenarios.js';
