import { ForwardCliffSensor } from './cliffSensor.js';
import { AltitudeSensor } from './altitudeSensor.js';
import { IMUSensor } from './imuSensor.js';
import { SpeedSensor } from './speedSensor.js';
import { EdgeRiskSensor } from './edgeRiskSensor.js';
import { LandingZoneSensor } from './landingZoneSensor.js';
export class SensorSuite {
  constructor(overrides={}) {
    this.modules={cliff:new ForwardCliffSensor(),altitude:new AltitudeSensor(),imu:new IMUSensor(),
      speed:new SpeedSensor(),edge:new EdgeRiskSensor(),landing:new LandingZoneSensor(),...overrides};
    for(const [name,sensor] of Object.entries(this.modules)) if(typeof sensor.sample!=='function')throw new Error(`Sensor ${name} requires sample(context)`);
  }
  sample(context) {
    const frame={time:context.state.t};
    for(const sensor of Object.values(this.modules)) Object.assign(frame,sensor.sample(context));
    return frame;
  }
}
