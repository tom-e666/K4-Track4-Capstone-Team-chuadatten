// Presets change physical initial conditions and guidance, not a prerecorded animation.
export const SCENARIOS = Object.freeze({
  direct:{name:'Cua muộn · đáp thẳng',riverWidth:24,initialSpeed:22,driverDelay:1.05,grip:.45,spinIntensity:1.8,rescueDelay:.55,landingStyle:'direct',landingOffset:0},
  wet:{name:'Đường ướt · tiếp cận vòng',riverWidth:30,initialSpeed:23,driverDelay:1.1,grip:.25,spinIntensity:2.4,rescueDelay:.65,landingStyle:'arc',landingOffset:-6},
  spin:{name:'Xoay mạnh · giữ cao rồi đáp',riverWidth:36,initialSpeed:24,driverDelay:1.15,grip:.35,spinIntensity:3.1,rescueDelay:.5,landingStyle:'hover',landingOffset:6,maxLift:28}
});
