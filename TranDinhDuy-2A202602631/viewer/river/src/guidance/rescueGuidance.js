export class RescueGuidance {
  generate({state:s,config:c,landingZone:zone}) {
    if(!zone?.enabled)return {rescuePath:[],targetHoverPoint:{x:s.x,y:c.hoverHeight,z:s.z},targetLandingPoint:null};
    const end=zone.position,path=[],arc=c.landingStyle==='arc'?-9:0;
    const progress=Math.max(0,Math.min(1,s.x/end.x));
    const remaining=end.x-s.x,holding=c.landingStyle==='hover'&&(s.hoverSince===null||s.t-s.hoverSince<1.5);
    const waypoint=holding?{x:end.x-5,y:c.hoverHeight,z:end.z}:c.landingStyle==='arc'&&s.x<c.riverWidth+12?
      {x:Math.min(end.x,s.x+9),y:c.hoverHeight,z:end.z+arc*Math.sin(Math.PI*Math.min(1,(s.x+9)/end.x))}:{...end};
    let altitude=s.x<c.riverWidth+3?c.hoverHeight:Math.min(c.hoverHeight,Math.max(0,remaining*.32));
    if(remaining<4)altitude=-.2;
    if(holding||s.hoverMode)altitude=c.hoverHeight;
    if(s.forceLanding&&s.x>c.riverWidth+2.3)altitude=-.2;
    // Guide only: physics follows commands, never a position on this curve.
    for(let i=0;i<=36;i++) {
      const u=i/36;
      path.push({x:s.x+(end.x-s.x)*u,y:s.y*(1-u)+end.y*u+Math.sin(Math.PI*u)*c.hoverHeight,
        z:s.z+(end.z-s.z)*u+arc*Math.sin(Math.PI*u)*(1-progress)});
    }
    return {rescuePath:path,targetHoverPoint:{x:end.x-5,y:c.hoverHeight,z:end.z},targetLandingPoint:{...end},
      targetWaypoint:waypoint,targetAltitude:altitude,targetYaw:remaining<12?(zone.heading??0):Math.atan2(waypoint.z-s.z,Math.max(3,waypoint.x-s.x)),
      holding,landingStyle:c.landingStyle};
  }
}
