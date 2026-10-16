"""Reference navigation server. Replace navigate() with the team's real planner.

Run: python tools/river_controller.py
Then connect ws://127.0.0.1:8765 from viewer/river.html.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math

try:
    from websockets.asyncio.server import serve
except ImportError:
    raise SystemExit("Install the optional adapter: python -m pip install -r tools/river-requirements.txt")


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def navigate(observation: dict) -> dict:
    """Reference slip/rescue controller. Replace this function with the team's planner."""
    s, sensors = observation["state"], observation["sensors"]
    c, vehicle, world = observation["scenario"], observation["vehicle"], observation["map"]
    guidance = observation["guidance"]
    target = guidance["targetLandingPoint"]
    activate = s["rescueTriggered"] or (s["autoRescueEnabled"] and s["airborneSince"] is not None
        and s["t"] - s["airborneSince"] >= c["rescueDelay"])
    if not vehicle["flightAvailable"] or not activate or target is None:
        reacted = s["driverAttentionSince"] is not None and s["t"] - s["driverAttentionSince"] >= c["driverDelay"]
        error = math.atan2(math.sin(-math.pi/2-s["yaw"]), math.cos(-math.pi/2-s["yaw"]))
        return {"brake": int(reacted or observation["metrics"]["distanceToEdge"] <= c["brakeDistance"]),
                "steer": clamp(error*2.5,-1,1) if reacted and s["grounded"] else 0}
    if s["grounded"] and s["x"] > world["farBankX"]:
        return {"brake": 1, "flight": False}
    waypoint = guidance.get("targetWaypoint", target)
    dx, dz = waypoint["x"]-s["x"], waypoint["z"]-s["z"]
    distance = math.hypot(dx,dz)
    stabilizing = s["y"] < 1 or abs(s["pitch"]) > .45 or abs(s["roll"]) > .45
    desired_speed = 0 if s["hoverMode"] or stabilizing else min(10, math.sqrt(max(0,2*2.6*(distance-.65))))
    desired_vx = dx/distance*desired_speed if distance > .01 else 0
    desired_vz = dz/distance*desired_speed if distance > .01 else 0
    measured_y = s["y"] if sensors["altitudeAboveGround"] is None else sensors["altitudeAboveGround"] + (world["waterY"] if sensors["surfaceBelow"] == "water" else 0)
    height = guidance.get("targetAltitude",c["hoverHeight"])
    acceleration_y = clamp((height-measured_y)*2.2-sensors["verticalVelocity"]*2.8,-4,9)
    rcs_first = s["rescueSince"] is None or s["t"]-s["rescueSince"] < .18
    lift = 0 if rcs_first else clamp((vehicle["gravity"]+acceleration_y)/vehicle["maxLift"],0,1)
    angular = sensors["angularVelocity"]
    pitch_mix = (-8*sensors["pitch"]-4*angular["pitch"])*vehicle["pitchInertiaPerMass"]/(vehicle["maxLift"]*vehicle["leverX"])
    roll_mix = (-9*sensors["roll"]-4.5*angular["roll"])*vehicle["rollInertiaPerMass"]/(vehicle["maxLift"]*vehicle["leverZ"])
    yaw_error = guidance.get("targetYaw",0)-sensors["yaw"]
    yaw_error = math.atan2(math.sin(yaw_error),math.cos(yaw_error))
    yaw = clamp(-4.5*angular["yaw"] if abs(angular["yaw"])>1 else 5*yaw_error-3.8*angular["yaw"],-6,6)/6
    pitch = clamp(-3*sensors["pitch"]-2.5*angular["pitch"],-4,4)/4
    roll = clamp(-3*sensors["roll"]-2.5*angular["roll"],-4,4)/4
    return {"flight":True,"lift":lift,
        "acceleration":{"x":clamp((desired_vx-s["vx"])*1.8,-10,5),"z":clamp((desired_vz-s["vz"])*1.8,-6,6)},
        "attitude":{"yawPositive":max(0,yaw),"yawNegative":max(0,-yaw),"pitchPositive":max(0,pitch),"pitchNegative":max(0,-pitch),"rollPositive":max(0,roll),"rollNegative":max(0,-roll)},
        "thrusters":{"frontLeft":0 if rcs_first else clamp(lift+pitch_mix+roll_mix,0,1),"frontRight":0 if rcs_first else clamp(lift+pitch_mix-roll_mix,0,1),
                    "rearLeft":0 if rcs_first else clamp(lift-pitch_mix+roll_mix,0,1),"rearRight":0 if rcs_first else clamp(lift-pitch_mix-roll_mix,0,1)},
        "throttle":clamp((desired_speed-s["speed"])*.5,0,1),"brake":clamp((s["speed"]-desired_speed)*.55,0,1),"steer":0}


async def handle(socket):
    print("Navigation client connected", flush=True)
    try:
        async for message in socket:
            try:
                observation = json.loads(message)
                if observation.get("schema") != "river-nav/1" or observation.get("type") != "observation":
                    continue
                controls = navigate(observation)
                await socket.send(json.dumps({
                    "schema": "river-nav/1", "type": "command",
                    "runId": observation["runId"], "seq": observation["seq"], "controls": controls,
                }, allow_nan=False))
            except (ValueError, TypeError, KeyError) as error:
                print(f"Invalid observation/planner result: {error}", flush=True)
    finally:
        print("Navigation client disconnected", flush=True)


async def main(host: str, port: int):
    async with serve(handle, host, port, max_size=64 * 1024):
        print(f"Reference navigator ready at ws://{host}:{port}", flush=True)
        await asyncio.get_running_loop().create_future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    args = parser.parse_args()
    try:
        asyncio.run(main(args.host, args.port))
    except KeyboardInterrupt:
        pass
