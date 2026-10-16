# SKYFALL — Three Tower Challenge

An optional 3D demonstration mode layered over the existing **Lab 11 Three.js viewer**. It reuses the original `viewer/app.js` renderer, orbit-camera infrastructure, ego car mesh, lights, road, ground and city backdrop. The original Lab 11 experience remains available without a query string.

## Open the demo

Open **`viewer/skyfall.html`** in Chrome/Edge, or open **`viewer/index.html?skyfall=1`** directly. No npm install, server, GPU model, data download or build process is needed; the bundled Three.js is used offline.

Switch between:

- **01 Precision Tower Jump** — fly from tower A, through tower B, and stop safely inside tower C.
- **02 Mid-Air Disturbance** — same route with a lateral rotation impulse and loss of vertical velocity.
- **PD/PID thrusters ON** vs **baseline ballistic (no active corrections)**; identical initial states.
- Adjust launch speed and vertical disturbance, then replay. Click **So sánh 20 lượt** for a deterministic multi-run comparison. Export a trajectory CSV.

The three extra skyscrapers have a window-height corridor from `y=44` to `y=64` metres and are displayed over the pre-existing city. Glass is a designated penetrable window region. Above/below it are structural collision regions. The final target is the *interior elevated floor* of tower C, **not a rooftop**.

## Code layout

| File | Role |
| --- | --- |
| `skyfall.html` | One-click redirect to the reused viewer with SKYFALL mode |
| `index.html` | Loads the optional SKYFALL scripts (normal mode unchanged) |
| `app.js` | Exposes the existing rendering scene and adds a short early-return flight hook |
| `skyfall.js` | Adds three towers, glass portals, HUD, playback and exports |
| `skyfall-scenarios.js` | Scenario factory + environment geometry |
| `skyfall-physics.js` | 2D physics, bounded acceleration/moment, obstacle checks and benchmarking |
| `skyfall-tests.js` | Headless Node smoke tests |

Run numeric smoke tests (if Node is installed):

```sh
node viewer/skyfall-tests.js
```

## Model and measurement

State `[x, y, vx, vy, pitch, pitchRate]`; gravity `9.81 m/s²`; fixed integration step `0.02 s`. Actuation is *hypothetical*: the concept vehicle has independently controlled vertical thrust (0–21 m/s²), horizontal acceleration (-7…7 m/s²) and pitch angular acceleration (-16…16 rad/s²). The controlled policy tracks `y=54 m`, decelerates for the tower C target, then descends to its elevated deck. Pitch is stabilized by PD. Baseline has no active thrust or stabilization. Use identical simulation conditions for comparisons.

The success condition is a correct entry through tower B followed by floor contact near the goal in C with `|vx| < 3.5 m/s`, `|vy| < 2.3 m/s`, `|pitch| < 0.18 rad`. Touching structural material outside the permitted opening, hitting the ground, flying past C or timing out fails the run. Recorded `status` and `reason` are based on geometry checks, not on a prerecorded animation.

**Important limitations:** This is an intentionally simplified planar, point-actuated simulation, not a physically validated road car or certification-grade airborne safety simulator. The 3D visualization is a rendering of 2D motion; windows are treated as permission zones, not as destructible glass physics. The 20-run comparison uses controlled perturbations of speed and pitch and must not be reported as a real-world safety probability.

## Scope / future work

A future version could add state estimation from noisy IMU/range data, lateral/z motion, aerodynamic effects, actuator delays, uncertainty bounds and better collision geometry. Those are intentionally left out of the 120-minute MVP.
