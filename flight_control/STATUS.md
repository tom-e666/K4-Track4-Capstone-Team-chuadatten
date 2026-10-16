# Executed flight-control audit ? 2026-10-16

Branch: `feat/thanh-flight-control`. Baseline HEAD before this work:
`11fed89049e27efb3c375fa2a1dd183039dd78fb`. Existing flight-control files and the
original eight tests were untracked; they were preserved in the ignored local
`evidence/initial_checkpoint/` before extension. Unchanged allocator logic and
original scenarios were retained. No Lab 11, avsim, viewer or teammate UI work is
part of this change. No reset/clean/discard was performed.

## Delivered and verified

- Controller-independent 6-DOF ground truth, ten actual forces/torques, finite
  per-jet fuel, quaternion integration and deterministic fixed-step replay.
- Sequential corner impulses with traceable normal/friction/angular impulses,
  energy change and penetration; removed the previous discrete position projection.
- Conservative braking guidance margin (0.45 of ideal net upward acceleration,
  previously 0.68) to reserve authority for simultaneous force/attitude demands.
  Unchanged demo landing thresholds. No force/mass scaling or forced upright pose.
- Per-step stability, seat free-flight specific force and separate impact delta-v
  proxies; explicit unavailable sensors/uncertainty and unknown safety assessment.
- Two JSON Schema 2020-12 contracts, strict exports, independent physics/accounting
  validation, CSV option, scenario runner, SHA-256 manifest and handoff archive.
- Corrected Three.js basis rotation sign: `[x,y,z] -> [x,z,-y]` is -90? about X.

Environment: Python 3.11.9; NumPy 2.4.6; SciPy 1.17.1; pytest 9.1.1;
jsonschema 4.26.0. Only the required JSON Schema validator and its dependencies
were added to the existing venv. Runtime requirements are scoped to this module.

## Default suite: dt=0.02 s

All numbers below are executed simulation results at first resolved impact.
`vz` is absolute vertical speed; `vh` is horizontal speed. PASS means only demo
landing acceptance. Impulse/force reflect the first contact solve, not settling.

| Scenario / guidance | Verdict | Fuel kg | vz m/s | vh m/s | Tilt deg | Rate rad/s | First stabilization s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Nominal / predictive | PASS | 6.220995 | 0.539387 | 0.475979 | 0.6021 | 0.002485 | 1.48 |
| Nominal / early_hover | PASS | 6.325854 | 0.548944 | 0.546473 | 0.0403 | 0.000574 | 1.48 |
| Upside down / predictive | FAIL | 3.729585 | 8.779528 | 5.355196 | 0.7609 | 0.006085 | null |
| Sideways spin / predictive | FAIL | 3.950804 | 4.693130 | 2.319613 | 0.0174 | 0.000627 | 2.12 |
| Too low, fast / predictive | FAIL | 0.067941 | 17.923598 | 0.000013 | 0 | ~0 | null |
| Failed roof_1 / predictive | PASS | 6.665424 | 0.277838 | 0.006365 | ~0 | ~0 | 1.62 |

| Scenario / guidance | Normal impulse N s | Average normal force N | KE before J | KE lost J | Seat impact-average g |
| --- | ---: | ---: | ---: | ---: | ---: |
| Nominal / predictive | 47.1944 | 943.888 | 34.098 | 20.438 | 0.6990 |
| Nominal / early_hover | 75.1730 | 1503.459 | 39.500 | 33.948 | 1.2771 |
| Upside down / predictive | 1225.9841 | 24519.681 | 7100.102 | 6987.531 | 20.4170 |
| Sideways spin / predictive | 654.2747 | 13085.494 | 1836.881 | 1833.501 | 10.9155 |
| Too low, fast / predictive | 2571.1284 | 51422.567 | 22155.707 | 22120.258 | 38.0161 |
| Failed roof_1 / predictive | 37.9493 | 758.986 | 5.072 | 5.064 | 0.5894 |

Upside-down and sideways cases both exceed vertical and horizontal landing limits.
They spend braking authority recovering attitude and lateral motion, leaving too
much descent speed. Maximum normalized allocation residuals are 1.4509 and 1.3275.
They eventually reach low tilt, which does not imply a passing landing. The
upside-down case does not complete the required stabilization dwell before impact;
the sideways case is control-stable yet lands too fast. No proof is claimed that
these two initial states are impossible under every alternative controller.
The low/fast state is provably outside even the optimistic braking bound at its
initial height; the current actuator request can still be allocation-feasible.
`roof_1` remains exactly zero actual force in the failure case.

Nominal predictive saves **0.104859 kg (about 1.66%)** against early-hover at
0.02 s, with both satisfying identical landing criteria. This is a local comparison,
not a robust fuel-efficiency claim and not a passenger-safety comparison.

## Timestep sensitivity: dt=0.05 s

| Scenario / guidance | Verdict | Fuel kg | vz m/s |
| --- | --- | ---: | ---: |
| Nominal / predictive | PASS | 6.550112 | 0.234683 |
| Nominal / early_hover | PASS | 6.416358 | 0.436112 |
| Upside down / predictive | FAIL | 3.739759 | 8.751894 |
| Sideways spin / predictive | FAIL | 3.964149 | 4.723830 |
| Too low, fast / predictive | FAIL | 0.084901 | 17.904515 |
| Failed roof_1 / predictive | PASS | 6.721764 | 0.243580 |

The coarse nominal comparison reverses: predictive consumes **0.133754 kg more**.
Controller discretization/local nonlinear allocation and first-contact sampling
matter. Broad superiority is not demonstrated. Default minimum clearance is
-0.01073 m for nominal predictive and -0.03618 m for low/fast. Coarse low/fast
penetration reaches -0.39284 m; no pose projection hides this discretization error.
Use the default 0.02 s evidence for playback. Continuous collision detection and
finer-step convergence remain future work.

## Reproducible checks and evidence

Executed scoped tests: **44 passed** (8 original + 36 additional parametrized
checks), including an archive/manifest check. Syntax compilation and whitespace
checks passed. Six telemetry/report pairs passed schema, finiteness, timestamp,
force/torque, fuel, contact impulse, seat kinematics and landing consistency checks.
A second complete 0.02 s suite produced **12 byte-identical JSON artifacts** across
six pairs. Six additional coarse-step pairs also validated.

```powershell
.\.venv\Scripts\python.exe -m pytest -q tests/test_flight_control.py tests/test_flight_control_physics.py
.\.venv\Scripts\python.exe -m flight_control.demo --dt 0.02 --csv --package --output-dir flight_control/evidence/demo
.\.venv\Scripts\python.exe -m flight_control.demo --dt 0.02 --output-dir flight_control/evidence/replay
.\.venv\Scripts\python.exe -m flight_control.validation flight_control/evidence/demo --replay-dir flight_control/evidence/replay
.\.venv\Scripts\python.exe -m flight_control.demo --dt 0.05 --output-dir flight_control/evidence/coarse
.\.venv\Scripts\python.exe -m flight_control.export flight_control/evidence/coarse
.\.venv\Scripts\python.exe -m compileall -q flight_control tests/test_flight_control.py tests/test_flight_control_physics.py
git diff --check
```

Generated files live under ignored `flight_control/evidence/demo/`; the zip is
`flight_control_handoff.zip`. The validation summary contains SHA-256 digests.
The old baseline is preserved under `evidence/baseline/` and is not the final
contract. Teammates should regenerate the final package after merging source.

Remaining limits: hypothetical 120 kg prototype/propulsion; no calibrated tank,
sensor fusion, aerodynamics, actuator lag, suspension, elastic structure, occupant
biomechanics or post-impact settling. Proportional inertia and first-order dynamics
have documented conservation errors. Average corner force assumes 0.05 s and is
not peak force, real wheel load, FEM, stress or damage. Structural assessment stays
unknown and occupant assessment stays NOT_VALIDATED even for passing landings.
