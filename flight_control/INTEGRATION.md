# Frontend handoff contract v1.0.0

The frontend consumes exported physics and safety decisions. It does not need
its own dynamics, fuel, contact, landing or occupant-load calculations. No browser
or teammate UI files are changed by this package.

## Reproduce from the repository root (PowerShell)

```powershell
.\.venv\Scripts\python.exe -m pip install -r flight_control/requirements.txt
.\.venv\Scripts\python.exe -m flight_control.demo --dt 0.02 --csv --package --output-dir flight_control/evidence/demo
.\.venv\Scripts\python.exe -m flight_control.export flight_control/evidence/demo
.\.venv\Scripts\python.exe -m pytest -q tests/test_flight_control.py tests/test_flight_control_physics.py
```

`pytest` is a development dependency already present in the repository venv; if
setting up a new venv, install it explicitly. `--plot` optionally uses matplotlib.
JSON/CSV export and playback do not need matplotlib.

Each of six scenario/guidance folders contains `telemetry.json` and
`safety_report.json`, plus optional `telemetry.csv`. Folder names:

- `comparison_nominal__predictive`
- `comparison_nominal__early_hover`
- `upside_down_recovery__predictive`
- `sideways_spin_recovery__predictive`
- `too_low_fast_impossible__predictive`
- `failed_roof_thruster__predictive`

`flight_recovery_demo.json` contains the suite summary and nominal comparison.
`flight_control_handoff.zip` contains all scenario JSON/CSV, summary, SHA-256
manifest and documentation/schemas. Evidence is generated locally and ignored by
Git. Teammates regenerate it with the command above; generated megabytes are not
required in a source merge. The preserved local checkpoint is also ignored.

For a full replay check:

```powershell
.\.venv\Scripts\python.exe -m flight_control.demo --dt 0.02 --output-dir flight_control/evidence/replay
.\.venv\Scripts\python.exe -m flight_control.validation flight_control/evidence/demo --replay-dir flight_control/evidence/replay
```

This validates all six required pairs and compares the twelve JSON artifacts
byte-for-byte. Same environment, inputs, dt and dependency versions are required
for exact replay; cross-platform SciPy floating-point bit identity is not promised.

## Schemas, timestamps and animation

Formal JSON Schema 2020-12 contracts are in `schemas/telemetry.schema.json` and
`schemas/safety_report.schema.json`. Schema version is `1.0.0`; additive fields
may appear, so consumers should ignore unknown fields. Missing physical/sensor
measurements use null. JSON never contains NaN or Infinity.

The telemetry root has `schema_version`, `scenario_id`, `guidance_mode`,
`physics_dt_s`, `sample_interval_s`, `world_frame`, `quaternion_order`, `samples`.
At default dt, samples start at zero and follow every 0.02 s (50 Hz), including
the final impact endpoint. No wall-clock timestamps enter the dynamics.

Each `samples[]` row includes:

| Field | Units and meaning |
| --- | --- |
| `time_s` | Endpoint simulation time, seconds |
| `interval_start_s`, `interval_dt_s` | Interval that supplied commands/dynamics; null at the initial sample |
| `physics_ground_truth` | True endpoint position m, velocity m/s, quaternion wxyz, body angular velocity rad/s, mass kg, fuel kg, inertia kg m^2, clearance m, kinetic energies J and contact record |
| `physics_ground_truth.acceleration_world_m_s2` | Free-flight interval-start derivative, excluding rigid impulse; null at t=0 |
| `physics_ground_truth.angular_acceleration_body_rad_s2` | Interval-start body derivative; excludes impulse |
| `physics_ground_truth.dynamics_evaluation_time_s`, `dynamics_evaluation_state` | Exact state/time used for derivatives, force frame and seat acceleration |
| `sensor_measurements` | `mode=NOT_SIMULATED`; IMU/radar/LiDAR null |
| `estimated_state` | `mode=PERFECT_STATE`; actual estimator copy used by the controller, at interval start, with null residuals/uncertainty |
| `controller_mode`, `controller_diagnostics` | Predictive/early-hover, selected phase, desired velocity/acceleration and attitude control request |
| `requested_force_body_N`, `requested_torque_body_Nm` | Controller request in interval-start body frame |
| `achieved_force_body_N`, `achieved_torque_body_Nm` | Sum of physical actuator forces/moments, same frame |
| `thruster_states[]` | Name, position, commanded/applied N, force direction, force N, moment N m, gimbal rad, saturation, availability/failure and interval fuel/flow |
| `fuel_consumed_kg`, `fuel_remaining_kg`, `mass_flow_kg_s` | Actual inventory and interval flow |
| `ground_clearance_m` | Lowest box-corner height; negative penetration retained |
| `attitude`, `attitude_error_deg` | Euler angles, world-up tilt, quaternion error and angular rate |
| `vertical_speed_m_s`, `horizontal_speed_m_s` | Signed world Z velocity and XY speed magnitude |
| `seat_free_flight` | True interval-start seat acceleration, specific-force vector and g magnitude |
| `recovery_feasible` | false only for a proven optimistic braking-bound violation; otherwise null |
| `allocation_feasible`, residual fields | Actuator-request feasibility, distinct from trajectory feasibility |
| `diagnostic_events[]` | Codes such as ACTUATOR_REQUEST_INFEASIBLE, ALLOCATION_SOLVER_NOT_CONVERGED, RECOVERY_INFEASIBLE, FIRST_GROUND_CONTACT |

To animate, advance a playback clock in simulated seconds, bracket it by row
`time_s`, linearly interpolate position and slerp normalized quaternions. For
thruster visual effects over `(t_k,t_(k+1)]`, use the **ending row's** interval
records, and use `applied_thrust_N` rather than requested thrust. Use the previous
row's body orientation to draw interval-start force vectors. At contact, display
the exact endpoint and discrete contact event; do not smooth impulses into fake
measured forces. Stop/freeze at the final sample: no bounce/settling was simulated.
Use the final report verdict. Interpolation is visual and not an extra physics
measurement. If the frontend is served over HTTP, copy/extract the selected
output folder into its static assets and fetch the two JSON files by URL.

## Python Z-up to Three.js Y-up

Map vectors by `B * [x,y,z] = [x,z,-y]`. This is a **-90 degree rotation about X**
(the earlier checkpoint document had the sign wrong). `det(B)=+1`, so handedness
is preserved. Quaternions in Python are active body-to-world `[w,x,y,z]`.
Three.js constructors take `(x,y,z,w)`.

If mesh-local axes and body-local thruster positions are also converted by B:

```
qBasis_wxyz = [cos(pi/4), -sin(pi/4), 0, 0]
qThree = qBasis * qSim * inverse(qBasis)
positionThree = B * positionWorld
localThrusterPositionThree = B * positionBody
localDirectionThree = B * forceDirectionBody
```

If mesh-local axes remain the original Python body axes, use `qBasis*qSim`
instead; conjugation applies only when converting both local and world bases.
Apply a separately documented static asset correction if the model's authored
axes differ. Place the mesh origin at simulated CG, not a visual bounding corner.
World contact points/normals/forces are converted by B as well. Do not rotate a
world-space force through body orientation a second time.

Thrusters map by stable names, not array order: roof_1..4, underside_1..4,
side_left, side_right. Their actual geometry is in `thruster_states.position_body_m`
and report vehicle assumptions. Roof force is body -Z, underside force body +Z;
exhaust visuals point opposite the exported force direction. Gimbal directions
are already resolved by Python. Failed `roof_1` has exactly zero actual force.

## Report and interpretation

`safety_report.json` contains scenario/vehicle assumptions, independent
`assessments`, `failure_reasons`, `threshold_results`, `flight_stability`,
`fuel_summary`, `thruster_statistics`, `allocation_summary`, `touchdown`,
`contact_loads`, `touchdown_prediction`, `occupant_load_proxy`, illustrative impact
threshold and `limitations`. Units are encoded in field suffixes. A report
`success=true` means only the unchanged four demo landing limits passed.

`CONTROL_STABLE`, `LANDING_PASS`, illustrative load flag, `STRUCTURAL_UNKNOWN`
and occupant `NOT_VALIDATED` must be displayed as separate assessments. Null
means unavailable or unproven, never zero. A failed actuator request does not
by itself prove an unrecoverable trajectory. See `SAFETY_METRICS.md` for exact
thresholds and definitions.

Contact forces are first-solve **simulated rigid-body corner loads**, averaged
using the assumed 0.05 s duration. They are not wheel loads, chassis stress or
actual impact peaks. Seat metrics are load proxies, not injury risk. The model
is a 120 kg scaled prototype with hypothetical propulsion assumptions. Display
these assumptions with the evidence; do not label the output validated Tesla
physics, compressed-air hardware, structural safety or passenger safety.

## Optional future sensor boundary

`StateEstimator.estimate(truth, dt)` is the existing adapter seam. The controller
records its actual estimated copy, separately from the physics record. No fusion
is currently implemented. Schema nullable sensor payloads specify IMU
`time_s/specific_force_body_m_s2/gyro_body_rad_s`, radar
`time_s/range_m/relative_velocity_m_s`, and LiDAR
`time_s/minimum_ground_clearance_m/ground_observations_world_m`.
A future provider must supply real synthetic/external measurements, their own
timestamps, mode and uncertainty model. Populating these schemas does not itself
implement a sensor or EKF; current exports leave them null.
