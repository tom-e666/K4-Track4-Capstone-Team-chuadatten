# Flight recovery control and evidence package

Offline Python robotics/control work for `feat/thanh-flight-control`. No frontend
dashboard or teammate component is included. The scaled reference vehicle has
120 kg dry mass, 18 kg initial finite propellant and ten thrusters. These are
simulation assumptions, not actual Tesla Roadster specifications. Propulsion is
hypothetical and is not calibrated compressed air.

Run from the repository root:

```powershell
.\.venv\Scripts\python.exe -m pip install -r flight_control/requirements.txt
.\.venv\Scripts\python.exe -m pytest -q tests/test_flight_control.py tests/test_flight_control_physics.py
.\.venv\Scripts\python.exe -m flight_control.demo --dt 0.02 --csv --package --output-dir flight_control/evidence/demo
.\.venv\Scripts\python.exe -m flight_control.export flight_control/evidence/demo
```

Outputs: six scenario/guidance folders, each with `telemetry.json` and
`safety_report.json`; optional CSV; suite summary; SHA-256 manifest; and
`flight_control_handoff.zip`. Generated evidence is ignored by Git. The package
contains source-controlled schemas/docs plus reproducible generated evidence.
`pytest` is required for tests; optional `--plot` needs matplotlib.

| Module | Responsibility |
| --- | --- |
| `model.py` | Configuration, quaternion math, ten-actuator geometry and state |
| `controller.py` | Perfect-state adapter, quaternion PD, early-hover/predictive descent envelope |
| `allocation.py` | Bounded nonlinear least-squares wrench allocation |
| `physics.py` | Independent 6-DOF dynamics, finite fuel, per-actuator force, rigid-corner impulses and seat kinematics |
| `simulation.py` | Deterministic scenario runner through first physical impact |
| `telemetry.py` | Ground truth/estimate separation, stability metrics and independent assessments |
| `export.py`, `schemas/` | Strict JSON, JSON Schema validation, physical accounting checks, CSV and handoff zip |
| `validation.py` | Required scenario coverage and byte-identical replay checks |

Read [PHYSICS_GROUND_TRUTH.md](PHYSICS_GROUND_TRUTH.md) for equations and numerical
limitations, [SAFETY_METRICS.md](SAFETY_METRICS.md) for threshold/proxy definitions,
[INTEGRATION.md](INTEGRATION.md) for playback and Three.js conversion, and
[STATUS.md](STATUS.md) for executed results, including failed recoveries.

Landing thresholds are unchanged demo criteria. Rigid contact forces assume a
0.05 s averaging duration; they are not stress or damage. Occupant assessment is
`NOT_VALIDATED`. The simulator stops at first impulse and does not model settling,
suspension, biomechanics, real sensors, tank thermodynamics or calibrated vehicle
hardware. A failed scenario is valid evidence, never replaced by animation.
