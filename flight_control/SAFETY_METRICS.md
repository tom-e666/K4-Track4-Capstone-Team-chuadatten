# Stability, contact loads and occupant proxies

These are measurements and illustrative diagnostics for a scaled simulation.
They are not automotive certification, FEM, chassis stress, injury probability
or validated crashworthiness. `occupant_safety_assessment` is always
`"NOT_VALIDATED"`; `structural_assessment` is `"STRUCTURAL_UNKNOWN"`.

## Separate assessments

| Field | Meaning |
| --- | --- |
| `control_stability` | `CONTROL_STABLE` only after the final continuous 0.5 s flight window meets ≤5° quaternion error and ≤0.15 rad/s angular rate. Otherwise `CONTROL_UNSTABLE`. |
| `landing_criteria` | `LANDING_PASS` iff all four first-contact demo limits below pass; otherwise `LANDING_FAIL`, including timeout. |
| `impact_load_assessment` | `HIGH_IMPACT_LOAD` if first-solve CG impulse delta-v / 0.05 s exceeds 5 standard g; otherwise `NO_ILLUSTRATIVE_HIGH_LOAD_FLAG`, or `UNKNOWN` without impact. |
| `structural_assessment` | Always `STRUCTURAL_UNKNOWN`. |
| `occupant_safety_assessment` | Always `NOT_VALIDATED`. |

The **5 g screening threshold is an illustrative engineering assumption** with
no validated injury or structural interpretation. Absence of a flag does not
establish safety. Standard g is 9.80665 m/s². The landing limits are unchanged:

| First-contact measurement | Inclusive maximum |
| --- | ---: |
| Absolute CG vertical speed | 2.0 m/s |
| CG horizontal speed magnitude | 1.5 m/s |
| Body +Z tilt from world +Z | 15 degrees |
| Angular velocity magnitude | 0.55 rad/s |

Measurements use pre-impulse velocity and attitude, never reduced post-impact
speed. Machine-readable reasons identify each exceeded limit. A timeout has
`simulation_timeout` or `simulation_timeout_after_propellant_exhaustion` and null
touchdown measurements. `success` only means these demo landing criteria pass.
Control stability, load flags and unknown safety assessments remain separate.

## Flight stability and actuator diagnostics

Roll/pitch/yaw use ZYX Euler diagnostics. Tilt is `acos(R[2,2])`. Quaternion error
to identity is `2*acos(abs(q.w))`, independent of quaternion sign. Angular velocity
is measured in body rad/s; angular acceleration is the free-flight derivative.
Peak rate includes initial state and pre-impact flight endpoints, excluding the
post-contact angular jump. Peak angular acceleration excludes rigid impulses.

`time_to_upright_stabilization_s` is the first endpoint confirming a continuous
0.5 s dwell. Both ends of every counted interval must meet the limits. It is null
if never attained. Final control stability requires a current dwell, so a past
stable interval does not conceal later instability. `time_outside_tilt_limit_s`
counts intervals whose starting tilt exceeds 15° (left endpoint approximation).

Oscillation is counted as sign changes in each world quaternion-error vector
component, ignoring magnitudes ≤0.5°. `max_attitude_error_after_first_entry_deg`
measures subsequent excursion after first entry into the 5° band; it is not a
frequency-domain damping ratio. Euler yaw is undefined at its singular attitude.

Requested force/torque and actually applied force/torque are exported separately.
The allocator's normalized residual is
`sqrt(sum((force_error/1000N)^2)+sum((torque_error/350Nm)^2))`; requests at ≤0.12
are labeled allocation-feasible. These are fixed normalization scales, not a
relative percentage of the request. The legacy summary name
`steps_over_12_percent_wrench_residual` is retained for compatibility and counts
steps above 0.12 in this normalization. Failed jets are unavailable, not physically
saturated. Actual saturation means within max(0.5 N, 1%) of the current thrust/fuel
limit; solver saturation diagnostics and physical availability are distinct.

The guidance envelope reserves 55% of ideal independent-jet vertical braking
acceleration because simultaneous lateral and attitude demands reduce usable
authority. It changes commanded braking timing, never force physics or landing
limits. Controller diagnostics include target vertical velocity, braking envelope,
desired acceleration, attitude error and actual selected descent/hover phase.

`recovery_feasible=false` is emitted only when even optimistic all-jet upward
thrust at dry mass cannot reach the 2 m/s vertical limit before the remaining
CG height budget. This bound ignores fuel exhaustion and attitude costs and uses
the smallest box half-extent as an optimistic contact height. Otherwise the field
is null, not a guarantee of recovery. `ACTUATOR_REQUEST_INFEASIBLE` is a separate
instantaneous diagnostic. Failed recovery never changes gravity or resets pose.

Clearance is the minimum of all eight true box-corner heights, including negative
penetration. Vertical speed is signed in telemetry and absolute in touchdown
criteria. Horizontal speed is an XY magnitude. Time-to-impact is `clearance /
downward_CG_speed`, null when not descending; it ignores rotation/acceleration.
Initial touchdown prediction holds velocity and orientation constant and ignores
gravity and future control. Predicted versus actual time, position, orientation
and speeds are labeled accordingly. It is a crude reference extrapolation.

## Contact analytics

`contact_loads.contacts[]` identifies `box_corner_0..7`, its body/world location,
normal, normal/tangential/vector impulse, estimated average normal force, angular
impulse about CG, and pre/post point velocities. Per-corner impulses come from
the solver; they are not split equally across four wheels. Total vector impulse,
estimated average vector force, angular impulse and average torque are summed.
Normal scalar impulse differs from vector-impulse magnitude when friction exists.
Simultaneous rigid-corner impulse splits may be nonunique and depend on solver
order/iterations; they do not identify unique physical chassis load paths.

Average force uses the **assumed 0.05 s stopping duration**, never physics dt.
Energy at first contact includes translation and rotation; dissipated energy is
the before/after difference for that first impulse solve. It is not total energy
absorbed by a frame. The solver also provides maximum penetration and a normal
velocity constraint residual. True peak contact force is null because a rigid
impulse has no resolved duration or force profile. First-corner impact can leave
substantial residual motion: there is no simulated settling or subsequent crash.

The compatibility field `peak_vertical_accel_m_s2` combines free-flight vertical
acceleration with impulse delta-v/physics-dt, and is resolution-dependent. It is
not used for occupant assessment or for force averaging. Prefer the separated
fields described here.

## Seat-location load proxy

The configurable seat position relative to CG defaults to body `(0.15,0.20,0.15)` m.
For free flight:

```
a_seat_world = a_CG_world
             + R * (alpha_body × r_seat_body
                    + omega_body × (omega_body × r_seat_body))
specific_force_world = a_seat_world - g_world
specific_force_g = |specific_force_world| / 9.80665
```

All terms use the interval-start true state. Free fall has zero specific force
for a nonrotating seat, even though world acceleration is downward gravity.
Per-step vector/magnitude and the peak free-flight magnitude are exported.
Free-flight jerk is the norm of adjacent seat-specific-force vector differences
divided by dt. Peak/mean describe sampled free-flight values, may include command
discontinuities, and depend on sampling. They exclude the contact impulse.

At impact, with orientation held across the instantaneous impulse:

```
delta_v_seat_world = delta_v_CG_world + R * (delta_omega_body × r_seat_body)
impact_average_seat_deceleration_g = |delta_v_seat_world| / 0.05 / 9.80665
```

CG delta-v and its average deceleration are also reported. Neither is mixed with
delta-v/physics-dt. Impact peak jerk is null with an explanation. No belts, seat
compliance, occupant motion, body orientation, biomechanics, injury probability
or combined safety score is modeled. The collection of proxies is the full
`occupant_load_proxy` output; it does not claim passenger safety.
