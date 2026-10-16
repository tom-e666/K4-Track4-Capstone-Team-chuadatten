# Physics ground truth and audit

This is a hypothetical scaled **120 kg dry + 18 kg propellant** rigid vehicle.
It is not a production Tesla Roadster, calibrated compressed-air system, or
validated passenger vehicle. No tank pressure, volume, temperature or gas species
is inferred. The 1800 m/s effective exhaust velocity is a propulsion placeholder.

## Frames, state and equations

All calculations use SI: kg, m, s, N, N m, kg m². Body and world frames are
right-handed. World gravity is `g_world = (0, 0, -9.81)` m/s² and ground is `z=0`.
The body origin is the fixed assumed center of mass; body +Z is vehicle up.
`q=[w,x,y,z]` is an active body-to-world quaternion; `R(q)` rotates a body vector
into world coordinates. Angular velocity and inertia are expressed in body axes.

For each actuator, its actual bounded magnitude and clamped gimbal direction give

```
F_i_body = applied_thrust_i * direction_i_body
tau_i_body = r_i_body × F_i_body
F_body = sum(F_i_body); tau_body = sum(tau_i_body)
m * a_world = R(q) * F_body + m * g_world + F_external_world
I * alpha_body + omega_body × (I * omega_body) = tau_body
q_dot = 0.5 * q ⊗ [0, omega_body]
dm_i/dt = -|F_i_body| / v_effective
m = m_dry + sum(m_i)
I(m) = diag(55,65,80) * m/120
E_trans = 0.5 * m * (v_world · v_world)
E_rot = 0.5 * omega_body^T * I * omega_body
```

`F_external_world` is zero in the scenario runner. The low-level `physics.advance`
accepts an external force for isolated verification. `physics.py` imports no
controller or estimator. Only bounded actual forces drive its dynamics; requested
wrenches and optimizer estimates never replace physical forces.

Roof jets `roof_1..4` act toward body -Z, 520 N maximum each. Underside jets
`underside_1..4` act toward body +Z, 360 N each. Their XY positions are, in order,
`(-.48,-.42),(-.48,.42),(.48,-.42),(.48,.42)` m; Z positions are `.42` and `-.38` m.
Side jets are at `side_left=(-.52,0,0)`, `side_right=(.52,0,0)` m, each 180 N.
Their direction is `(-sin(yaw)cos(pitch), cos(yaw)cos(pitch), sin(pitch))`;
yaw is bounded at ±150°, pitch at ±35°. These are assumed mechanical ranges,
with instantaneous gimbal motion and no actuator lag or slew limits.

Initial propellant is allocated to ten separate reservoirs in proportion to each
jet's rated thrust. There is no crossfeed. In each interval,
`F_applied <= min(F_max, m_i * v_effective / dt)`; failed or empty jets apply
exactly zero force and consume zero fuel. Clipping a rounding-scale negative fuel
remainder to zero does not create propellant. Export validation checks interval
and total mass accounting.

## Fixed-step integration and timestamp conventions

The default step is 0.02 s, with no random sources or adaptive stepping:

1. Estimate an isolated copy of state at `t_k`; compute guidance and commands.
2. Enforce actuators using fuel at `t_k`; evaluate `a_k`, `alpha_k`, `m_k`, `I_k`.
3. `v_(k+1)^- = v_k + a_k*dt`; `x_(k+1) = x_k + v_(k+1)^-*dt`.
4. `omega_(k+1)^- = omega_k + alpha_k*dt`.
5. `q_(k+1) = normalize(q_k ⊗ exp([0,omega_(k+1)^-]*dt/2))`.
6. Subtract each reservoir's `F_i/v_effective*dt`.
7. Resolve contact with the end-of-step mass/inertia. Stop at first impact.

Translation and angular velocity use first-order semi-implicit integration.
The quaternion exponential is exact for the held angular rate, but holding a
rate during an accelerating interval is an approximation. Normalization chooses
the equivalent quaternion with nonnegative scalar component. Euler angles are
diagnostics with the usual pitch singularity; the controller uses quaternions.

Telemetry has an initial state at `t=0`, then every physics endpoint. Commands,
forces, free-flight accelerations, seat acceleration and estimator output in row
`t_(k+1)` belong to the preceding interval. Their evaluation time and state are
explicitly exported. They are not instantaneous post-impact derivatives. The
initial row has null accelerations and no applied-thruster interval records.
Timeout is rounded up to a fixed-step boundary (at most one extra dt).

## Rigid-body contact derivation

The body is a box with half-extents `(.62,.56,.40)` m. Eight corners are ordered
by X sign, then Y sign, then Z sign, each negative before positive. A corner
participates only when its computed height is at/below the plane. For each:

```
r_world = R * r_body
v_point = v_CG + (R * omega_body) × r_world
K_n = 1/m + (r_world × n)^T * I_world^-1 * (r_world × n)
delta_jn = (target_normal_velocity - v_point·n) / K_n
j_n_accumulated = max(0, j_n_accumulated + delta_jn)
delta_v_CG = J_world / m
delta_omega_body = R^T * I_world^-1 * (r_world × J_world)
```

Normal is world `(0,0,1)`. Target separating velocity is `-e*min(v_pre·n,0)`,
with assumed restitution `e=.04`. A projected sequential solver performs 64
normal/friction sweeps; signed changes to accumulated normal impulse are applied.
Tangential accumulated impulses are projected to `|J_t| <= .45*J_n`.
No four-wheel load division is assumed. Per-corner impulse, point velocity,
angular impulse and a normal-velocity constraint residual remain auditable.
With simultaneous rigid corners, the impulse distribution can be nonunique and
depends on solver ordering/iterations. These are this solver's corner impulses,
not uniquely determined material loads.

There is **no position projection, upright pose replacement or velocity reset**.
First contact is detected at the next fixed boundary; negative clearance and
penetration are exported. It can miss a transient rotating corner crossing
between boundaries. There is no continuous collision detection. Reported loads
describe this first contact solve, not a complete crash or all subsequent impacts.
The simulated body corners are not tire/suspension or frame measurement points;
some assumed thruster positions also extend beyond the collision box.

`average_normal_force_approx = J_n / .05 s`, and total average force/torque use
the same explicitly assumed interval. This interval is independent of physics dt.
An ideal rigid impulse has no finite resolved peak force: the peak field is null.
Kinetic-energy loss is `E_before - E_after` at the same contact mass/inertia. It
does not identify deformation energy, frame stress or damage.

## Verification and numerical limits

Automated analytic checks cover independent frame rotations, exact held-rate
quaternions, opposite fixed-jet directions, force/torque/mass bounds, gravity-only
motion, zero specific force in free fall, momentum and energy in isolated
constant-mass principal-axis motion, torque-free asymmetric convergence, contact
linear/angular impulse balance, Coulomb bounds, and rigid seat kinematics.
The export validator independently reconstructs equations from serialized state,
checks integration without pose projection, and validates both JSON Schemas.
`STATUS.md` records actual suite results and timestep/replay evidence.

Semi-implicit free fall loses `0.5*m*g²*T*dt` of mechanical energy relative to
its discrete potential/kinetic accounting. General torque-free angular motion
has finite-step drift that decreases with dt. Exhaust carries momentum and energy
outside the modeled vehicle, so powered-flight vehicle energy is not conserved.
The proportional inertia law is a fixed-shape approximation: tank geometry,
shifting center of mass, inertia-rate/exhaust angular-momentum terms and slosh
are not resolved. The simple rotational equation does not claim exact conservation
for changing mass. No aerodynamic, elastic, thermal or plume model is present.

The allocator is bounded nonlinear least squares with local minima and a finite
iteration budget. Residual-based feasibility describes a requested actuator
wrench; it is not a proof that the entire trajectory can recover. The separate
optimistic braking bound can prove some falls impossible; otherwise feasibility
is null. Failed recoveries remain measured failures.
