"""Controller-independent dynamics and measured rigid-box contact impulses."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model import (FlightConfig, FlightState, ThrusterCommand, body_corners,
                    quat_integrate_body_rate,
                    quat_to_matrix, thruster_force_direction)

STANDARD_G = 9.80665


def mass(config: FlightConfig, state: FlightState) -> float:
    return config.dry_mass_kg + state.remaining_propellant_kg


def inertia_body(config: FlightConfig, mass_kg: float) -> np.ndarray:
    return np.diag(config.inertia_dry_kg_m2) * mass_kg / config.dry_mass_kg


def kinetic_energy(state: FlightState, mass_kg: float, inertia: np.ndarray) -> dict:
    v, w = state.velocity_world_m_s, state.angular_velocity_body_rad_s
    trans = 0.5 * mass_kg * float(v @ v)
    rot = 0.5 * float(w @ inertia @ w)
    return {"translational_J": trans, "rotational_J": rot, "total_J": trans + rot}


def applied_thrusters(config: FlightConfig, state: FlightState,
                      commands: tuple[ThrusterCommand, ...], dt: float) -> tuple:
    """Physical actuator enforcement, independent of allocator diagnostics."""
    if len(commands) != len(config.thrusters) or not np.isfinite(dt) or dt <= 0:
        raise ValueError("one command per thruster and positive finite dt required")
    forces, torques, flows, records = [], [], [], []
    for i, (t, c) in enumerate(zip(config.thrusters, commands)):
        available = t.name not in config.failed_thrusters and state.propellant_by_thruster_kg[i] > 0
        limit = min(t.max_thrust_N, state.propellant_by_thruster_kg[i] * config.effective_exhaust_velocity_m_s / dt)
        actual = float(np.clip(c.thrust_N, 0, limit)) if available else 0.0
        yaw = float(np.clip(c.gimbal_yaw_rad, -t.gimbal_yaw_limit_rad, t.gimbal_yaw_limit_rad))
        pitch = float(np.clip(c.gimbal_pitch_rad, -t.gimbal_pitch_limit_rad, t.gimbal_pitch_limit_rad))
        direction = thruster_force_direction(t, yaw, pitch)
        force = actual * direction
        torque = np.cross(t.position_body_m, force)
        flow = actual / config.effective_exhaust_velocity_m_s
        forces.append(force)
        torques.append(torque)
        flows.append(flow)
        records.append({
            "name": t.name, "position_body_m": list(t.position_body_m),
            "commanded_thrust_N": float(c.thrust_N), "applied_thrust_N": actual,
            "force_direction_body": direction.tolist(), "force_body_N": force.tolist(),
            "torque_body_Nm": torque.tolist(), "gimbal_yaw_rad": yaw,
            "gimbal_pitch_rad": pitch, "available": bool(available),
            "failed": t.name in config.failed_thrusters,
            "saturated": bool(available and actual >= limit - max(0.5, limit * 0.01)),
            "mass_flow_kg_s": flow, "gas_consumed_interval_kg": flow * dt,
            "fuel_remaining_before_kg": float(state.propellant_by_thruster_kg[i]),
            "max_thrust_N": t.max_thrust_N,
        })
    return np.array(forces), np.array(torques), np.array(flows), records


def seat_acceleration(acceleration_world: np.ndarray, alpha_body: np.ndarray,
                      omega_body: np.ndarray, rotation: np.ndarray,
                      seat_body: tuple, gravity_world: np.ndarray) -> dict:
    r = np.asarray(seat_body)
    a = acceleration_world + rotation @ (np.cross(alpha_body, r) + np.cross(omega_body, np.cross(omega_body, r)))
    specific = a - gravity_world
    return {"position_body_m": r.tolist(), "acceleration_world_m_s2": a.tolist(),
            "specific_force_world_m_s2": specific.tolist(),
            "specific_force_g": float(np.linalg.norm(specific) / STANDARD_G)}


def resolve_contact(state: FlightState, config: FlightConfig, mass_kg: float,
                    inertia: np.ndarray) -> dict | None:
    """Projected sequential impulses; no position projection or velocity reset.

    Only corners at/below the ground plane participate. The simulation stops
    after first impact; discrete penetration is retained and exported.
    """
    rotation = quat_to_matrix(state.orientation_body_to_world)
    corners = body_corners(config.body_half_extents_m)
    offsets = corners @ rotation.T
    points = state.position_world_m + offsets
    heights = points[:, 2] - config.ground_z_m
    ids = np.flatnonzero(heights <= 0)
    if not len(ids):
        return None
    pre = state.copy()
    inv_inertia = rotation @ np.linalg.inv(inertia) @ rotation.T
    normal = np.array([0., 0., 1.])
    pre_w_world = rotation @ pre.angular_velocity_body_rad_s
    pre_points_v = [pre.velocity_world_m_s + np.cross(pre_w_world, r) for r in offsets]
    targets = {i: -config.contact_restitution * min(0., pre_points_v[i][2]) for i in ids}
    normal_accum = {i: 0. for i in ids}
    tangent_accum = {i: np.zeros(3) for i in ids}

    def apply(impulse: np.ndarray, r: np.ndarray) -> None:
        state.velocity_world_m_s += impulse / mass_kg
        state.angular_velocity_body_rad_s += rotation.T @ inv_inertia @ np.cross(r, impulse)

    for _ in range(64):
        for i in ids:
            r = offsets[i]
            point_v = state.velocity_world_m_s + np.cross(rotation @ state.angular_velocity_body_rad_s, r)
            rn = np.cross(r, normal)
            k = 1 / mass_kg + float(rn @ inv_inertia @ rn)
            old = normal_accum[i]
            normal_accum[i] = max(0., old + (targets[i] - point_v[2]) / k)
            apply((normal_accum[i] - old) * normal, r)
            point_v = state.velocity_world_m_s + np.cross(rotation @ state.angular_velocity_body_rad_s, r)
            tangent_v = point_v - point_v[2] * normal
            speed = float(np.linalg.norm(tangent_v))
            if speed > 1e-12:
                direction = tangent_v / speed
                rt = np.cross(r, direction)
                kt = 1 / mass_kg + float(rt @ inv_inertia @ rt)
                desired = tangent_accum[i] - tangent_v / kt
            else:
                desired = tangent_accum[i].copy()
            limit = config.contact_friction * normal_accum[i]
            length = float(np.linalg.norm(desired))
            if length > limit:
                desired *= limit / length
            apply(desired - tangent_accum[i], r)
            tangent_accum[i] = desired

    contacts = []
    total_impulse, angular_impulse = np.zeros(3), np.zeros(3)
    residual = 0.
    for i in ids:
        impulse = normal_accum[i] * normal + tangent_accum[i]
        moment = np.cross(offsets[i], impulse)
        total_impulse += impulse
        angular_impulse += moment
        post_v = state.velocity_world_m_s + np.cross(rotation @ state.angular_velocity_body_rad_s, offsets[i])
        residual = max(residual, max(0., targets[i] - post_v[2]))
        contacts.append({
            "contact_id": f"box_corner_{i}", "region": "simulated_rigid_body_corner",
            "position_body_m": corners[i].tolist(), "position_world_m": points[i].tolist(),
            "normal_world": normal.tolist(), "normal_impulse_Ns": float(normal_accum[i]),
            "tangential_impulse_world_Ns": tangent_accum[i].tolist(),
            "total_impulse_world_Ns": impulse.tolist(),
            "average_normal_force_approx_N": float(normal_accum[i] / config.impact_stopping_time_assumption_s),
            "angular_impulse_about_CG_world_Nms": moment.tolist(),
            "pre_impact_point_velocity_world_m_s": pre_points_v[i].tolist(),
            "post_impact_point_velocity_world_m_s": post_v.tolist(),
        })
    before_energy = kinetic_energy(pre, mass_kg, inertia)
    after_energy = kinetic_energy(state, mass_kg, inertia)
    return {
        "contacts": contacts, "normal_contact_impulse_Ns": float(sum(normal_accum.values())),
        "total_impulse_world_Ns": total_impulse.tolist(),
        "total_average_force_approx_world_N": (total_impulse / config.impact_stopping_time_assumption_s).tolist(),
        "angular_impulse_about_CG_world_Nms": angular_impulse.tolist(),
        "average_torque_approx_world_Nm": (angular_impulse / config.impact_stopping_time_assumption_s).tolist(),
        "pre_impact_velocity_world_m_s": pre.velocity_world_m_s.tolist(),
        "post_impact_velocity_world_m_s": state.velocity_world_m_s.tolist(),
        "pre_impact_angular_velocity_body_rad_s": pre.angular_velocity_body_rad_s.tolist(),
        "post_impact_angular_velocity_body_rad_s": state.angular_velocity_body_rad_s.tolist(),
        "kinetic_energy_before": before_energy, "kinetic_energy_after": after_energy,
        "kinetic_energy_dissipated_approx_J": before_energy["total_J"] - after_energy["total_J"],
        "max_penetration_m": max(0., -float(np.min(heights))),
        "normal_velocity_constraint_residual_m_s": residual,
        "solver_iterations": 64, "force_time_assumption_s": config.impact_stopping_time_assumption_s,
        "true_peak_contact_force_N": None,
        "unavailable_reason": "Rigid impulse model has no force-time curve, tire, suspension or structural compliance.",
    }


@dataclass
class PhysicsStep:
    before: FlightState
    before_contact: FlightState
    force_body_N: np.ndarray
    torque_body_Nm: np.ndarray
    acceleration_world_m_s2: np.ndarray
    angular_acceleration_body_rad_s2: np.ndarray
    mass_before_kg: float
    inertia_before: np.ndarray
    thrusters: list[dict]
    mass_flow_kg_s: float
    seat: dict
    contact: dict | None


def advance(state: FlightState, config: FlightConfig, commands: tuple,
            dt: float, external_force_world_N: np.ndarray | None = None,
            ground_contact: bool = True) -> PhysicsStep:
    before = state.copy()
    forces, torques, flow, thrusters = applied_thrusters(config, state, commands, dt)
    m = mass(config, state)
    inertia = inertia_body(config, m)
    rotation = quat_to_matrix(state.orientation_body_to_world)
    force, torque = forces.sum(axis=0), torques.sum(axis=0)
    external = np.zeros(3) if external_force_world_N is None else np.asarray(external_force_world_N, dtype=float)
    if external.shape != (3,) or not np.all(np.isfinite(external)):
        raise ValueError("external force must be a finite world 3-vector")
    gravity = np.array([0., 0., -config.gravity_m_s2])
    accel = (rotation @ force + external) / m + gravity
    omega = before.angular_velocity_body_rad_s
    alpha = np.linalg.solve(inertia, torque - np.cross(omega, inertia @ omega))
    seat = seat_acceleration(accel, alpha, omega, rotation, config.seat_position_body_m, gravity)
    state.velocity_world_m_s += accel * dt
    state.position_world_m += state.velocity_world_m_s * dt
    state.angular_velocity_body_rad_s += alpha * dt
    state.orientation_body_to_world = quat_integrate_body_rate(state.orientation_body_to_world, state.angular_velocity_body_rad_s, dt)
    state.propellant_by_thruster_kg = np.maximum(0., state.propellant_by_thruster_kg - flow * dt)
    state.time_s += dt
    before_contact = state.copy()
    contact = resolve_contact(state, config, mass(config, state), inertia_body(config, mass(config, state))) if ground_contact else None
    for i, record in enumerate(thrusters):
        record["fuel_remaining_after_kg"] = float(state.propellant_by_thruster_kg[i])
    return PhysicsStep(before, before_contact, force, torque, accel, alpha, m, inertia,
                       thrusters, float(flow.sum()), seat, contact)
