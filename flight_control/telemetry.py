"""Versioned telemetry contract, measured stability and end-of-run assessments."""

from __future__ import annotations

from dataclasses import asdict

import numpy as np

from .model import lowest_corner_height, quat_error_vector, quat_to_matrix
from .physics import STANDARD_G, inertia_body, kinetic_energy, mass

SCHEMA_VERSION = "1.0.0"
STABILITY = {"attitude_error_limit_deg": 5., "angular_rate_limit_rad_s": 0.15,
             "dwell_time_s": 0.5, "outside_tilt_limit_deg": 15.,
             "oscillation_deadband_deg": 0.5}
LANDING = {"vertical_speed_m_s": 2., "horizontal_speed_m_s": 1.5,
           "attitude_tilt_deg": 15., "angular_rate_rad_s": 0.55}
# Illustrative screening flag only. No biomechanical or chassis validation.
HIGH_IMPACT_AVERAGE_CG_DECEL_G = 5.


def attitude(state) -> dict:
    r = quat_to_matrix(state.orientation_body_to_world)
    error = quat_error_vector((1, 0, 0, 0), state.orientation_body_to_world)
    return {
        "roll_deg": float(np.rad2deg(np.arctan2(r[2, 1], r[2, 2]))),
        "pitch_deg": float(np.rad2deg(np.arcsin(np.clip(-r[2, 0], -1, 1)))),
        "yaw_deg": float(np.rad2deg(np.arctan2(r[1, 0], r[0, 0]))),
        "tilt_deg": float(np.rad2deg(np.arccos(np.clip(r[2, 2], -1, 1)))),
        "quaternion_error_deg": float(np.rad2deg(np.linalg.norm(error))),
        "error_rotation_vector_world_deg": np.rad2deg(error).tolist(),
        "angular_rate_rad_s": float(np.linalg.norm(state.angular_velocity_body_rad_s)),
    }


def state_fields(state) -> dict:
    return {"position_world_m": state.position_world_m.tolist(),
            "velocity_world_m_s": state.velocity_world_m_s.tolist(),
            "orientation_body_to_world_wxyz": state.orientation_body_to_world.tolist(),
            "angular_velocity_body_rad_s": state.angular_velocity_body_rad_s.tolist()}


def recovery_bound(state, config) -> dict:
    # Even simultaneous full thrust from ALL jets aligned world-up, dry mass,
    # no fuel limit and no rotation cost cannot exceed this optimistic bound.
    thrust = sum(t.max_thrust_N for t in config.thrusters if t.name not in config.failed_thrusters)
    a_max = max(0., thrust / config.dry_mass_kg - config.gravity_m_s2)
    h = max(0., state.position_world_m[2] - config.ground_z_m - min(config.body_half_extents_m))
    down = max(0., -float(state.velocity_world_m_s[2]))
    required = max(0., down * down - LANDING["vertical_speed_m_s"] ** 2)
    impossible = required > 2 * a_max * h + 1e-10
    return {"recovery_feasible": False if impossible else None,
            "reason": "optimistic_vertical_braking_bound_exceeded" if impossible else "not_proven_by_bound",
            "optimistic_max_braking_acceleration_m_s2": a_max,
            "optimistic_clearance_budget_m": float(h)}


def touchdown_metrics(before, after, config, contact, dt) -> dict:
    att = attitude(before)
    dv = after.velocity_world_m_s - before.velocity_world_m_s
    rotation = quat_to_matrix(before.orientation_body_to_world)
    seat = np.asarray(config.seat_position_body_m)
    seat_dv = dv + rotation @ np.cross(after.angular_velocity_body_rad_s - before.angular_velocity_body_rad_s, seat)
    duration = config.impact_stopping_time_assumption_s
    return {
        "time_s": after.time_s, "position_world_m": before.position_world_m.tolist(),
        "vertical_speed_m_s": abs(float(before.velocity_world_m_s[2])),
        "horizontal_speed_m_s": float(np.linalg.norm(before.velocity_world_m_s[:2])),
        "attitude_tilt_deg": att["tilt_deg"], "attitude_error_from_upright_deg": att["quaternion_error_deg"],
        "angular_rate_rad_s": att["angular_rate_rad_s"],
        "orientation_body_to_world_wxyz": before.orientation_body_to_world.tolist(),
        "impact_kinetic_energy_J": contact["kinetic_energy_before"]["total_J"],
        "normal_contact_impulse_Ns": contact["normal_contact_impulse_Ns"],
        "average_contact_force_approx_N": contact["normal_contact_impulse_Ns"] / duration,
        "impact_stopping_time_assumption_s": duration,
        "contact_deceleration_sampled_m_s2": abs(float(dv[2])) / dt,
        "CG_delta_v_world_m_s": dv.tolist(),
        "CG_impact_average_deceleration_g": float(np.linalg.norm(dv) / duration / STANDARD_G),
        "seat_delta_v_world_m_s": seat_dv.tolist(),
        "seat_impact_average_deceleration_g": float(np.linalg.norm(seat_dv) / duration / STANDARD_G),
        "contact_force_is_structural_stress": False,
    }


def touchdown_failures(metrics) -> list[str]:
    codes = {"vertical_speed_m_s": "vertical_touchdown_speed_exceeded_2m_s",
             "horizontal_speed_m_s": "horizontal_touchdown_speed_exceeded_1_5m_s",
             "attitude_tilt_deg": "landing_tilt_exceeded_15deg",
             "angular_rate_rad_s": "angular_rate_exceeded_0_55rad_s"}
    return [codes[key] for key, limit in LANDING.items() if metrics[key] > limit]


def sample(state, config, scenario_id, mode, dt, step=None, allocation=None, estimate=None, guidance=None) -> dict:
    """Endpoint state, with interval dynamics explicitly tagged at their evaluation time."""
    att = attitude(state)
    zero = [0., 0., 0.]
    bound_state = step.before if step else state
    bound = recovery_bound(bound_state, config)
    truth = {**state_fields(state), "mass_kg": mass(config, state),
             "propellant_remaining_kg": state.remaining_propellant_kg,
             "propellant_by_thruster_kg": state.propellant_by_thruster_kg.tolist(),
             "inertia_body_kg_m2": inertia_body(config, mass(config, state)).tolist(),
             "ground_clearance_m": lowest_corner_height(state, config),
             "kinetic_energy": kinetic_energy(state, mass(config, state), inertia_body(config, mass(config, state))),
             "contact": step.contact if step else None,
             "applied_thruster_force_body_N": step.force_body_N.tolist() if step else [0., 0., 0.],
             "applied_thruster_torque_body_Nm": step.torque_body_Nm.tolist() if step else [0., 0., 0.],
             "external_force_world_N": [0., 0., 0.],
             "acceleration_world_m_s2": step.acceleration_world_m_s2.tolist() if step else None,
             "angular_acceleration_body_rad_s2": step.angular_acceleration_body_rad_s2.tolist() if step else None,
             "dynamics_evaluation_time_s": step.before.time_s if step else None,
             "dynamics_evaluation_state": {**state_fields(step.before), "mass_kg": step.mass_before_kg,
                                           "inertia_body_kg_m2": step.inertia_before.tolist()} if step else None,
             "acceleration_convention": "free_flight_derivative_at_interval_start_excludes_contact_impulse"}
    events = []
    if allocation and not allocation.feasible:
        events.append({"code": "ACTUATOR_REQUEST_INFEASIBLE", "normalized_residual": allocation.normalized_residual})
    if allocation and not allocation.solver_success:
        events.append({"code": "ALLOCATION_SOLVER_NOT_CONVERGED", "message": allocation.solver_message})
    if bound["recovery_feasible"] is False:
        events.append({"code": "RECOVERY_INFEASIBLE", "reason": bound["reason"]})
    if step and step.contact:
        events.append({"code": "FIRST_GROUND_CONTACT"})
    clearance = lowest_corner_height(state, config)
    downward = max(0., -float(state.velocity_world_m_s[2]))
    row = {
        "schema_version": SCHEMA_VERSION, "scenario_id": scenario_id, "time_s": state.time_s,
        "interval_start_s": step.before.time_s if step else None,
        "interval_dt_s": dt if step else None,
        "physics_ground_truth": truth,
        "sensor_measurements": {"mode": "NOT_SIMULATED", "imu": None, "radar": None, "lidar": None},
        "estimated_state": {"mode": "PERFECT_STATE", "time_s": step.before.time_s if step else state.time_s,
                            **state_fields(estimate if estimate is not None else state),
                            "residuals": None, "uncertainty": None},
        "controller_mode": mode,
        "controller_diagnostics": guidance,
        "requested_force_body_N": allocation.requested_force_body_N.tolist() if allocation else zero,
        "requested_torque_body_Nm": allocation.requested_torque_body_Nm.tolist() if allocation else zero,
        "achieved_force_body_N": step.force_body_N.tolist() if step else zero,
        "achieved_torque_body_Nm": step.torque_body_Nm.tolist() if step else zero,
        "thruster_states": step.thrusters if step else [],
        "fuel_consumed_kg": float(state.initial_propellant_kg - state.remaining_propellant_kg),
        "fuel_remaining_kg": state.remaining_propellant_kg,
        "mass_flow_kg_s": step.mass_flow_kg_s if step else 0.,
        "ground_clearance_m": clearance, "attitude_error_deg": att["quaternion_error_deg"],
        "attitude": att, "vertical_speed_m_s": float(state.velocity_world_m_s[2]),
        "horizontal_speed_m_s": float(np.linalg.norm(state.velocity_world_m_s[:2])),
        "time_to_impact_estimate_s": max(0., clearance) / downward if downward > 1e-9 else None,
        "time_to_impact_model": "constant_CG_vertical_velocity_ignores_rotation_and_acceleration",
        "recovery_feasible": bound["recovery_feasible"], "recovery_bound": bound,
        "allocation_feasible": allocation.feasible if allocation else None,
        "allocation_normalized_residual": allocation.normalized_residual if allocation else None,
        "allocation_force_residual_N": (allocation.requested_force_body_N - step.force_body_N).tolist() if step else None,
        "allocation_torque_residual_Nm": (allocation.requested_torque_body_Nm - step.torque_body_Nm).tolist() if step else None,
        "seat_free_flight": step.seat if step else None,
        "diagnostic_events": events,
        # Compatibility with the original plotter/history consumers.
        **state_fields(state), "propellant_kg": state.remaining_propellant_kg,
    }
    return row


def safety_report(result, config, scenario) -> dict:
    rows = result.history
    # Last row is post-impact; control stability refers to the pre-impact flight state.
    measured = [r["attitude"] for r in rows]
    if result.touchdown_metrics:
        pre = rows[-1]["physics_ground_truth"]["contact"]
        measured[-1] = {**measured[-1],
                        "angular_rate_rad_s": float(np.linalg.norm(pre["pre_impact_angular_velocity_body_rad_s"]))}
    dwell, first_stable = 0., None
    outside = 0.
    for i, (r, att) in enumerate(zip(rows, measured)):
        interval = r["interval_dt_s"] or 0.
        # Count intervals only if BOTH endpoints meet the stability limits.
        good = att["quaternion_error_deg"] <= STABILITY["attitude_error_limit_deg"] and att["angular_rate_rad_s"] <= STABILITY["angular_rate_limit_rad_s"]
        previous_good = i > 0 and measured[i-1]["quaternion_error_deg"] <= STABILITY["attitude_error_limit_deg"] and measured[i-1]["angular_rate_rad_s"] <= STABILITY["angular_rate_limit_rad_s"]
        dwell = dwell + interval if good and previous_good else 0.
        if dwell + 1e-10 >= STABILITY["dwell_time_s"] and first_stable is None:
            first_stable = r["time_s"]
        if i > 0 and measured[i-1]["tilt_deg"] > STABILITY["outside_tilt_limit_deg"]:
            outside += interval
    stable = dwell + 1e-10 >= STABILITY["dwell_time_s"]
    errors = np.array([a["error_rotation_vector_world_deg"] for a in measured])
    crossings = []
    for component in errors.T:
        signs = np.sign(component[np.abs(component) > STABILITY["oscillation_deadband_deg"]])
        crossings.append(int(np.count_nonzero(np.diff(signs))))
    first_entry = next((i for i, a in enumerate(measured) if a["quaternion_error_deg"] <= 5), None)
    steps = rows[1:]
    seats = [np.asarray(r["seat_free_flight"]["specific_force_world_m_s2"]) for r in steps]
    jerks = [float(np.linalg.norm(seats[i] - seats[i-1]) / result.dt_s) for i in range(1, len(seats))]
    contact = rows[-1]["physics_ground_truth"]["contact"]
    td = result.touchdown_metrics
    high_impact = td is not None and td["CG_impact_average_deceleration_g"] > HIGH_IMPACT_AVERAGE_CG_DECEL_G
    statistics = []
    for i, thruster in enumerate(config.thrusters):
        records = [r["thruster_states"][i] for r in steps]
        statistics.append({"name": thruster.name, "failed": thruster.name in config.failed_thrusters,
                           "peak_applied_thrust_N": max((t["applied_thrust_N"] for t in records), default=0.),
                           "gas_consumed_kg": sum(t["gas_consumed_interval_kg"] for t in records),
                           "saturated_duration_s": sum(result.dt_s for t in records if t["saturated"]),
                           "active_duration_s": sum(result.dt_s for t in records if t["applied_thrust_N"] > 1e-6)})
    failures = result.failure_reason.split(";") if result.failure_reason else []
    if any(r["recovery_feasible"] is False for r in rows):
        failures.append("optimistic_vertical_braking_bound_exceeded")
    threshold_results = {key: {"measured": td[key] if td else None, "maximum": value,
                               "pass": td[key] <= value if td else None} for key, value in LANDING.items()}
    assumptions = asdict(config)
    assumptions["failed_thrusters"] = sorted(config.failed_thrusters)
    scenario_config = asdict(scenario)
    scenario_config["failed_thrusters"] = sorted(scenario.failed_thrusters)
    scenario_config["orientation_body_to_world"] = list(scenario.orientation_body_to_world)
    clearances = [r["ground_clearance_m"] for r in rows]
    predicted_time = rows[0]["time_to_impact_estimate_s"]
    return {
        "schema_version": SCHEMA_VERSION, "scenario_id": result.scenario,
        "guidance_mode": result.guidance_mode, "units": "SI; angles explicitly suffixed deg or rad; g=9.80665 m/s^2",
        "scenario_configuration": scenario_config, "vehicle_assumptions": assumptions,
        "physics_dt_s": result.dt_s, "elapsed_s": result.elapsed_s,
        "success": result.status == "landed", "status": result.status,
        "assessments": {"control_stability": "CONTROL_STABLE" if stable else "CONTROL_UNSTABLE",
                        "landing_criteria": "LANDING_PASS" if result.status == "landed" else "LANDING_FAIL",
                        "impact_load_assessment": "HIGH_IMPACT_LOAD" if high_impact else "NO_ILLUSTRATIVE_HIGH_LOAD_FLAG" if td else "UNKNOWN",
                        "structural_assessment": "STRUCTURAL_UNKNOWN", "occupant_safety_assessment": "NOT_VALIDATED"},
        "failure_reasons": failures,
        "threshold_results": threshold_results,
        "flight_stability": {"thresholds": STABILITY,
                             "time_to_upright_stabilization_s": first_stable,
                             "final_continuous_stable_duration_s": dwell,
                             "time_outside_tilt_limit_s": outside,
                             "peak_angular_rate_rad_s": max(a["angular_rate_rad_s"] for a in measured),
                             "peak_angular_acceleration_rad_s2": max((float(np.linalg.norm(r["physics_ground_truth"]["angular_acceleration_body_rad_s2"])) for r in steps), default=0.),
                             "error_axis_zero_crossings_xyz": crossings,
                             "max_attitude_error_after_first_entry_deg": max((a["quaternion_error_deg"] for a in measured[first_entry:]), default=None) if first_entry is not None else None,
                             "minimum_ground_clearance_m": min(clearances),
                             "max_penetration_m": max(0., -min(clearances)),
                             "final_pre_impact_attitude": measured[-1]},
        "fuel_summary": {"initial_kg": scenario.propellant_kg, "consumed_kg": result.fuel_spent_kg,
                         "remaining_kg": result.propellant_remaining_kg,
                         "mass_balance_error_kg": scenario.propellant_kg - result.fuel_spent_kg - result.propellant_remaining_kg},
        "thruster_statistics": statistics, "allocation_summary": result.allocation_summary,
        "touchdown": td, "contact_loads": contact,
        "touchdown_prediction": {"model": "constant_velocity_and_orientation_extrapolation_no_control_no_gravity",
                                 "initial_time_to_impact_estimate_s": rows[0]["time_to_impact_estimate_s"],
                                 "predicted_position_world_m": (np.asarray(scenario.position_world_m) + np.asarray(scenario.velocity_world_m_s) * predicted_time).tolist() if predicted_time is not None else None,
                                 "actual_position_world_m": td["position_world_m"] if td else None,
                                 "predicted_vertical_speed_m_s": abs(scenario.velocity_world_m_s[2]),
                                 "actual_time_s": td["time_s"] if td else None,
                                 "actual_vertical_speed_m_s": td["vertical_speed_m_s"] if td else None,
                                 "predicted_touchdown_orientation_wxyz": list(scenario.orientation_body_to_world),
                                 "predicted_horizontal_speed_m_s": float(np.linalg.norm(scenario.velocity_world_m_s[:2])),
                                 "actual_horizontal_speed_m_s": td["horizontal_speed_m_s"] if td else None,
                                 "actual_orientation_wxyz": td["orientation_body_to_world_wxyz"] if td else None},
        "occupant_load_proxy": {"seat_position_body_m": list(config.seat_position_body_m),
                                "peak_free_flight_seat_specific_force_g": max((r["seat_free_flight"]["specific_force_g"] for r in steps), default=0.),
                                "peak_free_flight_jerk_m_s3": max(jerks, default=None),
                                "average_free_flight_jerk_m_s3": float(np.mean(jerks)) if jerks else None,
                                "impact_seat_delta_v_world_m_s": td["seat_delta_v_world_m_s"] if td else None,
                                "impact_average_seat_deceleration_g": td["seat_impact_average_deceleration_g"] if td else None,
                                "impact_peak_jerk_m_s3": None,
                                "impact_peak_jerk_unavailable_reason": "Discontinuous rigid impulse; no resolved force-time curve.",
                                "occupant_safety_assessment": "NOT_VALIDATED"},
        "illustrative_high_impact_threshold": {"CG_impact_average_deceleration_g": HIGH_IMPACT_AVERAGE_CG_DECEL_G,
                                              "validated": False},
        "limitations": ["120 kg dry scaled prototype, not production Tesla Roadster specifications.",
                        "18 kg propellant and 1800 m/s effective exhaust velocity are hypothetical, not calibrated compressed air.",
                        "Rigid box corner impulses, not wheel, suspension, chassis stress, FEM or damage predictions.",
                        "0.05 s force averaging is an assumption distinct from physics dt; true peak force unavailable.",
                        "Perfect-state estimator; IMU, radar, LiDAR, fusion residuals and uncertainty not simulated.",
                        "Landing thresholds are demo criteria, not certified automotive standards or passenger safety.",
                        "No biomechanics, aerodynamics, tank thermodynamics, actuator lag, elastic structures or post-impact settling.",
                        "First-contact state is resolved at fixed-step boundary; penetration is measured without pose projection."]}
