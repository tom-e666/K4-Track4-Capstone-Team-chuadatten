"""Fixed-step 6-DoF simulation, impulse ground contact, and touchdown metrics."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from .controller import FlightController
from .model import FlightConfig, FlightState, make_initial_state, lowest_corner_height
from .physics import advance
from .telemetry import sample, safety_report, touchdown_metrics, touchdown_failures


@dataclass(frozen=True)
class Scenario:
    name: str
    position_world_m: tuple[float, float, float]
    velocity_world_m_s: tuple[float, float, float]
    orientation_body_to_world: tuple[float, float, float, float] = (1, 0, 0, 0)
    angular_velocity_body_rad_s: tuple[float, float, float] = (0, 0, 0)
    failed_thrusters: frozenset[str] = frozenset()
    propellant_kg: float = 18.0
    timeout_s: float = 28.0
    description: str = ""


@dataclass
class SimulationResult:
    scenario: str
    guidance_mode: str
    status: str
    failure_reason: str | None
    final_state: FlightState
    touchdown_metrics: dict[str, Any] | None
    fuel_spent_kg: float
    propellant_remaining_kg: float
    peak_vertical_accel_m_s2: float
    elapsed_s: float
    dt_s: float
    allocation_summary: dict[str, Any]
    history: list[dict[str, Any]] = field(default_factory=list)

    safety_report: dict[str, Any] = field(default_factory=dict)

    def to_dict(self, include_history: bool = True) -> dict[str, Any]:
        state = self.final_state
        out = {
            "scenario": self.scenario,
            "guidance_mode": self.guidance_mode,
            "status": self.status,
            "failure_reason": self.failure_reason,
            "elapsed_s": self.elapsed_s,
            "dt_s": self.dt_s,
            "fuel_spent_kg": self.fuel_spent_kg,
            "propellant_remaining_kg": self.propellant_remaining_kg,
            "peak_vertical_accel_m_s2": self.peak_vertical_accel_m_s2,
            "touchdown_metrics": self.touchdown_metrics,
            "final_state": {
                "position_world_m": state.position_world_m.tolist(),
                "velocity_world_m_s": state.velocity_world_m_s.tolist(),
                "orientation_body_to_world_wxyz": state.orientation_body_to_world.tolist(),
                "angular_velocity_body_rad_s": state.angular_velocity_body_rad_s.tolist(),
                "remaining_propellant_by_thruster_kg": state.propellant_by_thruster_kg.tolist(),
                "sim_time_s": state.time_s,
            },
            "allocation_summary": self.allocation_summary,
            "safety_report": self.safety_report,
        }
        if include_history:
            out["history"] = self.history
        return out


def simulate(scenario: Scenario, guidance_mode: str = "predictive", dt: float = 0.02, base_config: FlightConfig | None = None) -> SimulationResult:
    """Fixed steps through first resolved contact; never force a successful pose."""
    if not np.isfinite(dt) or not 0 < dt <= 0.1:
        raise ValueError("fixed dt must be finite and in (0, 0.1] seconds")
    if guidance_mode not in ("predictive", "early_hover"):
        raise ValueError("guidance_mode must be 'predictive' or 'early_hover'")
    if not np.isfinite(scenario.timeout_s) or scenario.timeout_s <= 0:
        raise ValueError("scenario timeout must be finite and positive")
    config = replace(base_config or FlightConfig(), initial_propellant_kg=scenario.propellant_kg,
                     failed_thrusters=frozenset(scenario.failed_thrusters))
    state = make_initial_state(config, scenario.position_world_m, scenario.velocity_world_m_s,
                               scenario.orientation_body_to_world, scenario.angular_velocity_body_rad_s)
    if lowest_corner_height(state, config) <= 0:
        raise ValueError("initial scenario must start strictly above ground")
    controller = FlightController(config)
    history = [sample(state, config, scenario.name, guidance_mode, dt)]
    touchdown = None
    status, reason = "failed", "simulation_timeout"
    peak_vertical, max_residual, infeasible_steps = 0., 0., 0
    saturated = set()
    steps = int(np.ceil(scenario.timeout_s / dt))
    for index in range(steps):
        allocation = controller.command(state, dt, guidance_mode, state.time_s)
        step = advance(state, config, allocation.commands, dt)
        state.time_s = (index + 1) * dt  # prevent accumulated timestamp roundoff
        step.before_contact.time_s = state.time_s
        max_residual = max(max_residual, allocation.normalized_residual)
        infeasible_steps += int(not allocation.feasible)
        saturated.update(t["name"] for t in step.thrusters if t["saturated"])
        peak_vertical = max(peak_vertical, abs(float(step.acceleration_world_m_s2[2])))
        history.append(sample(state, config, scenario.name, guidance_mode, dt, step,
                              allocation, controller.last_estimated_state, controller.last_guidance))
        if step.contact and step.contact["normal_contact_impulse_Ns"] > 0:
            touchdown = touchdown_metrics(step.before_contact, state, config, step.contact, dt)
            failures = touchdown_failures(touchdown)
            status, reason = ("failed", ";".join(failures)) if failures else ("landed", None)
            peak_vertical = max(peak_vertical, touchdown["contact_deceleration_sampled_m_s2"])
            break
    if touchdown is None and state.remaining_propellant_kg < 1e-8:
        reason = "simulation_timeout_after_propellant_exhaustion"
    result = SimulationResult(
        scenario.name, guidance_mode, status, reason, state, touchdown,
        scenario.propellant_kg - state.remaining_propellant_kg, state.remaining_propellant_kg,
        peak_vertical, state.time_s, dt,
        {"max_normalized_wrench_residual": max_residual,
         "steps_over_12_percent_wrench_residual": infeasible_steps,
         "saturated_thrusters_seen": sorted(saturated),
         "effective_exhaust_velocity_m_s": config.effective_exhaust_velocity_m_s}, history)
    result.safety_report = safety_report(result, config, scenario)
    return result


def default_scenarios() -> tuple[Scenario, ...]:
    """Five repeatable cases spanning nominal, upset and impossible recoveries."""
    from .model import quat_from_axis_angle

    return (
        Scenario(
            "comparison_nominal",
            (0.0, 0.0, 32.0),
            (1.6, -0.9, -7.5),
            quat_from_axis_angle((1, 0.2, 0), np.deg2rad(12)),
            (0.08, -0.04, 0.03),
            description="Offset descent; also used for identical-input guidance comparison.",
        ),
        Scenario(
            "upside_down_recovery",
            (0.0, 0.0, 38.0),
            (-0.8, 0.6, -5.0),
            quat_from_axis_angle((1, 0, 0), np.pi),
            (0.0, 0.0, 0.0),
            description="Upside-down attitude recovery with usable altitude.",
        ),
        Scenario(
            "sideways_spin_recovery",
            (1.0, -0.7, 30.0),
            (-1.0, 0.8, -6.0),
            quat_from_axis_angle((0, 1, 0), np.deg2rad(78)),
            (0.25, -0.15, 0.2),
            description="Large tilt and nonzero angular rate.",
        ),
        Scenario(
            "too_low_fast_impossible",
            (0.0, 0.0, 1.8),
            (0.0, 0.0, -18.0),
            description="Too little clearance for the available braking acceleration.",
        ),
        Scenario(
            "failed_roof_thruster",
            (-0.5, 0.6, 34.0),
            (0.8, 1.1, -7.0),
            quat_from_axis_angle((0.2, 1, 0), np.deg2rad(20)),
            (0.1, 0.0, -0.12),
            failed_thrusters=frozenset({"roof_1"}),
            description="One roof lift thruster is disabled.",
        ),
    )
