"""Descent guidance, quaternion attitude control, and estimator adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .allocation import AllocationResult, allocate_wrench
from .model import (
    FlightConfig,
    FlightState,
    lowest_corner_height,
    quat_error_vector,
    quat_to_matrix,
)


class StateEstimator(Protocol):
    """Adapter seam for replacing perfect simulation truth with sensor estimates."""

    def estimate(self, truth: FlightState, dt: float) -> FlightState: ...


class PerfectStateEstimator:
    """Initial estimator implementation; returns an isolated copy of truth."""

    def estimate(self, truth: FlightState, dt: float) -> FlightState:
        del dt
        return truth.copy()


@dataclass
class FlightController:
    config: FlightConfig
    estimator: StateEstimator | None = None
    target_position_xy_m: tuple[float, float] = (0.0, 0.0)
    target_orientation_body_to_world: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    max_descent_speed_m_s: float = 7.0
    predictive_hold_s: float = 2.5
    _previous_commands: tuple | None = None
    last_estimated_state: FlightState | None = None
    last_guidance: dict | None = None

    def _max_upward_accel(self, state: FlightState, dt: float) -> float:
        """Planning envelope reserves 55% of ideal vertical acceleration.

        Independent per-pod maxima cannot be achieved while also satisfying
        lateral force and attitude demands. This margin is guidance only;
        actual bounded actuator force alone drives the physics.
        """
        rotation = quat_to_matrix(state.orientation_body_to_world)
        vertical_force = 0.0
        for i, thruster in enumerate(self.config.thrusters):
            if thruster.name in self.config.failed_thrusters or state.propellant_by_thruster_kg[i] <= 0:
                continue
            max_fuel_limited = state.propellant_by_thruster_kg[i] * self.config.effective_exhaust_velocity_m_s / max(dt, 1e-3)
            limit = min(thruster.max_thrust_N, max_fuel_limited)
            if not thruster.gimbaled:
                vertical_cos = float((rotation @ np.asarray(thruster.force_axis_body))[2])
                vertical_force += max(0.0, vertical_cos) * limit
                continue
            yaw, pitch = np.meshgrid(
                np.linspace(-thruster.gimbal_yaw_limit_rad, thruster.gimbal_yaw_limit_rad, 17),
                np.linspace(-thruster.gimbal_pitch_limit_rad, thruster.gimbal_pitch_limit_rad, 9),
            )
            dirs = np.stack(
                [-np.sin(yaw) * np.cos(pitch), np.cos(yaw) * np.cos(pitch), np.sin(pitch)], axis=-1
            ).reshape(-1, 3)
            best = float(np.max(dirs @ rotation[2, :]))
            vertical_force += max(0.0, best) * limit
        mass = self.config.dry_mass_kg + state.remaining_propellant_kg
        return max(0.0, 0.45 * (vertical_force / mass - self.config.gravity_m_s2))

    def command(
        self,
        truth: FlightState,
        dt: float,
        guidance_mode: str = "predictive",
        elapsed_since_start_s: float = 0.0,
    ) -> AllocationResult:
        if self.estimator is None:
            self.estimator = PerfectStateEstimator()
        state = self.estimator.estimate(truth, dt)
        self.last_estimated_state = state.copy()
        clearance = max(0.0, lowest_corner_height(state, self.config))
        max_up_accel = self._max_upward_accel(state, dt)
        fuel_fraction = state.remaining_propellant_kg / max(self.config.initial_propellant_kg, 1e-9)
        # Lower remaining fuel reduces the planned descent speed and increases
        # braking distance. Propellant is finite in the simulator itself.
        braking_accel = max(0.25, min(4.5, max_up_accel * (0.72 + 0.20 * np.sqrt(np.clip(fuel_fraction, 0.0, 1.0)))))
        target_down_speed = min(self.max_descent_speed_m_s, float(np.sqrt(2.0 * braking_accel * clearance)))
        target_vz = -target_down_speed
        early_hovering = guidance_mode == "early_hover" and elapsed_since_start_s < self.predictive_hold_s and clearance > 12.0
        if early_hovering:
            target_vz = 0.0

        speed_error = target_vz - state.velocity_world_m_s[2]
        if early_hovering:
            desired_az = float(np.clip(2.8 * speed_error, -7.6, max_up_accel))
        elif target_down_speed < self.max_descent_speed_m_s - 1e-6:
            # Feed forward the deceleration used to construct the stopping
            # speed envelope, then correct for speed error. This starts braking
            # at the envelope instead of waiting until a large velocity error.
            desired_az = float(np.clip(0.84 * braking_accel + 2.4 * speed_error, -7.6, max_up_accel))
        else:
            desired_az = float(np.clip(2.8 * speed_error + 0.2, -7.6, max_up_accel))

        xy_error = np.asarray(self.target_position_xy_m) - state.position_world_m[:2]
        desired_xy_accel = 0.65 * xy_error - 1.25 * state.velocity_world_m_s[:2]
        xy_norm = float(np.linalg.norm(desired_xy_accel))
        if xy_norm > 2.1:
            desired_xy_accel *= 2.1 / xy_norm
        desired_world_accel = np.r_[desired_xy_accel, desired_az]
        desired_world_force = (self.config.dry_mass_kg + state.remaining_propellant_kg) * (
            desired_world_accel - np.array([0.0, 0.0, -self.config.gravity_m_s2])
        )
        rotation = quat_to_matrix(state.orientation_body_to_world)
        desired_body_force = rotation.T @ desired_world_force

        attitude_error_world = quat_error_vector(self.target_orientation_body_to_world, state.orientation_body_to_world)
        attitude_error = rotation.T @ attitude_error_world
        inertia = np.diag(self.config.inertia_dry_kg_m2) * (
            (self.config.dry_mass_kg + state.remaining_propellant_kg) / self.config.dry_mass_kg
        )
        desired_alpha = 7.0 * attitude_error - 4.2 * state.angular_velocity_body_rad_s
        desired_alpha = np.clip(desired_alpha, -5.0, 5.0)
        omega = state.angular_velocity_body_rad_s
        desired_body_torque = inertia @ desired_alpha + np.cross(omega, inertia @ omega)
        self.last_guidance = {
            "evaluation_time_s": state.time_s,
            "vertical_phase": "EARLY_HOVER" if early_hovering else "BRAKING_ENVELOPE" if target_down_speed < self.max_descent_speed_m_s - 1e-6 else "DESCENT_SPEED_LIMIT",
            "target_vertical_velocity_m_s": float(target_vz),
            "planned_braking_acceleration_m_s2": float(braking_accel),
            "estimated_max_upward_acceleration_m_s2": float(max_up_accel),
            "desired_acceleration_world_m_s2": desired_world_accel.tolist(),
            "desired_angular_acceleration_body_rad_s2": desired_alpha.tolist(),
            "attitude_error_body_rad": attitude_error.tolist(),
        }
        result = allocate_wrench(
            desired_body_force,
            desired_body_torque,
            state,
            self.config,
            dt,
            self._previous_commands,
        )
        self._previous_commands = result.commands
        return result
