"""Bounded nonlinear allocation from a requested body wrench to ten thrusters."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .model import (
    FlightConfig,
    FlightState,
    ThrusterCommand,
    thruster_force_direction,
)


@dataclass(frozen=True)
class AllocationResult:
    commands: tuple[ThrusterCommand, ...]
    requested_force_body_N: np.ndarray
    requested_torque_body_Nm: np.ndarray
    achieved_force_body_N: np.ndarray
    achieved_torque_body_Nm: np.ndarray
    force_residual_N: np.ndarray
    torque_residual_Nm: np.ndarray
    feasible: bool
    normalized_residual: float
    saturated_thrusters: tuple[str, ...]
    solver_success: bool
    solver_message: str


def wrench_for_commands(config: FlightConfig, commands: tuple[ThrusterCommand, ...]) -> tuple[np.ndarray, np.ndarray]:
    if len(commands) != len(config.thrusters):
        raise ValueError("one command is required for each configured thruster")
    force = np.zeros(3)
    torque = np.zeros(3)
    for thruster, command in zip(config.thrusters, commands):
        magnitude = float(np.clip(command.thrust_N, 0.0, thruster.max_thrust_N))
        direction = thruster_force_direction(thruster, command.gimbal_yaw_rad, command.gimbal_pitch_rad)
        actuator_force = magnitude * direction
        force += actuator_force
        torque += np.cross(np.asarray(thruster.position_body_m), actuator_force)
    return force, torque


def allocate_wrench(
    requested_force_body_N: np.ndarray,
    requested_torque_body_Nm: np.ndarray,
    state: FlightState,
    config: FlightConfig,
    dt: float,
    previous: tuple[ThrusterCommand, ...] | None = None,
) -> AllocationResult:
    """Fit a six-axis wrench while honoring thrust, gimbal, fuel, and failure limits.

    Only actuators with available thrust receive optimizer variables. Fixed jets
    have no artificial angle bounds, and each active variable is normalized to
    a unit interval before optimization. This avoids the singular scaling from
    zero-width or near-zero bounds in trust-region least squares.
    """
    target_f = np.asarray(requested_force_body_N, dtype=float)
    target_t = np.asarray(requested_torque_body_Nm, dtype=float)
    if target_f.shape != (3,) or target_t.shape != (3,) or not np.all(np.isfinite(np.r_[target_f, target_t])):
        raise ValueError("requested wrench must contain two finite 3-vectors")
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError("dt must be finite and positive")

    n = len(config.thrusters)
    limits = np.zeros(n)
    for i, thruster in enumerate(config.thrusters):
        fuel_limit = state.propellant_by_thruster_kg[i] * config.effective_exhaust_velocity_m_s / dt
        limits[i] = min(thruster.max_thrust_N, fuel_limit)
        if thruster.name in config.failed_thrusters:
            limits[i] = 0.0

    # Variable kinds are thrust, yaw, or pitch. Angles are normalized by their
    # physical limits, so every free variable has an order-one scale.
    specs: list[tuple[int, str]] = []
    var_index: dict[tuple[int, str], int] = {}
    for i, thruster in enumerate(config.thrusters):
        if limits[i] <= 0.0:
            continue
        for kind in ("thrust", "yaw", "pitch") if thruster.gimbaled else ("thrust",):
            var_index[(i, kind)] = len(specs)
            specs.append((i, kind))

    def commands_for(x: np.ndarray) -> tuple[ThrusterCommand, ...]:
        thrust = np.zeros(n)
        yaw = np.zeros(n)
        pitch = np.zeros(n)
        for i, kind in specs:
            value = float(x[var_index[(i, kind)]])
            if kind == "thrust":
                thrust[i] = value * limits[i]
            elif kind == "yaw":
                yaw[i] = value * config.thrusters[i].gimbal_yaw_limit_rad
            else:
                pitch[i] = value * config.thrusters[i].gimbal_pitch_limit_rad
        return tuple(ThrusterCommand(float(thrust[i]), float(yaw[i]), float(pitch[i])) for i in range(n))

    x0 = np.zeros(len(specs))
    lo = np.empty(len(specs))
    hi = np.empty(len(specs))
    penalty = np.empty(len(specs))
    # Seed a gimbaled jet in the requested direction with nonzero thrust. At
    # zero thrust its angle derivatives vanish and a local solver cannot find
    # the vertical component that the pods can physically provide.
    target_norm = float(np.linalg.norm(target_f))
    for i, kind in specs:
        j = var_index[(i, kind)]
        thruster = config.thrusters[i]
        lo[j], hi[j] = (0.0, 1.0) if kind == "thrust" else (-1.0, 1.0)
        if kind == "thrust":
            penalty[j] = 0.012
            if previous is not None and i < len(previous):
                x0[j] = np.clip(previous[i].thrust_N / max(limits[i], 1e-12), 0.0, 1.0)
            elif thruster.gimbaled:
                x0[j] = np.clip(max(0.08 * limits[i], target_norm * 0.5), 0.0, limits[i]) / limits[i]
            else:
                same_axis = sum(
                    1 for k, other in enumerate(config.thrusters)
                    if limits[k] > 0 and not other.gimbaled
                    and np.allclose(other.force_axis_body, thruster.force_axis_body)
                )
                projected = max(0.0, float(target_f @ np.asarray(thruster.force_axis_body)))
                seed = projected / max(1, same_axis)
                x0[j] = np.clip(max(seed, 0.03 * limits[i]), 0.0, limits[i]) / limits[i]
        else:
            penalty[j] = 0.002
            if previous is not None and i < len(previous):
                angle = previous[i].gimbal_yaw_rad if kind == "yaw" else previous[i].gimbal_pitch_rad
            elif kind == "yaw":
                angle = np.arctan2(-target_f[0], target_f[1])
            else:
                angle = np.arctan2(target_f[2], np.hypot(target_f[0], target_f[1]))
            angle_limit = thruster.gimbal_yaw_limit_rad if kind == "yaw" else thruster.gimbal_pitch_limit_rad
            x0[j] = np.clip(angle / angle_limit, -1.0, 1.0) if angle_limit > 0 else 0.0

    torque_scale = 350.0
    # The contact-safe world-up force component matters most during descent.
    # Weight it by the vehicle's current orientation, while retaining a common
    # scale for the two lateral force components.
    from .model import quat_to_matrix
    vertical_body = quat_to_matrix(state.orientation_body_to_world).T @ np.array([0.0, 0.0, 1.0])
    vertical_weight = 1.5
    force_weight = (np.eye(3) + vertical_weight * np.outer(vertical_body, vertical_body)) / 1000.0

    def residual(x: np.ndarray) -> np.ndarray:
        achieved_f, achieved_t = wrench_for_commands(config, commands_for(x))
        return np.r_[force_weight @ (achieved_f - target_f),
                     (achieved_t - target_t) / torque_scale,
                     penalty * x]

    def residual_jacobian(x: np.ndarray) -> np.ndarray:
        commands = commands_for(x)
        jac = np.zeros((6 + len(specs), len(specs)))
        for i, thruster in enumerate(config.thrusters):
            if limits[i] <= 0.0:
                continue
            command = commands[i]
            direction = thruster_force_direction(thruster, command.gimbal_yaw_rad, command.gimbal_pitch_rad)
            position = np.asarray(thruster.position_body_m)
            derivatives: dict[str, np.ndarray] = {"thrust": limits[i] * direction}
            if thruster.gimbaled:
                yaw, pitch = command.gimbal_yaw_rad, command.gimbal_pitch_rad
                cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
                derivatives["yaw"] = command.thrust_N * thruster.gimbal_yaw_limit_rad * np.array(
                    [-cy * cp, -sy * cp, 0.0]
                )
                derivatives["pitch"] = command.thrust_N * thruster.gimbal_pitch_limit_rad * np.array(
                    [sy * sp, -cy * sp, cp]
                )
            for kind, derivative in derivatives.items():
                key = (i, kind)
                if key not in var_index:
                    continue
                j = var_index[key]
                jac[:3, j] = force_weight @ derivative
                jac[3:6, j] = np.cross(position, derivative) / torque_scale
                jac[6 + j, j] = penalty[j]
        return jac

    if specs:
        fit = least_squares(
            residual,
            np.clip(x0, lo, hi),
            bounds=(lo, hi),
            jac=residual_jacobian,
            method="trf",
            x_scale=1.0,
            max_nfev=80,
            ftol=1e-5,
            xtol=1e-5,
            gtol=1e-5,
        )
        commands = commands_for(fit.x)
        solver_success, solver_message = bool(fit.success), str(fit.message)
    else:
        commands = tuple(ThrusterCommand(0.0) for _ in config.thrusters)
        solver_success, solver_message = True, "no thrusters have available propellant"

    achieved_f, achieved_t = wrench_for_commands(config, commands)
    f_error, t_error = target_f - achieved_f, target_t - achieved_t
    normalized = float(np.sqrt(np.sum((f_error / 1000.0) ** 2) + np.sum((t_error / torque_scale) ** 2)))
    saturated = []
    for i, thruster in enumerate(config.thrusters):
        if limits[i] > 0 and commands[i].thrust_N >= limits[i] - max(0.5, limits[i] * 0.01):
            saturated.append(thruster.name)
        elif limits[i] == 0 and thruster.name in config.failed_thrusters:
            saturated.append(thruster.name)
    return AllocationResult(
        commands,
        target_f.copy(),
        target_t.copy(),
        achieved_f,
        achieved_t,
        f_error,
        t_error,
        normalized <= 0.12,
        normalized,
        tuple(saturated),
        solver_success,
        solver_message,
    )
