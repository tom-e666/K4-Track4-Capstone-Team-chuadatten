"""Physical models and quaternion helpers for the flight recovery prototype.

Coordinates use SI units and a right-handed Z-up world frame. Quaternions are
stored as [w, x, y, z] and rotate body-frame vectors into world coordinates.
Thruster directions below describe force on the vehicle (opposite the nozzle
exhaust direction).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

Array = np.ndarray


def _vec3(value: Iterable[float]) -> Array:
    result = np.asarray(tuple(value), dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError("expected a finite 3-vector")
    return result


def quat_normalize(q: Iterable[float]) -> Array:
    q = np.asarray(tuple(q), dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be a finite [w, x, y, z] vector")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("zero quaternion has no orientation")
    q = q / norm
    return q if q[0] >= 0 else -q


def quat_multiply(a: Iterable[float], b: Iterable[float]) -> Array:
    aw, ax, ay, az = np.asarray(tuple(a), dtype=float)
    bw, bx, by, bz = np.asarray(tuple(b), dtype=float)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def quat_conjugate(q: Iterable[float]) -> Array:
    w, x, y, z = np.asarray(tuple(q), dtype=float)
    return np.array([w, -x, -y, -z], dtype=float)


def quat_from_axis_angle(axis: Iterable[float], angle_rad: float) -> Array:
    axis = _vec3(axis)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        if abs(angle_rad) < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0])
        raise ValueError("a nonzero axis is required for a nonzero angle")
    half = 0.5 * float(angle_rad)
    return quat_normalize(np.r_[np.cos(half), axis / norm * np.sin(half)])


def quat_to_matrix(q: Iterable[float]) -> Array:
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_error_vector(target: Iterable[float], actual: Iterable[float]) -> Array:
    """Shortest-arc WORLD rotation vector that takes actual to target."""
    error = quat_normalize(quat_multiply(target, quat_conjugate(actual)))
    if error[0] < 0:
        error = -error
    vector = error[1:]
    sine_half = float(np.linalg.norm(vector))
    if sine_half < 1e-10:
        return 2.0 * vector
    angle = 2.0 * np.arctan2(sine_half, max(0.0, float(error[0])))
    return vector * (angle / sine_half)


def quat_integrate_body_rate(q: Iterable[float], omega_body: Iterable[float], dt: float) -> Array:
    """Exponential increment: q_next = q * exp([0, omega_body] * dt/2)."""
    q_arr = quat_normalize(q)
    omega = _vec3(omega_body)
    rate = float(np.linalg.norm(omega))
    if rate < 1e-12:
        return q_arr
    dq = quat_from_axis_angle(omega / rate, rate * float(dt))
    return quat_normalize(quat_multiply(q_arr, dq))


@dataclass(frozen=True)
class Thruster:
    name: str
    position_body_m: tuple[float, float, float]
    force_axis_body: tuple[float, float, float]
    max_thrust_N: float
    gimbaled: bool = False
    gimbal_yaw_limit_rad: float = 0.0
    gimbal_pitch_limit_rad: float = 0.0

    def __post_init__(self) -> None:
        p = _vec3(self.position_body_m)
        d = _vec3(self.force_axis_body)
        n = float(np.linalg.norm(d))
        if n < 1e-12 or self.max_thrust_N <= 0:
            raise ValueError("thruster requires a direction and positive thrust limit")
        object.__setattr__(self, "position_body_m", tuple(p.tolist()))
        object.__setattr__(self, "force_axis_body", tuple((d / n).tolist()))
        if not self.gimbaled and (self.gimbal_yaw_limit_rad or self.gimbal_pitch_limit_rad):
            raise ValueError("fixed thruster cannot have gimbal limits")


def default_thrusters() -> tuple[Thruster, ...]:
    """Ten actuators: four roof lift units, four underside braking units, two side pods."""
    result: list[Thruster] = []
    for i, (x, y) in enumerate(((-0.48, -0.42), (-0.48, 0.42), (0.48, -0.42), (0.48, 0.42)), 1):
        result.append(Thruster(f"roof_{i}", (x, y, 0.42), (0, 0, -1), 520.0))
    for i, (x, y) in enumerate(((-0.48, -0.42), (-0.48, 0.42), (0.48, -0.42), (0.48, 0.42)), 1):
        result.append(Thruster(f"underside_{i}", (x, y, -0.38), (0, 0, 1), 360.0))
    # Exhaust points roughly toward -Y; force is +Y. A wide yaw sweep provides
    # lateral authority in either direction, while pitch enables vertical force.
    result.extend(
        [
            Thruster("side_left", (-0.52, 0.0, 0.0), (0, 1, 0), 180.0, True, np.deg2rad(150), np.deg2rad(35)),
            Thruster("side_right", (0.52, 0.0, 0.0), (0, 1, 0), 180.0, True, np.deg2rad(150), np.deg2rad(35)),
        ]
    )
    return tuple(result)


@dataclass
class FlightConfig:
    thrusters: tuple[Thruster, ...] = field(default_factory=default_thrusters)
    dry_mass_kg: float = 120.0
    initial_propellant_kg: float = 18.0
    effective_exhaust_velocity_m_s: float = 1800.0
    gravity_m_s2: float = 9.81
    inertia_dry_kg_m2: tuple[float, float, float] = (55.0, 65.0, 80.0)
    body_half_extents_m: tuple[float, float, float] = (0.62, 0.56, 0.40)
    ground_z_m: float = 0.0
    contact_restitution: float = 0.04
    contact_friction: float = 0.45
    impact_stopping_time_assumption_s: float = 0.05
    seat_position_body_m: tuple[float, float, float] = (0.15, 0.20, 0.15)
    failed_thrusters: frozenset[str] = frozenset()

    def __post_init__(self) -> None:
        if len(self.thrusters) != 10:
            raise ValueError("the reference vehicle model requires exactly 10 thrusters")
        if len({t.name for t in self.thrusters}) != 10:
            raise ValueError("thruster names must be unique")
        if self.dry_mass_kg <= 0 or self.initial_propellant_kg < 0:
            raise ValueError("mass must be positive and propellant nonnegative")
        if self.effective_exhaust_velocity_m_s <= 0 or self.gravity_m_s2 < 0:
            raise ValueError("exhaust velocity must be positive and gravity nonnegative")
        if self.impact_stopping_time_assumption_s <= 0:
            raise ValueError("impact stopping-time assumption must be positive")
        if not self.failed_thrusters.issubset({t.name for t in self.thrusters}):
            raise ValueError("failed_thrusters contains an unknown thruster")
        values = [self.dry_mass_kg, self.initial_propellant_kg,
                  self.effective_exhaust_velocity_m_s, self.gravity_m_s2,
                  self.ground_z_m, self.contact_restitution, self.contact_friction,
                  self.impact_stopping_time_assumption_s]
        if not np.all(np.isfinite(values)):
            raise ValueError("configuration scalars must be finite")
        if np.any(_vec3(self.inertia_dry_kg_m2) <= 0) or np.any(_vec3(self.body_half_extents_m) <= 0):
            raise ValueError("inertia and half extents must be positive")
        _vec3(self.seat_position_body_m)
        if not 0 <= self.contact_restitution <= 1 or self.contact_friction < 0:
            raise ValueError("invalid restitution or friction")


@dataclass
class FlightState:
    position_world_m: Array
    velocity_world_m_s: Array
    orientation_body_to_world: Array
    angular_velocity_body_rad_s: Array
    propellant_by_thruster_kg: Array
    time_s: float = 0.0
    initial_propellant_kg: float | None = None

    def __post_init__(self) -> None:
        self.position_world_m = _vec3(self.position_world_m)
        self.velocity_world_m_s = _vec3(self.velocity_world_m_s)
        self.orientation_body_to_world = quat_normalize(self.orientation_body_to_world)
        self.angular_velocity_body_rad_s = _vec3(self.angular_velocity_body_rad_s)
        fuel = np.asarray(self.propellant_by_thruster_kg, dtype=float)
        if fuel.ndim != 1 or not np.all(np.isfinite(fuel)) or np.any(fuel < -1e-12):
            raise ValueError("propellant amounts must be a nonnegative finite vector")
        self.propellant_by_thruster_kg = np.maximum(fuel, 0.0)
        if self.initial_propellant_kg is None:
            self.initial_propellant_kg = float(np.sum(self.propellant_by_thruster_kg))

    def copy(self) -> "FlightState":
        return FlightState(
            self.position_world_m.copy(),
            self.velocity_world_m_s.copy(),
            self.orientation_body_to_world.copy(),
            self.angular_velocity_body_rad_s.copy(),
            self.propellant_by_thruster_kg.copy(),
            self.time_s,
            self.initial_propellant_kg,
        )

    @property
    def remaining_propellant_kg(self) -> float:
        return float(np.sum(self.propellant_by_thruster_kg))


def make_initial_state(
    config: FlightConfig,
    position_world_m: Iterable[float] = (0.0, 0.0, 30.0),
    velocity_world_m_s: Iterable[float] = (0.0, 0.0, -5.0),
    orientation_body_to_world: Iterable[float] = (1.0, 0.0, 0.0, 0.0),
    angular_velocity_body_rad_s: Iterable[float] = (0.0, 0.0, 0.0),
) -> FlightState:
    weights = np.array([t.max_thrust_N for t in config.thrusters], dtype=float)
    if config.initial_propellant_kg == 0:
        fuel = np.zeros(len(config.thrusters))
    else:
        fuel = weights / np.sum(weights) * config.initial_propellant_kg
    return FlightState(
        _vec3(position_world_m),
        _vec3(velocity_world_m_s),
        quat_normalize(orientation_body_to_world),
        _vec3(angular_velocity_body_rad_s),
        fuel,
        initial_propellant_kg=config.initial_propellant_kg,
    )


@dataclass(frozen=True)
class ThrusterCommand:
    thrust_N: float
    gimbal_yaw_rad: float = 0.0
    gimbal_pitch_rad: float = 0.0

    def __post_init__(self) -> None:
        if not np.all(np.isfinite([self.thrust_N, self.gimbal_yaw_rad, self.gimbal_pitch_rad])):
            raise ValueError("actuator command must be finite")


def thruster_force_direction(thruster: Thruster, yaw_rad: float = 0.0, pitch_rad: float = 0.0) -> Array:
    """Return the unit force direction, after clamping gimbal angles to limits."""
    if not thruster.gimbaled:
        return np.asarray(thruster.force_axis_body, dtype=float)
    yaw = float(np.clip(yaw_rad, -thruster.gimbal_yaw_limit_rad, thruster.gimbal_yaw_limit_rad))
    pitch = float(np.clip(pitch_rad, -thruster.gimbal_pitch_limit_rad, thruster.gimbal_pitch_limit_rad))
    c_y, s_y = np.cos(yaw), np.sin(yaw)
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    # Gimbal angles are defined about body Z then the rotated thruster's X axis.
    d = np.array([-s_y * c_p, c_y * c_p, s_p], dtype=float)
    return d / np.linalg.norm(d)


def body_corners(half_extents_m: Iterable[float]) -> Array:
    hx, hy, hz = _vec3(half_extents_m)
    return np.array(
        [(sx * hx, sy * hy, sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
        dtype=float,
    )


def lowest_corner_height(state: FlightState, config: FlightConfig) -> float:
    rotation = quat_to_matrix(state.orientation_body_to_world)
    corners = state.position_world_m[None, :] + body_corners(config.body_half_extents_m) @ rotation.T
    return float(np.min(corners[:, 2]) - config.ground_z_m)
