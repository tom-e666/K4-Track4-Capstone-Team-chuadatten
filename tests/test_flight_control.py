import numpy as np

from flight_control.allocation import allocate_wrench, wrench_for_commands
from flight_control.controller import FlightController
from flight_control.model import (
    FlightConfig,
    ThrusterCommand,
    default_thrusters,
    make_initial_state,
    quat_from_axis_angle,
    quat_integrate_body_rate,
    quat_to_matrix,
    thruster_force_direction,
)
from flight_control.simulation import default_scenarios, simulate


def test_quaternion_rotation_integration_and_normalization():
    q = quat_from_axis_angle((0, 0, 1), np.pi / 2)
    rotated = quat_to_matrix(q) @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(rotated, [0.0, 1.0, 0.0], atol=1e-12)
    integrated = quat_integrate_body_rate([1, 0, 0, 0], [0, 0, 2], 0.25)
    assert np.isclose(np.linalg.norm(integrated), 1.0)
    assert np.isclose(2 * np.arccos(integrated[0]), 0.5, atol=2e-3)


def test_roof_and_underside_thrusters_apply_reversed_body_z_force():
    thrusters = default_thrusters()
    assert np.array_equal(thruster_force_direction(thrusters[0]), [0, 0, -1])
    assert np.array_equal(thruster_force_direction(thrusters[4]), [0, 0, 1])
    commands = [ThrusterCommand(0.0) for _ in thrusters]
    commands[0] = ThrusterCommand(100.0)
    commands[4] = ThrusterCommand(100.0)
    force, _ = wrench_for_commands(FlightConfig(), tuple(commands))
    np.testing.assert_allclose(force, [0.0, 0.0, 0.0], atol=1e-12)


def test_allocation_respects_magnitude_gimbal_failure_and_reports_saturation():
    config = FlightConfig(failed_thrusters=frozenset({"roof_1"}))
    state = make_initial_state(config)
    allocation = allocate_wrench(np.array([5000.0, -2000.0, 9000.0]), np.array([1500.0, -1000.0, 2000.0]), state, config, 0.02)
    assert allocation.commands[0].thrust_N == 0.0
    assert "roof_1" in allocation.saturated_thrusters
    assert allocation.normalized_residual > 0.12
    for thruster, command in zip(config.thrusters, allocation.commands):
        assert 0.0 <= command.thrust_N <= thruster.max_thrust_N + 1e-8
        assert abs(command.gimbal_yaw_rad) <= thruster.gimbal_yaw_limit_rad + 1e-8
        assert abs(command.gimbal_pitch_rad) <= thruster.gimbal_pitch_limit_rad + 1e-8
    assert not allocation.feasible


def test_controller_commands_stay_inside_actuator_bounds_with_upside_down_state():
    config = FlightConfig()
    state = make_initial_state(
        config,
        position_world_m=(0, 0, 25),
        velocity_world_m_s=(0, 0, -2),
        orientation_body_to_world=quat_from_axis_angle((1, 0, 0), np.pi),
    )
    allocation = FlightController(config).command(state, 0.02)
    assert len(allocation.commands) == 10
    for thruster, command in zip(config.thrusters, allocation.commands):
        assert 0 <= command.thrust_N <= thruster.max_thrust_N
        assert abs(command.gimbal_yaw_rad) <= thruster.gimbal_yaw_limit_rad + 1e-8
        assert abs(command.gimbal_pitch_rad) <= thruster.gimbal_pitch_limit_rad + 1e-8


def test_gas_depletion_is_nonnegative_and_mass_is_conserved():
    scenario = default_scenarios()[0]
    result = simulate(scenario, dt=0.05)
    assert np.all(result.final_state.propellant_by_thruster_kg >= 0)
    initial_total_mass = 120.0 + scenario.propellant_kg
    final_mass_plus_expelled_gas = 120.0 + result.propellant_remaining_kg + result.fuel_spent_kg
    assert np.isclose(final_mass_plus_expelled_gas, initial_total_mass, atol=1e-10)
    assert result.fuel_spent_kg > 0


def test_impact_metrics_use_contact_impulse_and_explicit_stopping_time_approximation():
    result = simulate(default_scenarios()[3], dt=0.02)
    metrics = result.touchdown_metrics
    assert result.status == "failed"
    assert metrics is not None
    assert metrics["vertical_speed_m_s"] > 2.0
    assert metrics["normal_contact_impulse_Ns"] > 0
    assert np.isclose(
        metrics["average_contact_force_approx_N"],
        metrics["normal_contact_impulse_Ns"] / metrics["impact_stopping_time_assumption_s"],
    )
    assert metrics["contact_force_is_structural_stress"] is False
    assert result.peak_vertical_accel_m_s2 >= metrics["contact_deceleration_sampled_m_s2"] - 1e-8


def test_too_low_fast_case_is_not_reported_as_success():
    result = simulate(default_scenarios()[3], dt=0.02)
    assert result.status != "landed"
    assert result.failure_reason
    assert result.touchdown_metrics["vertical_speed_m_s"] > 2.0


def test_deterministic_fixed_step_run():
    scenario = default_scenarios()[0]
    first = simulate(scenario, dt=0.05)
    second = simulate(scenario, dt=0.05)
    assert first.status == second.status
    assert first.failure_reason == second.failure_reason
    assert first.elapsed_s == second.elapsed_s
    assert first.fuel_spent_kg == second.fuel_spent_kg
    np.testing.assert_array_equal(first.final_state.position_world_m, second.final_state.position_world_m)
    np.testing.assert_array_equal(first.final_state.velocity_world_m_s, second.final_state.velocity_world_m_s)
