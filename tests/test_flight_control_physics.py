"""Independent analytic checks of dynamics, impulse balance and export contracts."""

import numpy as np
import pytest

from flight_control.export import export_result, load_strict, validate_directory, validate_pair
from flight_control.model import (FlightConfig, ThrusterCommand, make_initial_state,
                                  quat_from_axis_angle, quat_integrate_body_rate, quat_to_matrix)
from flight_control.physics import (STANDARD_G, advance, applied_thrusters, inertia_body,
                                    kinetic_energy, mass, resolve_contact, seat_acceleration)
from flight_control.simulation import Scenario, default_scenarios, simulate
from flight_control.telemetry import LANDING, recovery_bound, touchdown_failures


def off():
    return tuple(ThrusterCommand(0.) for _ in range(10))


@pytest.mark.parametrize("axis", [(1, 0, 0), (0, 1, 0), (0, 0, 1)])
def test_exact_constant_body_rate_and_world_frame_inverse(axis):
    omega = np.array(axis) * .7
    initial = quat_from_axis_angle((1, 2, -1), .8)
    q = initial.copy()
    for _ in range(100):
        q = quat_integrate_body_rate(q, omega, .01)
    np.testing.assert_allclose(quat_to_matrix(q), quat_to_matrix(initial) @ quat_to_matrix(quat_from_axis_angle(axis, .7)), atol=1e-13)
    r = quat_to_matrix(q)
    np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-14)
    assert np.linalg.det(r) == pytest.approx(1)


def test_threejs_basis_matches_vector_mapping_and_quaternion_conjugation():
    b = quat_to_matrix(quat_from_axis_angle((1, 0, 0), -np.pi / 2))
    np.testing.assert_allclose(b @ [2, 3, 4], [2, 4, -3], atol=1e-14)
    q = quat_from_axis_angle((1, 2, 3), 1.1)
    v = np.array([.7, -.3, 1.2])
    np.testing.assert_allclose((b @ quat_to_matrix(q) @ b.T) @ (b @ v), b @ (quat_to_matrix(q) @ v), atol=1e-14)


@pytest.mark.parametrize("index", range(10))
def test_actual_thruster_force_torque_flow_and_failure(index):
    config = FlightConfig()
    state = make_initial_state(config)
    commands = list(off())
    commands[index] = ThrusterCommand(1e6, 20, -20)
    force, torque, flows, records = applied_thrusters(config, state, tuple(commands), .02)
    t = config.thrusters[index]
    assert np.linalg.norm(force[index]) == pytest.approx(t.max_thrust_N)
    # Fixed jets act exactly along independently known +/- Z.
    if index < 8:
        np.testing.assert_allclose(force[index], [0, 0, (-1 if index < 4 else 1) * t.max_thrust_N])
    np.testing.assert_allclose(torque[index], np.cross(t.position_body_m, force[index]))
    assert flows[index] == pytest.approx(t.max_thrust_N / 1800)
    assert abs(records[index]["gimbal_yaw_rad"]) <= t.gimbal_yaw_limit_rad
    failed_config = FlightConfig(failed_thrusters=frozenset({t.name}))
    f, tau, flow, _ = applied_thrusters(failed_config, state, tuple(commands), .02)
    np.testing.assert_array_equal(f[index], np.zeros(3))
    np.testing.assert_array_equal(tau[index], np.zeros(3))
    assert flow[index] == 0


def test_fuel_exhaustion_limiter_and_step_mass_balance():
    config = FlightConfig(initial_propellant_kg=.0001)
    state = make_initial_state(config)
    initial_mass = mass(config, state)
    commands = tuple(ThrusterCommand(1e5) for _ in range(10))
    step = advance(state, config, commands, .1, ground_contact=False)
    assert state.remaining_propellant_kg == pytest.approx(0, abs=1e-18)
    assert initial_mass - mass(config, state) == pytest.approx(step.mass_flow_kg_s * .1)
    assert all(t["gas_consumed_interval_kg"] <= t["fuel_remaining_before_kg"] + 1e-18 for t in step.thrusters)


def test_freefall_and_semiimplicit_gravity_energy_error_is_known():
    config = FlightConfig(initial_propellant_kg=0)
    state = make_initial_state(config, position_world_m=(0, 0, 100), velocity_world_m_s=(1, 2, 0))
    m = mass(config, state)
    e0 = kinetic_energy(state, m, inertia_body(config, m))["total_J"] + m * 9.81 * 100
    dt = .01
    for _ in range(100):
        step = advance(state, config, off(), dt, ground_contact=False)
        assert step.seat["specific_force_g"] == pytest.approx(0, abs=1e-14)
    np.testing.assert_allclose(state.velocity_world_m_s, [1, 2, -9.81], atol=1e-12)
    np.testing.assert_allclose(state.position_world_m, [1, 2, 100 - .5 * 9.81 * (1 + dt)], atol=1e-11)
    e1 = kinetic_energy(state, m, inertia_body(config, m))["total_J"] + m * 9.81 * state.position_world_m[2]
    assert e1 - e0 == pytest.approx(-.5 * m * 9.81**2 * dt, abs=1e-8)


def test_isolated_linear_and_principal_axis_angular_momentum_energy():
    config = FlightConfig(initial_propellant_kg=0, gravity_m_s2=0)
    state = make_initial_state(config, velocity_world_m_s=(2, -3, 4), angular_velocity_body_rad_s=(0, 0, 1.2))
    m, inertia = mass(config, state), inertia_body(config, mass(config, state))
    p0 = m * state.velocity_world_m_s.copy()
    l0 = quat_to_matrix(state.orientation_body_to_world) @ inertia @ state.angular_velocity_body_rad_s
    e0 = kinetic_energy(state, m, inertia)["total_J"]
    for _ in range(100):
        advance(state, config, off(), .01, ground_contact=False)
    np.testing.assert_allclose(m * state.velocity_world_m_s, p0, atol=1e-12)
    np.testing.assert_allclose(quat_to_matrix(state.orientation_body_to_world) @ inertia @ state.angular_velocity_body_rad_s, l0, atol=1e-12)
    assert kinetic_energy(state, m, inertia)["total_J"] == pytest.approx(e0)


def test_asymmetric_torque_free_energy_error_decreases_with_dt():
    config = FlightConfig(initial_propellant_kg=0, gravity_m_s2=0)
    errors = []
    for dt in (.01, .005):
        state = make_initial_state(config, angular_velocity_body_rad_s=(.4, .6, .8))
        m, inertia = mass(config, state), inertia_body(config, mass(config, state))
        initial = kinetic_energy(state, m, inertia)["total_J"]
        for _ in range(round(1 / dt)):
            advance(state, config, off(), dt, ground_contact=False)
        errors.append(abs(kinetic_energy(state, m, inertia)["total_J"] - initial))
    assert errors[1] < .6 * errors[0]


def test_rotated_vehicle_thrust_acceleration_uses_body_to_world():
    config = FlightConfig(gravity_m_s2=0)
    state = make_initial_state(config, orientation_body_to_world=quat_from_axis_angle((0, 1, 0), np.pi / 2))
    commands = list(off())
    for i in range(4, 8):
        commands[i] = ThrusterCommand(100)
    step = advance(state, config, tuple(commands), .02, ground_contact=False)
    np.testing.assert_allclose(step.acceleration_world_m_s2, [400/138, 0, 0], atol=1e-14)
    np.testing.assert_allclose(step.angular_acceleration_body_rad_s2, [0, 0, 0], atol=1e-14)


def test_seat_acceleration_analytic_rigid_body_and_specific_force():
    # alpha_z=2, omega_z=3, seat=(1,0,0): alpha cross r=(0,2,0), centripetal=(-9,0,0).
    seat = seat_acceleration(np.array([1., 2., 3.]), np.array([0., 0., 2.]), np.array([0., 0., 3.]), np.eye(3), (1, 0, 0), np.array([0., 0., -9.81]))
    np.testing.assert_allclose(seat["acceleration_world_m_s2"], [-8, 4, 3])
    assert seat["specific_force_g"] == pytest.approx(np.linalg.norm([-8, 4, 12.81]) / STANDARD_G)


def test_contact_impulse_momentum_angular_momentum_and_energy_without_pose_projection():
    config = FlightConfig(initial_propellant_kg=0)
    state = make_initial_state(config, position_world_m=(0, 0, .39), velocity_world_m_s=(2, -1, -5), angular_velocity_body_rad_s=(.1, -.2, .3))
    before = state.copy()
    m, inertia = mass(config, state), inertia_body(config, mass(config, state))
    contact = resolve_contact(state, config, m, inertia)
    np.testing.assert_array_equal(state.position_world_m, before.position_world_m)
    np.testing.assert_array_equal(state.orientation_body_to_world, before.orientation_body_to_world)
    np.testing.assert_allclose(m * (state.velocity_world_m_s - before.velocity_world_m_s), contact["total_impulse_world_Ns"], atol=1e-10)
    np.testing.assert_allclose(inertia @ (state.angular_velocity_body_rad_s - before.angular_velocity_body_rad_s), contact["angular_impulse_about_CG_world_Nms"], atol=1e-10)
    assert contact["kinetic_energy_dissipated_approx_J"] > 0
    assert contact["normal_velocity_constraint_residual_m_s"] < 1e-6
    assert contact["max_penetration_m"] == pytest.approx(.01)
    for c in contact["contacts"]:
        assert np.linalg.norm(c["tangential_impulse_world_Ns"]) <= .45 * c["normal_impulse_Ns"] + 1e-10
        assert c["average_normal_force_approx_N"] == pytest.approx(c["normal_impulse_Ns"] / .05)
    # Actual corner loads need not, and do not, divide equally into four wheels.
    assert np.ptp([c["normal_impulse_Ns"] for c in contact["contacts"]]) > 1


def test_no_impulse_above_ground_even_inside_old_contact_skin():
    config = FlightConfig()
    state = make_initial_state(config, position_world_m=(0, 0, .401), velocity_world_m_s=(0, 0, -10))
    assert resolve_contact(state, config, mass(config, state), inertia_body(config, mass(config, state))) is None


def test_optimistic_bound_proves_low_fast_impossible_without_claiming_other_cases_safe():
    config = FlightConfig()
    low = make_initial_state(config, position_world_m=(0, 0, 1.8), velocity_world_m_s=(0, 0, -18))
    assert recovery_bound(low, config)["recovery_feasible"] is False
    assert recovery_bound(make_initial_state(config), config)["recovery_feasible"] is None


@pytest.mark.parametrize("key", list(LANDING))
def test_landing_thresholds_are_inclusive_and_independent(key):
    values = dict(LANDING)
    assert touchdown_failures(values) == []
    values[key] += 1e-6
    assert len(touchdown_failures(values)) == 1


@pytest.fixture(scope="module")
def short_result():
    return simulate(Scenario("export_test", (0, 0, 1.8), (1, 0, -18)), dt=.02)


def test_telemetry_replay_json_accounting_and_seat_impact(short_result, tmp_path):
    folder = export_result(short_result, tmp_path, csv_output=True)
    assert validate_directory(tmp_path) == 1
    a = (folder / "telemetry.json").read_bytes()
    export_result(short_result, tmp_path)
    assert (folder / "telemetry.json").read_bytes() == a
    telemetry = load_strict(folder / "telemetry.json")
    report = load_strict(folder / "safety_report.json")
    assert telemetry["samples"][0]["time_s"] == 0
    row = telemetry["samples"][-1]
    assert row["estimated_state"]["time_s"] == pytest.approx(row["interval_start_s"])
    assert row["estimated_state"]["position_world_m"] == row["physics_ground_truth"]["dynamics_evaluation_state"]["position_world_m"]
    assert row["sensor_measurements"] == {"mode": "NOT_SIMULATED", "imu": None, "radar": None, "lidar": None}
    td, contact = report["touchdown"], report["contact_loads"]
    r = quat_to_matrix(td["orientation_body_to_world_wxyz"])
    dw = np.array(contact["post_impact_angular_velocity_body_rad_s"]) - contact["pre_impact_angular_velocity_body_rad_s"]
    expected = np.array(td["CG_delta_v_world_m_s"]) + r @ np.cross(dw, report["vehicle_assumptions"]["seat_position_body_m"])
    np.testing.assert_allclose(td["seat_delta_v_world_m_s"], expected)
    assert td["seat_impact_average_deceleration_g"] == pytest.approx(np.linalg.norm(expected) / .05 / STANDARD_G)
    assert report["assessments"]["occupant_safety_assessment"] == "NOT_VALIDATED"
    assert "optimistic_vertical_braking_bound_exceeded" in report["failure_reasons"]


@pytest.mark.parametrize("corruption", ["nan", "time", "force", "verdict"])
def test_export_validator_rejects_corrupt_evidence(short_result, tmp_path, corruption):
    folder = export_result(short_result, tmp_path)
    telemetry = load_strict(folder / "telemetry.json")
    report = load_strict(folder / "safety_report.json")
    if corruption == "nan":
        telemetry["samples"][1]["mass_flow_kg_s"] = float("nan")
    elif corruption == "time":
        telemetry["samples"][1]["time_s"] += .001
    elif corruption == "force":
        telemetry["samples"][1]["achieved_force_body_N"][0] += 1
    else:
        report["success"] = True
    with pytest.raises((ValueError, AssertionError)):
        validate_pair(telemetry, report)


def test_timeout_null_touchdown_and_failure_reason(tmp_path):
    result = simulate(Scenario("timeout", (0, 0, 100), (0, 0, 0), timeout_s=.04))
    export_result(result, tmp_path)
    assert result.status == "failed" and result.touchdown_metrics is None
    assert result.safety_report["failure_reasons"] == ["simulation_timeout"]


def test_perfect_estimator_is_independent_copy():
    from flight_control.controller import PerfectStateEstimator
    state = make_initial_state(FlightConfig())
    estimated = PerfectStateEstimator().estimate(state, .02)
    estimated.position_world_m[:] = 999
    assert state.position_world_m[2] == 30


def test_invalid_numeric_config_and_initial_ground_overlap():
    with pytest.raises(ValueError):
        FlightConfig(gravity_m_s2=float("nan"))
    with pytest.raises(ValueError):
        simulate(Scenario("overlap", (0, 0, .1), (0, 0, 0)))


def test_handoff_archive_contains_valid_manifest_and_contracts(short_result, tmp_path):
    import hashlib
    import zipfile
    from flight_control.export import package_outputs, write_json
    export_result(short_result, tmp_path)
    write_json(tmp_path / "flight_recovery_demo.json", {"results": [short_result.to_dict(False)]})
    archive = package_outputs(tmp_path)
    manifest = load_strict(tmp_path / "manifest.json")
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.testzip() is None
        for name, digest in manifest["sha256"].items():
            assert hashlib.sha256(bundle.read(name)).hexdigest() == digest
        assert "docs/schemas/telemetry.schema.json" in bundle.namelist()
        assert "docs/INTEGRATION.md" in bundle.namelist()
