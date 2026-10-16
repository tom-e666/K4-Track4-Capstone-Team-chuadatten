"""Strict JSON exports and independent contract/accounting validation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import zipfile
from functools import lru_cache

import numpy as np
from jsonschema import Draft202012Validator

from .telemetry import LANDING, SCHEMA_VERSION
from .model import quat_to_matrix


def write_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def export_result(result, output_dir: Path, csv_output: bool = False) -> Path:
    folder = output_dir / f"{result.scenario}__{result.guidance_mode}"
    folder.mkdir(parents=True, exist_ok=True)
    telemetry = {"schema_version": SCHEMA_VERSION, "scenario_id": result.scenario,
                 "guidance_mode": result.guidance_mode, "physics_dt_s": result.dt_s,
                 "sample_interval_s": result.dt_s, "world_frame": "right_handed_Z_up",
                 "quaternion_order": "wxyz_body_to_world", "samples": result.history}
    validate_pair(telemetry, result.safety_report)
    write_json(folder / "telemetry.json", telemetry)
    write_json(folder / "safety_report.json", result.safety_report)
    if csv_output:
        with (folder / "telemetry.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "fuel_remaining_kg", "attitude_error_deg"])
            for row in result.history:
                writer.writerow([row["time_s"], *row["position_world_m"], *row["velocity_world_m_s"], row["fuel_remaining_kg"], row["attitude_error_deg"]])
    return folder


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    if isinstance(value, dict):
        for child in value.values():
            _finite(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            _finite(child)
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        _require(np.isfinite(value), "nonfinite number")


@lru_cache(maxsize=2)
def _schema_validator(name: str):
    schema = json.loads((Path(__file__).parent / "schemas" / f"{name}.schema.json").read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


def validate_pair(telemetry: dict, report: dict) -> None:
    """Validate required shapes plus physics accounting, independent of controller."""
    _finite(telemetry)
    _finite(report)
    _schema_validator("telemetry").validate(telemetry)
    _schema_validator("safety_report").validate(report)
    _require(telemetry["schema_version"] == report["schema_version"] == SCHEMA_VERSION, "schema version mismatch")
    _require(telemetry["scenario_id"] == report["scenario_id"], "scenario mismatch")
    _require(telemetry["guidance_mode"] == report["guidance_mode"], "guidance mismatch")
    rows = telemetry["samples"]
    _require(len(rows) >= 2, "initial state plus physics steps required")
    dt = telemetry["physics_dt_s"]
    _require(dt > 0 and dt == telemetry["sample_interval_s"] == report["physics_dt_s"], "invalid sample interval")
    _require(rows[0]["time_s"] == 0 and rows[0]["interval_dt_s"] is None, "missing initial state")
    np.testing.assert_allclose(np.diff([r["time_s"] for r in rows]), dt, atol=1e-12, rtol=0)
    config = report["vehicle_assumptions"]
    initial = report["fuel_summary"]["initial_kg"]
    names = [t["name"] for t in config["thrusters"]]
    previous_fuel = initial
    consumed = np.zeros(len(names))
    for index, r in enumerate(rows):
        required = ("schema_version", "scenario_id", "time_s", "physics_ground_truth", "sensor_measurements",
                    "estimated_state", "controller_mode", "requested_force_body_N", "requested_torque_body_Nm",
                    "achieved_force_body_N", "achieved_torque_body_Nm", "thruster_states", "fuel_consumed_kg",
                    "fuel_remaining_kg", "mass_flow_kg_s", "ground_clearance_m", "attitude_error_deg",
                    "recovery_feasible", "diagnostic_events")
        _require(all(k in r for k in required), "missing required telemetry fields")
        _require(r["schema_version"] == SCHEMA_VERSION and r["scenario_id"] == telemetry["scenario_id"], "sample identity mismatch")
        truth = r["physics_ground_truth"]
        for field, length in (("position_world_m", 3), ("velocity_world_m_s", 3),
                              ("orientation_body_to_world_wxyz", 4), ("angular_velocity_body_rad_s", 3)):
            _require(len(truth[field]) == length, "invalid state vector shape")
        np.testing.assert_allclose(np.linalg.norm(truth["orientation_body_to_world_wxyz"]), 1, atol=1e-12)
        _require(r["fuel_remaining_kg"] >= 0, "negative fuel")
        np.testing.assert_allclose(r["fuel_remaining_kg"] + r["fuel_consumed_kg"], initial, atol=1e-10)
        np.testing.assert_allclose(truth["mass_kg"], config["dry_mass_kg"] + r["fuel_remaining_kg"], atol=1e-10)
        np.testing.assert_allclose(sum(truth["propellant_by_thruster_kg"]), r["fuel_remaining_kg"], atol=1e-10)
        if index == 0:
            continue
        np.testing.assert_allclose(r["interval_start_s"], rows[index-1]["time_s"], atol=1e-12)
        _require(r["interval_dt_s"] == dt, "interval duration mismatch")
        _require([t["name"] for t in r["thruster_states"]] == names, "thruster map mismatch")
        forces, torques = [], []
        for i, t in enumerate(r["thruster_states"]):
            spec = config["thrusters"][i]
            _require(0 <= t["applied_thrust_N"] <= spec["max_thrust_N"] + 1e-9, "thrust bounds violated")
            _require(abs(t["gimbal_yaw_rad"]) <= spec["gimbal_yaw_limit_rad"] + 1e-10, "yaw bound violated")
            _require(abs(t["gimbal_pitch_rad"]) <= spec["gimbal_pitch_limit_rad"] + 1e-10, "pitch bound violated")
            if t["failed"] or not t["available"]:
                _require(t["applied_thrust_N"] == 0, "unavailable thruster applied force")
            np.testing.assert_allclose(np.linalg.norm(t["force_direction_body"]), 1, atol=1e-12)
            np.testing.assert_allclose(t["force_body_N"], np.array(t["force_direction_body"]) * t["applied_thrust_N"], atol=1e-10)
            np.testing.assert_allclose(t["torque_body_Nm"], np.cross(spec["position_body_m"], t["force_body_N"]), atol=1e-10)
            np.testing.assert_allclose(t["gas_consumed_interval_kg"], t["applied_thrust_N"] / config["effective_exhaust_velocity_m_s"] * dt, atol=1e-12)
            consumed[i] += t["gas_consumed_interval_kg"]
            forces.append(t["force_body_N"])
            torques.append(t["torque_body_Nm"])
        np.testing.assert_allclose(np.sum(forces, axis=0), r["achieved_force_body_N"], atol=1e-9)
        np.testing.assert_allclose(np.sum(torques, axis=0), r["achieved_torque_body_Nm"], atol=1e-9)
        evaluation = truth["dynamics_evaluation_state"]
        rotation = quat_to_matrix(evaluation["orientation_body_to_world_wxyz"])
        inertia = np.asarray(evaluation["inertia_body_kg_m2"])
        omega = np.asarray(evaluation["angular_velocity_body_rad_s"])
        acceleration = np.asarray(truth["acceleration_world_m_s2"])
        alpha = np.asarray(truth["angular_acceleration_body_rad_s2"])
        gravity = np.array([0., 0., -config["gravity_m_s2"]])
        np.testing.assert_allclose(evaluation["mass_kg"] * (acceleration - gravity), rotation @ r["achieved_force_body_N"], atol=1e-8)
        np.testing.assert_allclose(inertia @ alpha + np.cross(omega, inertia @ omega), r["achieved_torque_body_Nm"], atol=1e-8)
        seat_r = np.asarray(config["seat_position_body_m"])
        expected_seat = acceleration + rotation @ (np.cross(alpha, seat_r) + np.cross(omega, np.cross(omega, seat_r)))
        np.testing.assert_allclose(r["seat_free_flight"]["acceleration_world_m_s2"], expected_seat, atol=1e-9)
        pre_impact_velocity = np.asarray(evaluation["velocity_world_m_s"]) + acceleration * dt
        # First contact never projects the position or replaces physical velocity.
        np.testing.assert_allclose(truth["position_world_m"], np.asarray(evaluation["position_world_m"]) + pre_impact_velocity * dt, atol=1e-9)
        impulse = truth["contact"]["total_impulse_world_Ns"] if truth["contact"] else np.zeros(3)
        np.testing.assert_allclose(truth["velocity_world_m_s"], pre_impact_velocity + np.asarray(impulse) / truth["mass_kg"], atol=1e-9)
        np.testing.assert_allclose(previous_fuel - r["fuel_remaining_kg"], r["mass_flow_kg_s"] * dt, atol=1e-10)
        previous_fuel = r["fuel_remaining_kg"]
    np.testing.assert_allclose(consumed, [s["gas_consumed_kg"] for s in report["thruster_statistics"]], atol=1e-10)
    np.testing.assert_allclose(report["elapsed_s"], rows[-1]["time_s"], atol=1e-12)
    np.testing.assert_allclose(report["fuel_summary"]["remaining_kg"], rows[-1]["fuel_remaining_kg"], atol=1e-10)
    assessments = report["assessments"]
    _require(assessments["structural_assessment"] == "STRUCTURAL_UNKNOWN", "unvalidated structural claim")
    _require(assessments["occupant_safety_assessment"] == "NOT_VALIDATED", "unvalidated occupant claim")
    td, contact = report["touchdown"], report["contact_loads"]
    if td is not None:
        expected_pass = all(td[k] <= v for k, v in LANDING.items())
        _require(report["success"] == expected_pass, "landing verdict inconsistent")
        _require(assessments["landing_criteria"] == ("LANDING_PASS" if expected_pass else "LANDING_FAIL"), "landing label inconsistent")
        _require(contact is not None, "touchdown without contact record")
        duration = config["impact_stopping_time_assumption_s"]
        np.testing.assert_allclose(sum(c["normal_impulse_Ns"] for c in contact["contacts"]), td["normal_contact_impulse_Ns"], atol=1e-9)
        np.testing.assert_allclose(td["average_contact_force_approx_N"], td["normal_contact_impulse_Ns"] / duration, atol=1e-9)
        np.testing.assert_allclose(np.sum([c["total_impulse_world_Ns"] for c in contact["contacts"]], axis=0), contact["total_impulse_world_Ns"], atol=1e-8)
        np.testing.assert_allclose(contact["total_average_force_approx_world_N"], np.asarray(contact["total_impulse_world_Ns"]) / duration, atol=1e-8)
        dv = np.array(contact["post_impact_velocity_world_m_s"]) - contact["pre_impact_velocity_world_m_s"]
        np.testing.assert_allclose(rows[-1]["physics_ground_truth"]["mass_kg"] * dv, contact["total_impulse_world_Ns"], atol=1e-8)
        final_truth = rows[-1]["physics_ground_truth"]
        rotation = quat_to_matrix(final_truth["orientation_body_to_world_wxyz"])
        dw = np.array(contact["post_impact_angular_velocity_body_rad_s"]) - contact["pre_impact_angular_velocity_body_rad_s"]
        np.testing.assert_allclose(rotation @ np.asarray(final_truth["inertia_body_kg_m2"]) @ dw, contact["angular_impulse_about_CG_world_Nms"], atol=1e-8)
    if not report["success"]:
        _require(bool(report["failure_reasons"]), "failure has no machine-readable reason")


def load_strict(path: Path) -> dict:
    def reject(value):
        raise ValueError(f"invalid JSON constant: {value}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def validate_directory(output: Path) -> int:
    count = 0
    for path in sorted(output.glob("*/telemetry.json")):
        validate_pair(load_strict(path), load_strict(path.with_name("safety_report.json")))
        count += 1
    _require(count > 0, "no scenario exports found")
    return count


def package_outputs(output: Path) -> Path:
    docs = Path(__file__).parent
    paths = sorted(output.glob("*/*.json")) + sorted(output.glob("*/*.csv"))
    paths += [output / "flight_recovery_demo.json"]
    if (output / "validation_summary.json").exists():
        paths.append(output / "validation_summary.json")
    manifest = {str(p.relative_to(output)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    write_json(output / "manifest.json", {"schema_version": SCHEMA_VERSION, "sha256": manifest})
    paths.append(output / "manifest.json")
    archive = output / "flight_control_handoff.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            info = zipfile.ZipInfo(str(path.relative_to(output)).replace("\\", "/"), date_time=(2026, 10, 16, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            bundle.writestr(info, path.read_bytes())
        for path in sorted(docs.glob("*.md")) + sorted((docs / "schemas").glob("*.json")):
            bundle.write(path, "docs/" + str(path.relative_to(docs)).replace("\\", "/"))
    return archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    print(f"Validated {validate_directory(args.output_dir)} telemetry/safety-report pairs")


if __name__ == "__main__":
    main()
