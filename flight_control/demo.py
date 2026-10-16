"""Run the deterministic recovery suite and write JSON evidence (optional plots)."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from .simulation import SimulationResult, default_scenarios, simulate
from .export import export_result, package_outputs, validate_directory, write_json


def _write_plots(results: list[SimulationResult], output_dir: Path) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    made: list[str] = []
    for result in results:
        if not result.history:
            continue
        time_s = [item["time_s"] for item in result.history]
        z = [item["position_world_m"][2] for item in result.history]
        vz = [item["velocity_world_m_s"][2] for item in result.history]
        fuel = [item["propellant_kg"] for item in result.history]
        fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        axes[0].plot(time_s, z)
        axes[0].set_ylabel("CG z (m)")
        axes[1].plot(time_s, vz)
        axes[1].set_ylabel("vertical speed (m/s)")
        axes[2].plot(time_s, fuel)
        axes[2].set_ylabel("gas remaining (kg)")
        axes[2].set_xlabel("time (s)")
        fig.suptitle(f"{result.scenario} / {result.guidance_mode}: {result.status}")
        fig.tight_layout()
        path = output_dir / f"{result.scenario}__{result.guidance_mode}.png"
        fig.savefig(path, dpi=140)
        plt.close(fig)
        made.append(str(path))
    return made


def run_suite(dt: float = 0.02, plot: bool = False, output_dir: Path | None = None,
              csv_output: bool = False, package: bool = False) -> dict[str, Any]:
    output_dir = output_dir or Path(__file__).resolve().parent / "evidence"
    output_dir.mkdir(parents=True, exist_ok=True)
    results: list[SimulationResult] = []
    scenarios = default_scenarios()
    for scenario in scenarios:
        results.append(simulate(scenario, "predictive", dt))
        if scenario.name == "comparison_nominal":
            results.append(simulate(scenario, "early_hover", dt))

    same_constraints_pass = all(r.status == "landed" for r in results[:2])
    comparison = {
        "same_scenario_inputs": True,
        "scenario": "comparison_nominal",
        "early_hover_fuel_spent_kg": results[1].fuel_spent_kg,
        "predictive_fuel_spent_kg": results[0].fuel_spent_kg,
        "fuel_saved_predictive_vs_early_hover_kg": results[1].fuel_spent_kg - results[0].fuel_spent_kg,
        "early_hover_status": results[1].status,
        "predictive_status": results[0].status,
        "both_satisfy_same_landing_criteria": same_constraints_pass,
        "fuel_saving_demonstrated_under_same_landing_criteria": same_constraints_pass and results[0].fuel_spent_kg < results[1].fuel_spent_kg,
        "comparison_is_occupant_or_structural_safety_validation": False,
    }
    document = {
        "prototype": "Track 4 Capstone flight recovery control",
        "units": "SI",
        "world_frame": "right-handed Z-up",
        "quaternion_order": "wxyz, body-to-world",
        "assumptions": {
            "effective_exhaust_velocity_m_s": 1800.0,
            "contact_restitution": 0.04,
            "contact_friction_coefficient": 0.45,
            "contact_stopping_time_assumption_s": 0.05,
            "contact_average_force_is_structural_stress": False,
            "dt_s": dt,
        },
        "comparison": comparison,
        "results": [result.to_dict(include_history=False) for result in results],
    }
    document["scenario_output_directories"] = [export_result(result, output_dir, csv_output).name for result in results]
    document["plots"] = _write_plots(results, output_dir) if plot else []
    output_path = output_dir / "flight_recovery_demo.json"
    write_json(output_path, document)
    document["validated_pairs"] = validate_directory(output_dir)
    document["evidence_json"] = str(output_path)
    if package:
        document["package"] = str(package_outputs(output_dir))
    return document


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=0.02, help="fixed integration step, seconds")
    parser.add_argument("--plot", action="store_true", help="write optional matplotlib PNG plots")
    parser.add_argument("--output-dir", type=Path, default=None, help="evidence output directory")
    parser.add_argument("--csv", action="store_true", help="also write simple timestamp/state CSV")
    parser.add_argument("--package", action="store_true", help="zip JSON/CSV evidence, docs and schemas")
    args = parser.parse_args()
    document = run_suite(args.dt, args.plot, args.output_dir, args.csv, args.package)
    for result in document["results"]:
        td = result["touchdown_metrics"] or {}
        print(
            f"{result['scenario']:<27} {result['guidance_mode']:<12} {result['status']:<7} "
            f"fuel={result['fuel_spent_kg']:.3f} kg  "
            f"touchdown_vz={td.get('vertical_speed_m_s', float('nan')):.3f} m/s  "
            f"reason={result['failure_reason']}"
        )
    comp = document["comparison"]
    print(
        "comparison fuel delta (early-hover - predictive): "
        f"{comp['fuel_saved_predictive_vs_early_hover_kg']:.4f} kg"
    )
    print(f"evidence: {document['evidence_json']}")
    print(f"validated pairs: {document['validated_pairs']}")
    if "package" in document:
        print(f"package: {document['package']}")
    if document["plots"]:
        print("plots: " + ", ".join(document["plots"]))


if __name__ == "__main__":
    main()
