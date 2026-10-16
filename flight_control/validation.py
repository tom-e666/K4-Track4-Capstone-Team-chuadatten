"""Verify six scenario contracts and optional byte-for-byte deterministic replay."""

import argparse
import hashlib
from pathlib import Path

from .export import load_strict, validate_directory, write_json
from .simulation import default_scenarios


def verify_suite(output_dir: Path, replay_dir: Path | None = None) -> dict:
    pairs = validate_directory(output_dir)
    expected = {f"{s.name}__predictive" for s in default_scenarios()} | {"comparison_nominal__early_hover"}
    actual = {p.parent.name for p in output_dir.glob("*/telemetry.json")}
    if actual != expected:
        raise ValueError(f"scenario set differs: {actual ^ expected}")
    low = load_strict(output_dir / "too_low_fast_impossible__predictive/safety_report.json")
    if low["success"] or "optimistic_vertical_braking_bound_exceeded" not in low["failure_reasons"]:
        raise ValueError("impossible recovery was not correctly diagnosed")
    hashes = {}
    for name in sorted(expected):
        for filename in ("telemetry.json", "safety_report.json"):
            relative = Path(name) / filename
            data = (output_dir / relative).read_bytes()
            hashes[relative.as_posix()] = hashlib.sha256(data).hexdigest()
            if replay_dir is not None and data != (replay_dir / relative).read_bytes():
                raise ValueError(f"nondeterministic replay: {relative}")
    summary = {"validated_scenario_pairs": pairs,
               "deterministic_replay_pairs": pairs if replay_dir is not None else None,
               "impossible_recovery_diagnosed": True, "sha256": hashes}
    write_json(output_dir / "validation_summary.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--replay-dir", type=Path)
    args = parser.parse_args()
    result = verify_suite(args.output_dir, args.replay_dir)
    print(f"Validated {result['validated_scenario_pairs']} pairs; deterministic replay pairs: {result['deterministic_replay_pairs']}")


if __name__ == "__main__":
    main()
