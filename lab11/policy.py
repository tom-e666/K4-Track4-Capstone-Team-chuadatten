"""Part E: choose a policy honestly. TODO 10."""
from __future__ import annotations


def choose_policy(table: list[dict], baseline_progress: float, max_loss: float = 0.05):
    threshold = (1.0 - max_loss) * baseline_progress
    qualifying = [row for row in table if row["progress"] >= threshold]
    if not qualifying:
        return None
    return min(qualifying, key=lambda r: (r["fail_hi"], -r["progress"]))
