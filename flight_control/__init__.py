"""Self-contained 6-DoF flight recovery prototype (SI units, Z-up world)."""

from .model import FlightConfig, FlightState, ThrusterCommand, default_thrusters
from .simulation import Scenario, SimulationResult, simulate

__all__ = [
    "FlightConfig",
    "FlightState",
    "Scenario",
    "SimulationResult",
    "ThrusterCommand",
    "default_thrusters",
    "simulate",
]
