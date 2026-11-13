"""P3 simulation configuration for the large-scale emergency-aware routing build.

This module captures the agreed design targets for the upcoming build phase:
- 4,000 vehicles in total (set from the measured capacity sweep; see
  config/experiment_contract.md section 11, decision 13.8)
- 5–7% emergency vehicles
- 3600-second simulation duration
"""

from dataclasses import dataclass
from typing import Dict, List

SIMULATION_DURATION_SECONDS = 3600
# 8000 until 2026-09-04. Lowered to the network's measured maximum stable
# operating point - see experiment_contract.md section 11, decision 13.8.
TOTAL_VEHICLES = 4000
EMERGENCY_RATIO = 0.06
EMERGENCY_RATIO_MIN = 0.05
EMERGENCY_RATIO_MAX = 0.07
# The four emergency roles. Used when *generating* demand.
EMERGENCY_VEHICLE_ROLES = ("ambulance", "fire_truck", "police", "relief_vehicle")

# Prefix used by the ambient emergency fleet in demand/porto.emergency.rou.xml, which
# scripts/10 writes as emergency_ambulance, emergency_police and so on.
AMBIENT_EMERGENCY_PREFIX = "emergency_"

# Every vType id that counts as an emergency vehicle. Used when *classifying* a vehicle
# seen in the simulation.
#
# This must contain both naming schemes. The three scripted responders use the bare role
# names, but the 240 ambient emergency vehicles required by contract decision 13.1 use the
# prefixed names, and the previous four-name list matched none of them - so 240 of the 243
# emergency vehicles were treated as ordinary traffic by the protocol layer, the routing
# logic and the metrics, even though SUMO gives all of them vClass="emergency".
AMBIENT_EMERGENCY_TYPES = tuple(f"{AMBIENT_EMERGENCY_PREFIX}{role}" for role in EMERGENCY_VEHICLE_ROLES)
EMERGENCY_VEHICLE_TYPES = tuple(EMERGENCY_VEHICLE_ROLES) + AMBIENT_EMERGENCY_TYPES


def is_emergency_vehicle_type(vehicle_type: str) -> bool:
    """True if this vType id denotes an emergency vehicle."""
    return bool(vehicle_type) and (
        vehicle_type in EMERGENCY_VEHICLE_TYPES
        or vehicle_type.startswith(AMBIENT_EMERGENCY_PREFIX)
    )


@dataclass(frozen=True)
class SimulationConfig:
    """Central configuration object for the P3 simulation build."""

    simulation_duration_seconds: int = SIMULATION_DURATION_SECONDS
    total_vehicles: int = TOTAL_VEHICLES
    emergency_ratio: float = EMERGENCY_RATIO

    @property
    def emergency_vehicle_count(self) -> int:
        return int(self.total_vehicles * self.emergency_ratio)

    @property
    def normal_vehicle_count(self) -> int:
        return self.total_vehicles - self.emergency_vehicle_count

    def validate(self) -> None:
        if not (EMERGENCY_RATIO_MIN <= self.emergency_ratio <= EMERGENCY_RATIO_MAX):
            raise ValueError("Emergency ratio must stay within 5–7% of total vehicles.")
        if self.total_vehicles <= 0:
            raise ValueError("Total vehicles must be positive.")
        if self.simulation_duration_seconds <= 0:
            raise ValueError("Simulation duration must be positive.")


def get_default_config() -> SimulationConfig:
    """Return the default large-scale P3 configuration."""

    return SimulationConfig()


def get_scope_summary() -> Dict[str, object]:
    """Return a human-readable summary of the agreed simulation scope."""

    config = get_default_config()
    return {
        "simulation_duration_seconds": config.simulation_duration_seconds,
        "total_vehicles": config.total_vehicles,
        "emergency_ratio": config.emergency_ratio,
        "emergency_vehicle_count": config.emergency_vehicle_count,
        "normal_vehicle_count": config.normal_vehicle_count,
        "emergency_vehicle_types": list(EMERGENCY_VEHICLE_ROLES),
    }


def build_vehicle_type_distribution(total_vehicles: int = TOTAL_VEHICLES, emergency_ratio: float = EMERGENCY_RATIO) -> List[str]:
    """Create a simple vehicle-type list for the scaled simulation setup."""

    config = SimulationConfig(
        total_vehicles=total_vehicles,
        emergency_ratio=emergency_ratio,
    )
    config.validate()

    emergency_count = config.emergency_vehicle_count
    normal_count = config.normal_vehicle_count
    emergency_types = list(EMERGENCY_VEHICLE_ROLES)
    vehicles = ["normal"] * normal_count
    vehicles.extend(
        emergency_types[index % len(emergency_types)]
        for index in range(emergency_count)
    )
    return vehicles
