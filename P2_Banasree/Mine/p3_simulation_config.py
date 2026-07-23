"""P3 simulation configuration for the large-scale emergency-aware routing build.

This module captures the agreed design targets for the upcoming build phase:
- 8,000 vehicles in total
- 5–7% emergency vehicles
- 3600-second simulation duration
"""

from dataclasses import dataclass
from typing import Dict, List

SIMULATION_DURATION_SECONDS = 3600
TOTAL_VEHICLES = 8000
EMERGENCY_RATIO = 0.06
EMERGENCY_RATIO_MIN = 0.05
EMERGENCY_RATIO_MAX = 0.07
EMERGENCY_VEHICLE_TYPES = ("ambulance", "fire_truck", "police", "relief_vehicle")


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
        "emergency_vehicle_types": list(EMERGENCY_VEHICLE_TYPES),
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
    emergency_types = list(EMERGENCY_VEHICLE_TYPES)
    vehicles = ["normal"] * normal_count
    vehicles.extend(
        emergency_types[index % len(emergency_types)]
        for index in range(emergency_count)
    )
    return vehicles
