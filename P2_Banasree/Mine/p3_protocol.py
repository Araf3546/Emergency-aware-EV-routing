"""P3 protocol layer for protocol-driven emergency-aware routing.

This module defines the message schema, serialization helpers, and a lightweight
neighbor-state manager. It is intentionally simple and reusable so that it can be
wired into SUMO routing logic later without changing the existing architecture.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from p3_simulation_config import EMERGENCY_VEHICLE_TYPES


@dataclass
class VehicleMessage:
    """Compact vehicle-to-vehicle message schema for the P3 build."""

    vehicle_id: str
    timestamp: float
    vehicle_type: str
    position_edge: str
    position_lane: Optional[str] = None
    speed: float = 0.0
    priority_level: int = 0
    emergency_flag: bool = False
    route_intent: str = ""
    congestion_estimate: Optional[float] = None
    yield_request: bool = False
    next_edge: Optional[str] = None
    distance_to_emergency: Optional[float] = None
    hop_count: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "VehicleMessage":
        return cls(**payload)

    def is_emergency(self) -> bool:
        return self.emergency_flag or self.vehicle_type in EMERGENCY_VEHICLE_TYPES

    def is_stale(self, current_time: float, freshness_window: float = 3.0) -> bool:
        return (current_time - self.timestamp) > freshness_window

    def serialize(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def deserialize(cls, payload: str) -> "VehicleMessage":
        data = json.loads(payload)
        return cls.from_dict(data)


@dataclass
class NeighborEntry:
    """Stores the most recent message received from a neighbor."""

    message: VehicleMessage
    last_seen: float


class ProtocolStateManager:
    """Tracks the latest messages seen from nearby vehicles."""

    def __init__(self, freshness_window: float = 3.0) -> None:
        self.freshness_window = freshness_window
        self.neighbors: Dict[str, NeighborEntry] = {}

    def update(self, current_time: float, message: VehicleMessage) -> None:
        self.neighbors[message.vehicle_id] = NeighborEntry(
            message=message,
            last_seen=current_time,
        )
        self._prune_stale(current_time)

    def _prune_stale(self, current_time: float) -> None:
        stale_ids = [
            vehicle_id
            for vehicle_id, entry in self.neighbors.items()
            if entry.message.is_stale(current_time, self.freshness_window)
        ]
        for vehicle_id in stale_ids:
            self.neighbors.pop(vehicle_id, None)

    def get_fresh_neighbors(self, current_time: float) -> List[VehicleMessage]:
        self._prune_stale(current_time)
        return [entry.message for entry in self.neighbors.values()]

    def get_emergency_neighbors(self, current_time: float) -> List[VehicleMessage]:
        return [message for message in self.get_fresh_neighbors(current_time) if message.is_emergency()]

    def clear(self) -> None:
        self.neighbors.clear()


def build_message(
    vehicle_id: str,
    current_time: float,
    vehicle_type: str,
    position_edge: str,
    speed: float,
    priority_level: int = 0,
    emergency_flag: bool = False,
    route_intent: str = "",
    position_lane: Optional[str] = None,
    congestion_estimate: Optional[float] = None,
    yield_request: bool = False,
    next_edge: Optional[str] = None,
    distance_to_emergency: Optional[float] = None,
    hop_count: int = 0,
) -> VehicleMessage:
    """Create a message object using the agreed P3 schema."""

    return VehicleMessage(
        vehicle_id=vehicle_id,
        timestamp=current_time,
        vehicle_type=vehicle_type,
        position_edge=position_edge,
        position_lane=position_lane,
        speed=speed,
        priority_level=priority_level,
        emergency_flag=emergency_flag,
        route_intent=route_intent,
        congestion_estimate=congestion_estimate,
        yield_request=yield_request,
        next_edge=next_edge,
        distance_to_emergency=distance_to_emergency,
        hop_count=hop_count,
    )


def build_emergency_message(
    vehicle_id: str,
    current_time: float,
    vehicle_type: str,
    position_edge: str,
    speed: float,
    route_intent: str = "",
    position_lane: Optional[str] = None,
    next_edge: Optional[str] = None,
) -> VehicleMessage:
    """Convenience constructor for an emergency broadcast."""

    return build_message(
        vehicle_id=vehicle_id,
        current_time=current_time,
        vehicle_type=vehicle_type,
        position_edge=position_edge,
        speed=speed,
        priority_level=4,
        emergency_flag=True,
        route_intent=route_intent,
        position_lane=position_lane,
        next_edge=next_edge,
    )
