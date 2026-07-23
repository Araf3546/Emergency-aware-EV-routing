"""Protocol-aware hybrid routing scaffold for the P3 build.

This module is intentionally lightweight and designed to mirror the design-phase
requirements for E3 Hybrid:
- ACO pheromone as reinforcement
- PSO-style velocity adaptation as dynamic adjustment
- BCO-inspired scout/exploration bonus
- protocol-based emergency weight from V2V messages
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ProtocolMessage:
    """Minimal protocol message used by the hybrid router."""

    vehicle_id: str
    emergency_flag: bool = False
    priority_level: int = 0
    next_edge: Optional[str] = None
    route_intent: Optional[str] = None
    distance_to_emergency: Optional[float] = None
    yield_request: bool = False
    hop_count: int = 0


@dataclass(frozen=True)
class HybridContext:
    """Context values for scoring one candidate edge."""

    pheromone: float
    heuristic: float
    protocol_weight: float
    bco_exploration: float
    velocity: float = 0.0


@dataclass
class HybridRouter:
    """Scaffold for the E3 Hybrid decision logic."""

    weights: Dict[str, float] = field(
        default_factory=lambda: {
            "pheromone": 0.4,
            "heuristic": 0.2,
            "protocol": 0.4,
            "bco": 0.1,
            "velocity": 0.05,
        }
    )
    scout_memory: Dict[str, float] = field(default_factory=dict)

    def score_edge(self, context: HybridContext) -> float:
        """Compute a combined score for a single candidate edge."""

        score = (
            self.weights.get("pheromone", 0.4) * context.pheromone
            + self.weights.get("heuristic", 0.2) * context.heuristic
            + self.weights.get("protocol", 0.4) * context.protocol_weight
            + self.weights.get("bco", 0.1) * context.bco_exploration
            + self.weights.get("velocity", 0.05) * max(0.0, context.velocity)
        )
        return round(min(1.0, max(0.0, score)), 6)

    def select_best_edge(
        self,
        candidates: Sequence[Tuple[str, HybridContext]],
    ) -> Tuple[Optional[str], float]:
        """Return the best edge and its score from a list of candidates."""

        if not candidates:
            return None, 0.0

        scored = [(edge, self.score_edge(context)) for edge, context in candidates]
        best_edge, best_score = max(scored, key=lambda item: item[1])
        return best_edge, best_score

    def protocol_weight_for_edge(
        self,
        edge_id: str,
        messages: Iterable[ProtocolMessage],
        is_emergency_vehicle: bool = False,
    ) -> float:
        """Turn incoming protocol messages into a single edge-level weight."""

        total = 0.0
        count = 0
        for message in messages:
            if message.next_edge and message.next_edge != edge_id:
                continue

            if message.emergency_flag:
                if is_emergency_vehicle:
                    total += 0.8 + min(0.2, message.priority_level / 10.0)
                else:
                    total += (
                        0.8 + min(0.2, message.priority_level / 10.0)
                        if message.next_edge == edge_id
                        else 0.2
                    )
            elif message.yield_request:
                total += 0.4 + min(0.1, message.priority_level / 10.0)
            else:
                total += 0.2 + min(0.1, message.priority_level / 10.0)

            count += 1

        if count == 0:
            return 0.0
        return round(min(1.0, total / count), 6)

    def decay_scout_memory(self, decay_rate: float = 0.02) -> None:
        """Gradually lower exploration memory to keep the BCO component adaptive."""

        for edge_id in list(self.scout_memory.keys()):
            new_value = max(0.0, self.scout_memory[edge_id] - decay_rate)
            if new_value <= 0.0:
                self.scout_memory.pop(edge_id, None)
            else:
                self.scout_memory[edge_id] = new_value

    def update_exploration_memory(
        self,
        candidate_edges: Sequence[str],
        selected_edge: Optional[str],
    ) -> None:
        """Update the BCO scout memory after a route decision."""

        for edge in candidate_edges:
            if edge == selected_edge:
                self.scout_memory[edge] = min(1.0, self.scout_memory.get(edge, 0.0) + 0.15)
            else:
                self.scout_memory[edge] = min(1.0, self.scout_memory.get(edge, 0.0) + 0.05)

    def build_context(
        self,
        pheromone: float,
        heuristic: float,
        edge_id: str,
        messages: Iterable[ProtocolMessage],
        velocity: float = 0.0,
        is_emergency_vehicle: bool = False,
    ) -> HybridContext:
        """Create a scoring context using local and protocol information."""

        protocol_weight = self.protocol_weight_for_edge(edge_id, messages, is_emergency_vehicle)
        exploration = self.scout_memory.get(edge_id, 0.0)
        return HybridContext(
            pheromone=pheromone,
            heuristic=heuristic,
            protocol_weight=protocol_weight,
            bco_exploration=exploration,
            velocity=velocity,
        )
