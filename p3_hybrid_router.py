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

# Protocol weights for an edge. Higher means "more attractive to drive on".
EMERGENCY_CORRIDOR_WEIGHT = 0.9         # a responder may keep using its own corridor
ORDINARY_AVOID_EMERGENCY_WEIGHT = 0.05  # ordinary traffic gets out of the way (audit P8)
NEUTRAL_WEIGHT = 0.5                    # no emergency information about this edge
# YIELDING_NEIGHBOUR_WEIGHT (0.3, "a neighbour is yielding here, so prefer elsewhere") was removed on
# 2026-09-15 by the clean design: a yield flag no longer affects routing (see protocol_weight_for_edge).
# The responder's claim on the edge it is about to enter (P8, ablation gate 3). An explicit
# emergency-layer rule since 2026-09-14: it used to arrive only as the responder's own broadcast,
# read back as though a neighbour had sent it (audit A1). The value is unchanged: the corridor
# weight plus the priority bonus of a responder's priority-4 broadcast, 0.9 + min(0.1, 4 / 40).
RESPONDER_CORRIDOR_WEIGHT = EMERGENCY_CORRIDOR_WEIGHT + min(0.1, 4 / 40.0)

# The five scoring components, in the order they are weighted. Audit A3: the score is a
# weighted sum of these after each one has been min-max normalised across the candidate
# set, so no component can dominate by being on a larger numeric scale than the others.
COMPONENT_NAMES = ("pheromone", "heuristic", "protocol", "bco", "velocity")

DEFAULT_WEIGHTS = {
    "pheromone": 0.4,
    "heuristic": 0.2,
    "protocol": 0.4,
    "bco": 0.1,
    "velocity": 0.05,
}

# Value given to a component that is identical for every candidate. Such a component
# cannot discriminate between the candidates, and a constant added to every score cannot
# change the argmax, so the exact value is arbitrary; the midpoint keeps scores readable.
CONSTANT_COMPONENT_VALUE = 0.5

# How long BCO remembers that an edge was used, in simulated seconds. Set to one reroute
# interval: the minimum for a vehicle to still remember an edge at its own next decision,
# which the previous per-decision decay (effectively 1.3 s) never achieved.
#
# 30, 120 and 300 s were measured at 3,600 s on 2026-09-09. Longer memories cost completed
# trips monotonically (3,242 / 3,182 / 3,150 against 3,338 for the old behaviour) and did
# not improve traffic distribution - Gini moved non-monotonically, which is noise, not a
# trend. 30 s is the best of the three on completions and teleports and stays closest to the
# previously measured behaviour while removing the dependence on how busy the router is.
BCO_MEMORY_HALF_LIFE_S = 30.0


TUNER_EVALUATOR = "p3_runtime.P3RuntimeCoordinator(emergency_layer=True)"
# The contracted campaign: 20 trials of 3,600 s on the tuning seed, 10 random start-up then 10 TPE (contract 13.19,
# amended from 25 on 2026-09-15 by the user, after the noise analysis in results/tuning_budget_analysis_2026-09-15/).
CONTRACT_TUNING_TRIALS = 20


def _check_campaign_provenance(path, document) -> None:
    """A tuner output file is only a valid weight set if it came from the real campaign.

    That means the corrected evaluator (2026-09-12), the finalized objective (2026-09-13,
    contract 13.17), the held-out tuning seed (13.15) and the full contracted duration (13.5).
    Every file written before those fixes lacks the first two, and the smoke and validation
    studies (700 s, 1,800 s) fail the last. Refusing them here means no stale or throwaway
    weight set can reach the router, however it is configured.

    Since PD-1 question 2 and the resumable campaign (both 2026-09-13) it must also have been
    tuned under the current road permissions, and the campaign must be complete: all
    CONTRACT_TUNING_TRIALS (20 since 2026-09-15) trials evaluated. A campaign still in progress
    writes no weight file at all; this is the second line.
    """
    import yaml as _yaml
    from pathlib import Path as _Path

    from emergency_manager import ROAD_PERMISSION_POLICY
    from metrics.tuning_objective import OBJECTIVE_WEIGHTS

    root = _Path(__file__).resolve().parent
    config = _yaml.safe_load((root / "config" / "scenario.yaml").read_text(encoding="utf-8"))
    problems = []
    if document.get("evaluator") != TUNER_EVALUATOR:
        problems.append("not produced by the corrected tuner (no P3RuntimeCoordinator evaluator)")
    if document.get("objective_weights") != OBJECTIVE_WEIGHTS:
        problems.append("not scored with the finalized objective (contract 13.17)")
    if document.get("road_permissions") != ROAD_PERMISSION_POLICY:
        problems.append("not tuned under the current road permissions (PD-1 question 2)")
    if document.get("campaign_complete") is not True or \
            int(document.get("trials_evaluated", 0)) < CONTRACT_TUNING_TRIALS:
        problems.append(f"not a complete campaign ({document.get('trials_evaluated')} of "
                        f"{CONTRACT_TUNING_TRIALS} trials evaluated)")
    if document.get("seed") != int(config["seeds"]["tuning"]):
        problems.append(f"seed {document.get('seed')} is not the tuning seed {config['seeds']['tuning']}")
    if float(document.get("sim_time", -1)) != float(config["tuning"]["duration_seconds"]):
        problems.append(f"sim_time {document.get('sim_time')} is not the contracted "
                        f"{config['tuning']['duration_seconds']} s")
    if problems:
        raise ValueError(f"{path}: not a valid tuned weight set - " + "; ".join(problems))


def load_weights_file(path, require_campaign: bool = True) -> Dict[str, float]:
    """Read a tuned weight set written by `scripts/tune_e3_weights.py` (audit C10).

    Accepts either the tuner's own file, which nests the values under "best_weights", or a
    plain mapping. Every one of the five components must be present and numeric, and no
    unknown name is allowed: a typo must fail loudly rather than silently leave a component
    at its default. Nothing calls this unless a weights file is configured, so the default
    run is unchanged.

    2026-09-13: a tuner file must also prove it came from the real campaign (see
    `_check_campaign_provenance`), and nothing under results/_invalid/ loads at all.
    `require_campaign=False` is for diagnostics that inspect a smoke study's file; the runtime
    never passes it. A plain mapping is a deliberate hand-written choice and is recorded as the
    run's `weights_source`.
    """
    import json as _json
    from pathlib import Path as _Path

    # 2026-09-13: nothing quarantined may ever reach the router (results/_invalid/README.md).
    if "_invalid" in _Path(path).resolve().parts:
        raise ValueError(f"{path}: quarantined in results/_invalid/ - never a valid weight set")
    document = _json.loads(_Path(path).read_text(encoding="utf-8"))
    if require_campaign and isinstance(document, dict) and "best_weights" in document:
        _check_campaign_provenance(path, document)
    values = document.get("best_weights", document) if isinstance(document, dict) else None
    if not isinstance(values, dict):
        raise ValueError(f"{path}: no weight mapping found")
    unknown = sorted(set(values) - set(COMPONENT_NAMES))
    missing = sorted(set(COMPONENT_NAMES) - set(values))
    if unknown or missing:
        raise ValueError(f"{path}: unknown components {unknown}, missing components {missing}")
    return {name: float(values[name]) for name in COMPONENT_NAMES}


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

    weights: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_WEIGHTS))
    scout_memory: Dict[str, float] = field(default_factory=dict)

    # Audit A6 follow-up: how long the BCO scout memory lasts, in simulated seconds.
    bco_half_life_s: float = BCO_MEMORY_HALF_LIFE_S

    # The emergency layer's routing half. False is the control arm of the matched
    # ablation: E3 routes normal traffic in both arms, and only the emergency mechanism
    # differs. The term is forced to the NEUTRAL constant rather than to zero, because a
    # constant is added to every candidate and therefore cannot change the argmax - which
    # is what makes "off" provably the same router rather than a third, different one.
    # Setting it to zero would be a different algorithm.
    emergency_layer_enabled: bool = True

    def weight_of(self, component: str) -> float:
        return self.weights.get(component, DEFAULT_WEIGHTS[component])

    @staticmethod
    def component_values(context: HybridContext) -> Dict[str, float]:
        """The five raw component values of one candidate, by name."""

        return {
            "pheromone": context.pheromone,
            "heuristic": context.heuristic,
            "protocol": context.protocol_weight,
            "bco": context.bco_exploration,
            # This used to be `max(0.0, context.velocity)`, on the reasoning that a negative
            # PSO velocity is a signal to avoid an edge rather than a bonus. Audit A3 made
            # that clamp unnecessary - every component is min-max normalised across the
            # candidate set before it is weighted, so the lowest velocity in the set already
            # scores 0 and the highest 1, whatever the signs are. Once PSO's fitness became
            # a realised travel outcome the clamp became actively harmful: velocity is now a
            # rate of change and most roads at any moment are not improving, so measured
            # over 600 s velocity differed between candidates in 83.3% of selections but in
            # only 1.7% after the clamp, which flattened 112 of 140 candidate values to a
            # single 0.0 and put the term back where audit C8 found it.
            "velocity": context.velocity,
        }

    def normalise_candidates(
        self,
        candidates: Sequence[Tuple[str, HybridContext]],
    ) -> List[Dict[str, float]]:
        """Min-max each component across this candidate set, into [0, 1].

        Audit A3. The components were previously summed on their own scales and the total
        was clamped to [0, 1]. ACO pheromone grows without bound, so measured over 900 s
        36.9% of raw scores landed above the clamp (99th percentile 3.99) and 71% of
        selections were made at it. At the clamp every candidate scores exactly 1.0, the
        argmax is a tie, and the winner is whichever edge comes first in sorted order -
        so the router was, most of the time, not choosing at all. Worse, once the sum is
        saturated the emergency term cannot move it: the protocol weight changed the
        chosen edge in 1 of 193 selections.

        Normalising across the candidate set makes every component a comparable [0, 1]
        quantity for this decision, so the weights mean what they say and the weighted sum
        cannot saturate.
        """
        values = [self.component_values(context) for _, context in candidates]
        normalised: List[Dict[str, float]] = []
        spans = {}
        for name in COMPONENT_NAMES:
            column = [row[name] for row in values]
            spans[name] = (min(column), max(column))

        for row in values:
            scaled = {}
            for name in COMPONENT_NAMES:
                low, high = spans[name]
                width = high - low
                scaled[name] = (
                    CONSTANT_COMPONENT_VALUE if width <= 0.0 else (row[name] - low) / width
                )
            normalised.append(scaled)
        return normalised

    def score_candidates(
        self,
        candidates: Sequence[Tuple[str, HybridContext]],
    ) -> List[Tuple[str, float]]:
        """Score every candidate against the others. This is the production scorer.

        Dividing by the total weight keeps the result in [0, 1] whatever weights tuning
        picks, which is what lets the old clamp be dropped: a normalised component is at
        most 1, so the weighted mean is at most 1 and there is nothing to clamp. Note that
        the score is now a *relative* quantity - it says where a candidate stands among the
        edges available at this junction, not how good the edge is in absolute terms.
        """
        if not candidates:
            return []

        normalised = self.normalise_candidates(candidates)
        total_weight = sum(abs(self.weight_of(name)) for name in COMPONENT_NAMES) or 1.0

        scored = []
        for (edge, _), row in zip(candidates, normalised):
            score = sum(self.weight_of(name) * row[name] for name in COMPONENT_NAMES)
            scored.append((edge, round(score / total_weight, 6)))
        return scored

    def score_edge(self, context: HybridContext) -> float:
        """Score one candidate on its own, on the raw component scales.

        Audit A3: this is no longer the production scorer - `score_candidates` is, because
        a single edge cannot be normalised against a candidate set it does not have. This
        is kept for the unit tests and for diagnostics that need an absolute per-edge
        number, and it keeps the [0, 1] clamp that A3 removed from the selection path.
        """

        score = sum(
            self.weight_of(name) * value
            for name, value in self.component_values(context).items()
        )
        return round(min(1.0, max(0.0, score)), 6)

    def select_best_edge(
        self,
        candidates: Sequence[Tuple[str, HybridContext]],
    ) -> Tuple[Optional[str], float]:
        """Return the best edge and its score from a list of candidates."""

        if not candidates:
            return None, 0.0

        scored = self.score_candidates(candidates)
        best_edge, best_score = max(scored, key=lambda item: item[1])
        return best_edge, best_score

    def protocol_weight_for_edge(
        self,
        edge_id: str,
        messages: Iterable[ProtocolMessage],
        is_emergency_vehicle: bool = False,
        corridor_edge: Optional[str] = None,
    ) -> float:
        """Turn incoming protocol messages into a single edge-level weight.

        Audit A3, two defects fixed here.

        It used to **average** the weights of every message naming the edge. One ambulance
        broadcasting 0.05 ("get off this road") mixed with four ordinary vehicles at 0.5
        came out as 0.41, barely different from neutral, so the emergency signal was
        diluted by exactly the traffic that was supposed to move out of the way. The
        strongest signal must decide, so an ordinary receiver takes the **minimum** - the
        road it is most strongly told to avoid - and a responder takes the **maximum**, the
        corridor it is most strongly entitled to.

        It also returned 0.0 when no message named the edge but about 0.5 when a routine
        one did. Silence therefore looked like the worst possible road, which is a large
        discontinuity in the middle of the score for no physical reason. No information now
        means the neutral weight, the same as a routine message.

        2026-09-15, the clean design: a message carrying `yield_request` is skipped. See the branch below.

        2026-09-14, audit A1 and A5. `messages` never includes the deciding vehicle's own broadcast.
        A responder's claim on its own corridor is the explicit `corridor_edge` rule instead: the edge
        it is about to enter on its current route scores `RESPONDER_CORRIDOR_WEIGHT`. A message that
        names no next edge says nothing about any edge. It used to count for every edge, which
        flattened the term network-wide.
        """

        if not self.emergency_layer_enabled:
            return NEUTRAL_WEIGHT

        observed: List[float] = []
        if is_emergency_vehicle and corridor_edge is not None and edge_id == corridor_edge:
            observed.append(RESPONDER_CORRIDOR_WEIGHT)
        for message in messages:
            if message.next_edge != edge_id:
                continue

            if message.emergency_flag:
                if is_emergency_vehicle:
                    # A responder may use the corridor it is heading down.
                    observed.append(
                        EMERGENCY_CORRIDOR_WEIGHT + min(0.1, message.priority_level / 40.0)
                    )
                else:
                    # Audit P8. This used to add 0.8-1.0 here, the highest weight of any
                    # branch, which made ordinary vehicles *prefer* the edge an emergency
                    # vehicle had just announced it was taking - the protocol steered
                    # traffic into the responder's path. Measured effect: the incident edge
                    # held 12.6 vehicles on average in a P3-enabled run against 0.2 in the
                    # baseline. Emergency information must push ordinary traffic away.
                    observed.append(ORDINARY_AVOID_EMERGENCY_WEIGHT)
            elif message.yield_request:
                # The clean design (user decision, 2026-09-15). The flag means only "I am yielding". It says
                # nothing about this edge, so it has no routing effect, for ordinary cars and responders
                # alike.
                # It used to make every receiver score the sender's next edge 0.3 (P8's "mild avoid")
                # while the flag was set by mere proximity to an incident. The measured effect ignored
                # direction: it rescued cars trapped behind a closure, and misfired when the sender was
                # already escaping. That rescue is now E3's own reachability check
                # (`route_utils.reachable_candidates`), and the emergency-path signal stays the
                # responder's broadcast above.
                continue
            else:
                observed.append(NEUTRAL_WEIGHT + min(0.1, message.priority_level / 10.0))

        if not observed:
            return NEUTRAL_WEIGHT

        combined = max(observed) if is_emergency_vehicle else min(observed)
        return round(min(1.0, max(0.0, combined)), 6)

    def decay_scout_memory(self, seconds: float = 1.0) -> None:
        """Fade exploration memory on the clock. Call once per simulated second.

        This used to subtract a flat 0.02 once per *routing decision*, and was called from
        `p3_runtime._apply_hybrid_decision`. Decisions happen about 5.77 times per simulated
        second in this scenario, so the real decay was 0.115 per second and a chosen edge
        was forgotten after **1.30 s** - shorter than one reroute interval, so no vehicle
        ever reached its next decision with the memory intact. Worse, the rate depended on
        how busy the router was: at a tenth of the demand the same code would remember ten
        times longer. That is the same event-driven defect audit A10 removed from the
        pheromone table.

        The decay is now exponential with a stated half-life, matching what A10 gave
        pheromone so there is one decay concept in the design rather than two. An entry that
        has faded to nothing is dropped, so a decayed edge and a never-visited edge both
        read 0.0 and score the same - no cliff at the boundary.
        """
        if not self.scout_memory:
            return

        rate = 0.5 ** (max(seconds, 0.0) / self.bco_half_life_s)
        for edge_id in list(self.scout_memory.keys()):
            faded = self.scout_memory[edge_id] * rate
            if faded < 1e-3:
                self.scout_memory.pop(edge_id, None)
            else:
                self.scout_memory[edge_id] = faded

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
        corridor_edge: Optional[str] = None,
    ) -> HybridContext:
        """Create a scoring context using local and protocol information."""

        protocol_weight = self.protocol_weight_for_edge(
            edge_id, messages, is_emergency_vehicle, corridor_edge=corridor_edge)
        # Audit A6. `scout_memory` correctly records how recently and how often an edge has
        # been used, but this used to be read straight into the score, where a *higher*
        # value makes an edge *more* attractive. The edge just chosen receives the largest
        # increment, so it became the most attractive next time - reinforcement, which is
        # what pheromone already does. design_phase.md section 2.1 asks BCO for the
        # opposite: "a more explicit exploration mechanism... helps the system discover
        # alternative routes that the other two methods may not explore aggressively
        # enough."
        #
        # The counter is left exactly as it is and only its interpretation is inverted, so
        # a less recently used edge scores higher. scout_memory is bounded to [0, 1] by
        # construction (min/max in update and decay, measured range 0.0000-0.7600), so the
        # inverted value is bounded too. An edge that has decayed out of the table reads
        # the same as one never visited - both fully explorable - so there is no cliff at
        # the boundary.
        exploration = 1.0 - self.scout_memory.get(edge_id, 0.0)
        return HybridContext(
            pheromone=pheromone,
            heuristic=heuristic,
            protocol_weight=protocol_weight,
            bco_exploration=exploration,
            velocity=velocity,
        )
