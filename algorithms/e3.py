"""E3 Hybrid (ACO + PSO + BCO + protocol) - the comparison-table entry point.

**This module no longer contains an algorithm.** Work order item 2 (audit C8 remainder, A7)
removed the second E3 that used to live here. It is now a thin adapter over
`algorithms/e3_core.E3Core`, which is the same object the P3 runtime drives, over the same
`algorithms/aco_pso` pheromone and PSO state.

Why that mattered: this file used to hold its own `pheromone`, `velocity`, `personal_best`
and `scout_memory` dictionaries and its own copy of the decision logic. Every repair made
during the 2026-09-08 and 2026-09-09 sessions - bounded pheromone (A10), the destination-aware
heuristic, corrected BCO exploration (A6), the BCO memory clock, outcome-based pheromone and
PSO learning - landed in the runtime copy only. Two different algorithms were both being
called "E3", one of them in the six-algorithm comparison table and the other in the headline
P3 results. Three defects that were fixed elsewhere survived here in particular:

* `initialize()` set `scout_memory[edge] = 0.0` for **every** edge in the network, so the BCO
  term was identically 0.0 for every candidate and could never change a decision (audit A7);
* the PSO velocity was computed from the pheromone value, and the bests were raised from the
  pheromone of whatever edge won;
* `update_pheromone` multiplied all 21,385 entries by `(1 - RHO)` on every applied reroute,
  so the decay rate depended on how often the router acted.

None of that exists any more. The routing decision made through this module is now the same
decision, from the same state, that a P3 run makes.

**What a caller of this module gets, and what it does not.** No V2V messages are passed, so
`protocol_weight_for_edge` returns the neutral constant for every candidate - and a constant
added to every candidate cannot change the argmax. The comparison table therefore runs E3 with
the emergency term switched off, which is exactly the control arm of the matched ablation. It
is a deliberate property, not an omission: the six-algorithm table compares *routing*, and the
emergency layer is measured separately.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence

import traci

from algorithms.aco_pso import (
    ALPHA,
    BETA,
    C1,
    C2,
    Q,
    RHO,
    TAU_INIT,
    TAU_MAX,
    TAU_MIN,
    VMAX,
    W,
    deposit,
    personal_best,
    pheromone,
    velocity,
)
from algorithms.e3_core import E3Core
from algorithms.route_utils import (
    build_destination_valid_route,
    get_outgoing_edges,
    reachable_candidates,
    remaining_route,
)
from p3_hybrid_router import HybridContext, HybridRouter, ProtocolMessage

# The single shared decision maker. Created on first use so importing this module does not
# require a TraCI connection.
_core: Optional[E3Core] = None


def get_core() -> E3Core:
    """The shared E3Core, created on first use."""
    global _core
    if _core is None:
        # `reset_state=False`: whoever owns the run resets the shared tables, and a caller
        # that lazily creates the core mid-run must not wipe what is already there.
        _core = E3Core(emergency_layer=True, reset_state=False)
    return _core


def initialize() -> None:
    """Start a clean run.

    The old version seeded all 21,385 edges with `pheromone = 1.0`, `velocity = 0.0`,
    `personal_best = 1.0` and `scout_memory = 0.0`. Seeding is now not merely unnecessary but
    wrong on two counts: audit A10 made an absent edge read exactly `TAU_INIT`, so the table
    and its default already agree, and seeding `scout_memory` to 0.0 everywhere is what made
    the BCO term a constant (audit A7).
    """
    global _core
    _core = E3Core(emergency_layer=True, reset_state=True)


def reset_state() -> None:
    """Clear all shared search state between tuning trials or runs."""
    initialize()


# ===============================
# Compatibility surface
# ===============================
#
# `pheromone`, `velocity` and `personal_best` above are imported from `algorithms.aco_pso`
# and are the *same dictionary objects* the runtime uses, so existing code that inspects or
# clears them still works and now sees the shared state rather than a private copy.


def get_scout_memory() -> Dict[str, float]:
    """BCO's exploration memory. A function, not a module dictionary, because it now lives
    on the shared router rather than in this file - anything that used to read or clear
    `e3.scout_memory` must go through `reset_state()` or this accessor."""
    return get_core().router.scout_memory


def heuristic(edge_id: str) -> float:
    """The congestion half of the heuristic, kept for tests and diagnostics.

    Note this is only *half* of what E3 now scores. The routing decision blends it with the
    distance to the vehicle's destination, per `design_phase.md` section 2.3; see
    `E3Core.blend_heuristic`.
    """
    try:
        return E3Core.congestion_suitability(edge_id)
    except traci.TraCIException:
        return 0.5


def decay_scout_memory(seconds: float = 1.0) -> None:
    """Fade BCO exploration memory. Now on the clock, at a 30 s half-life.

    It used to subtract a flat 0.02 once per routing decision, which in this scenario meant
    an edge was forgotten after 1.3 simulated seconds - less than one reroute interval - and
    made the memory's lifetime depend on how busy the router was.
    """
    get_core().router.decay_scout_memory(seconds=seconds)


def update_exploration_memory(candidate_edges, selected_edge) -> None:
    """Record which edge was taken, for BCO."""
    get_core().router.update_exploration_memory(candidate_edges, selected_edge)


def build_fallback_protocol_messages(vehicle_id: str, current_edge: str) -> List[ProtocolMessage]:
    """Deprecated: returns nothing, deliberately.

    This used to fabricate a protocol message *from the vehicle itself* and feed it to the
    scorer, so a vehicle received its own broadcast as though a neighbour had sent it. For an
    emergency vehicle that awarded its own next edge the corridor weight; for everyone else it
    contributed a near-neutral value. Either way it was not V2V information, and the
    fallback that `design_phase.md` section 2.5 actually asks for is "use local observations
    instead" - which is the congestion half of the heuristic, already in the score.

    With no messages the protocol term is the neutral constant for every candidate and cannot
    change which edge wins, which is the correct behaviour for a routing-only comparison.
    """
    return []


def select_route(
    veh_id: str,
    current_edge: str,
    protocol_messages: Optional[Iterable[ProtocolMessage]] = None,
) -> List[str]:
    """Choose a route for `veh_id`, using the one shared E3 implementation.

    Returns the vehicle's *remaining* route when there is nothing better to do, so the caller
    can always apply the result safely and applying it is a genuine no-op.

    That word "remaining" is not a detail. This used to return the full route including the
    edges already travelled. `set_route_checked` compares a proposal against the remaining
    route, so a full route never matched, and the no-change case fell through to a real
    `traci.vehicle.setRoute` call carrying the travelled prefix. SUMO accepts such a route -
    it does contain the current edge - re-anchors the vehicle on it, and the result is then
    counted as a silent rejection. The P3 entry point never did this, so the two paths
    diverged physically after about ten minutes even though every routing decision agreed:
    measured at 900 s, the first 60 decisions were identical and the arms then differed in
    *which vehicle* was being decided for, not in what was chosen.
    """
    core = get_core()
    try:
        current_route = list(traci.vehicle.getRoute(veh_id))
    except traci.TraCIException:
        return []

    def unchanged():
        return remaining_route(veh_id) or current_route

    outgoing = get_outgoing_edges(current_edge, veh_id=veh_id)
    if not outgoing or len(current_route) < 2:
        return unchanged()
    # The clean design (2026-09-15), exactly as in E3Core.decide: one E3, one candidate rule.
    outgoing = reachable_candidates(veh_id, outgoing, current_route[-1])

    # PD-1 (2026-09-13): responder status from the emergency manager's explicit flag, as in
    # the P3 path - never from the vehicle type. Result-neutral on this path today, because
    # it passes no V2V messages and the protocol term is then neutral for every candidate.
    from emergency_manager import get_emergency_manager

    is_emergency = get_emergency_manager().is_active_responder(veh_id)

    selected_edge, _ = core.select_edge(
        veh_id,
        outgoing,
        current_route[-1],
        protocol_messages if protocol_messages is not None else (),
        is_emergency_vehicle=is_emergency,
    )
    if not selected_edge or selected_edge == current_edge:
        return unchanged()

    return build_destination_valid_route(veh_id, current_route, current_edge, selected_edge)


def update_pheromone(route: List[str]) -> None:
    """Deprecated: the flat, immediate deposit this used to perform is gone.

    Pheromone is no longer laid down when a route is chosen. It is laid down when the route's
    realised travel-time efficiency has been measured, scaled by it (work order item 1), which
    is what `design_phase.md` section 2.5 asks for: "pheromone updates from **successful**
    route choices".

    Calling this is harmless and does nothing. The caller must instead tell the core when a
    route was applied - `E3Core.decide` does that by itself, and `select_route` callers should
    call `note_applied_route` below.
    """
    return


def note_applied_route(vehicle_id: str, sim_time: float, route: Sequence[str]) -> None:
    """Tell the learner that `route` was actually applied, so its outcome gets measured.

    Delegates to `E3Core.note_applied_route`, so a window this closes is learned from exactly as
    in the P3 path (audit A2, 2026-09-14).
    """
    get_core().note_applied_route(vehicle_id, sim_time, list(route))


def step(sim_time: float) -> None:
    """Per-simulation-step upkeep: the three decays and outcome settlement."""
    get_core().step(sim_time)


def set_hybrid_weights(weights: Dict[str, float]) -> None:
    """Update the scoring weights, for tuning."""
    get_core().router.weights.update(weights)


def get_hybrid_router() -> HybridRouter:
    """The router carrying the weights and the BCO scout memory."""
    return get_core().router


def get_state_summary() -> Dict[str, object]:
    """A summary of E3's state, for monitoring and debugging."""
    return get_core().state_summary()
