"""The one E3 implementation. ACO + PSO + BCO + protocol, in a single place.

Work order item 2 (audit C8 remainder, A7). Until now there were **two** algorithms both
called E3. The six-algorithm comparison table ran `algorithms/e3.py`; the P3 results ran the
logic inside `p3_runtime.P3RuntimeCoordinator._apply_hybrid_decision`. They kept separate
pheromone, velocity, personal-best and scout state, and every repair of the 2026-09-08 and
2026-09-09 sessions landed in the runtime copy only, so by the time this was written the two
had diverged completely:

| | `algorithms/e3.py` (old) | `p3_runtime` (validated) |
|---|---|---|
| pheromone | unbounded, decayed per reroute over all 21,385 edges | bounded [0.1, 5.0], 300 s half-life on the clock (A10) |
| pheromone deposit | flat `Q` on every edge, at decision time | scaled by realised travel-time efficiency (item 1) |
| PSO fitness | the pheromone value | realised travel-time efficiency (item 1) |
| PSO bests | raised from pheromone on selection | raised after the velocity step, decaying (item 1) |
| heuristic | congestion only | congestion **and** distance to destination (design 2.3) |
| BCO | `scout_memory` read straight in, and `initialize()` set it to 0.0 for every edge, so the term was identically zero (audit A7) | inverted to exploration (A6), decayed on the clock at a 30 s half-life |
| score | A3 normalisation, via the shared `HybridRouter` | same |
| emergency layer | always on, fed a vehicle's own fabricated message | ON/OFF gate, fed real V2V messages |

Everything in the right-hand column is preserved here and nothing is re-tuned. `e3.py` is now
a thin adapter over this class, and `P3RuntimeCoordinator` delegates to it, so there is one
implementation, one state, and one set of parameters.

Two callers, one algorithm, and the difference between them is only what they can *tell* it:

* the P3 runtime passes real V2V messages, the set of vehicles actively responding to an
  incident, and whether the emergency layer is on;
* the comparison table passes no messages at all, which makes the protocol term the neutral
  constant for every candidate - and a constant cannot change an argmax, so the routing is
  provably the same router with the emergency term switched off. That is the same argument
  the ablation's control arm rests on.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import traci

from algorithms.aco_pso import (
    advance_velocities,
    decay_fitness,
    deposit,
    evaporate,
    pheromone,
    record_outcome,
    reset_pheromone,
    velocity_of,
)
from algorithms.route_outcomes import RouteOutcomeTracker
from algorithms.route_utils import (
    build_destination_valid_route,
    get_outgoing_edges,
    reachable_candidates,
    set_route_checked,
)
from p3_hybrid_router import HybridContext, HybridRouter
from p3_hybrid_router import load_weights_file, ProtocolMessage

# How the two halves of the heuristic are mixed: 0.0 is congestion only (the behaviour
# before the distance half was restored), 1.0 is distance only, which would make E3 a
# shortest-path router with extra steps. An even split gives both halves the equal footing
# `design_phase.md` section 2.3 implies by naming them together.
HEURISTIC_PROGRESS_BLEND = 0.5

# Which edges a realised outcome reinforces (work order item 1).
#
#   "planned"   - every edge of the route that was applied, scaled by the quality measured
#                 on its opening stretch. The classic ACO shape, where a whole constructed
#                 solution is reinforced in proportion to its cost.
#   "travelled" - only the roads physically driven during the observation window.
#
# Settled by measurement at two seeds, not by argument: "travelled" lost 445 completed trips
# and teleported 103 times at seed 20260721, and halved pheromone's ability to tell candidate
# edges apart (52.8% against 86.1%), because it starves the table it is meant to improve.
DEPOSIT_SCOPE_DEFAULT = "planned"



def configured_weights_file():
    """Path of the tuned weight set to load, or None to keep DEFAULT_WEIGHTS (audit C10).

    `P3_E3_WEIGHTS` wins, so a tuning trial can point at its own file without editing config.
    Otherwise `tuning.weights_file` in config/scenario.yaml, when it names an existing file.
    """
    import os
    from pathlib import Path

    override = os.environ.get("P3_E3_WEIGHTS")
    if override:
        return Path(override)
    try:
        import yaml

        root = Path(__file__).resolve().parent.parent
        with open(root / "config" / "scenario.yaml", "r", encoding="utf-8") as handle:
            configured = (yaml.safe_load(handle).get("tuning") or {}).get("weights_file")
        if configured:
            candidate = root / str(configured)
            return candidate if candidate.is_file() else None
    except Exception:
        return None
    return None


class E3Core:
    """One E3 decision maker: scoring, selection, and outcome-based learning.

    The ACO pheromone table and the PSO velocity/fitness state live in
    `algorithms.aco_pso` as module state, shared by every caller, which is what makes this
    one algorithm rather than two that look alike. BCO scout memory lives on the
    `HybridRouter` this class owns.
    """

    def __init__(
        self,
        emergency_layer: bool = True,
        deposit_scope: str = DEPOSIT_SCOPE_DEFAULT,
        reset_state: bool = True,
    ) -> None:
        if reset_state:
            # Audit A10: clear the pheromone table so one run cannot inherit another's
            # search state. Two arms compared in the same process would otherwise not start
            # equal.
            reset_pheromone()
        self.router = HybridRouter(emergency_layer_enabled=emergency_layer)
        # Audit C10: until 2026-09-12 nothing ever loaded a tuned weight set, so every run
        # used DEFAULT_WEIGHTS whatever tuning produced. A weights file is used only when one
        # is configured - P3_E3_WEIGHTS, or `tuning.weights_file` in config/scenario.yaml - so
        # with neither set this is exactly the previous behaviour.
        self.weights_source = "DEFAULT_WEIGHTS"
        configured = configured_weights_file()
        if configured is not None:
            self.router.weights = load_weights_file(configured)
            self.weights_source = str(configured)
        self.emergency_layer = emergency_layer
        self.deposit_scope = deposit_scope
        # Work order item 1: what each applied reroute actually achieved. Pheromone deposits
        # and PSO fitness both come from here, so nothing is reinforced until it has been
        # driven and measured.
        self.outcomes = RouteOutcomeTracker()
        # Edge geometry for the distance half of the heuristic. One TraCI call per distinct
        # edge for the whole run; the network has 21,385 of them and a run touches far fewer.
        self._edge_end_cache: Dict[str, Optional[tuple]] = {}
        # Set by the caller each step when it knows; empty means "nobody is responding",
        # which is correct for the comparison table.
        self.active_responders: set = set()

    # ------------------------------------------------------------------
    # per-step upkeep
    # ------------------------------------------------------------------
    def decay(self, seconds: float = 1.0) -> None:
        """Fade all three of E3's memories. Call once per simulated second.

        Each of them used to run per routing decision or per applied reroute, which made
        their real rate depend on how busy the router happened to be - audit A10 for
        pheromone, the A6 follow-up for BCO's scout memory, work order item 1 for PSO's
        personal and global bests. All three now run on the clock.

        The half-lives are deliberately different and each was set separately: 300 s for
        pheromone and for PSO's bests, matching how fast this network's congestion actually
        changes, and 30 s for BCO, which is one reroute interval.

        PSO's velocities also advance here, once per step for every edge scored in the step
        before (audit S5, 2026-09-14). Scoring only reads them.
        """
        evaporate(seconds=seconds)
        decay_fitness(seconds=seconds)
        advance_velocities()
        self.router.decay_scout_memory(seconds=seconds)

    def step(self, sim_time: float) -> None:
        """Everything that happens on the clock: the decays, then outcome settlement.

        Callers that need to observe settlement separately - the P3 runtime does, because a
        diagnostic replaces its settlement method - should call `decay` and
        `settle_outcomes` themselves instead of this.
        """
        self.decay(seconds=1.0)
        self.settle_outcomes(sim_time)

    def settle_outcomes(self, sim_time: float) -> None:
        """Turn finished observation windows into pheromone and PSO fitness.

        This is where "learn from actual outcomes" (design_phase.md 2.5) is implemented.
        Two distinct things happen from one measurement:

        * the **pheromone deposit** is scaled by the route's realised travel-time
          efficiency, so a road that delivered free-flowing traffic is reinforced several
          times as strongly as one that delivered a jam - the classic `Q / L`, replacing a
          flat `Q` that reinforced every applied route equally;
        * the **PSO fitness** of each road actually driven is updated from the same
          measurement, which is what stops PSO from being a function of pheromone.

        PSO fitness always comes from the roads the vehicle really drove, whatever
        `deposit_scope` says: a road nobody has driven has no realised outcome to report.
        """
        for outcome in self.outcomes.update(sim_time):
            self._learn_from(outcome)

    def _learn_from(self, outcome, update_mean: bool = True) -> None:
        """Turn one settled observation window into PSO fitness and a pheromone deposit.

        The one place this happens, however the window closed: its horizon, the vehicle leaving,
        or a new decision replacing the route it was judging (audit A2, 2026-09-14).

        `update_mean=False` keeps the outcome out of the running mean quality and changes nothing
        else: same fitness update, same deposit, scaled by the same mean. `note_applied_route`
        passes it (the "mean protected" rule; see there).
        """
        record_outcome(outcome.travelled_edges, outcome.quality, update_mean=update_mean)
        target = (
            outcome.planned_route
            if self.deposit_scope == "planned"
            else outcome.travelled_edges
        )
        if target:
            deposit(target, quality=outcome.quality)

    def note_applied_route(self, vehicle_id: str, sim_time: float, route: Sequence[str]) -> None:
        """Start watching a route that was just applied.

        If it replaces a route that was still being watched, the tracker closes that window
        and returns its outcome. Audit A2: both production paths used to drop that return
        value, so the outcome was measured and counted but never learned from. In campaign
        2026-09-14b that was 17-23% of all measured outcomes.

        That outcome updates PSO fitness and deposits exactly like any other, but stays out of
        the running mean quality (user decision 2026-09-14, "mean protected"). The trigger reroutes
        only slow cars, so a window closed this way is a shorter, congested subsample, 3.4-11.3
        times lower in quality than a full window in every diagnosis run. The mean is the
        yardstick for a typical full window. Mixed in, these windows lowered it in all three weight
        sets tried and inflated every deposit normalised by it. With trial 0's weights that raised
        pheromone saturation from 39 to 331 edges and J by 0.21. Kept out, the rule was never worse
        than dropping the outcome in any weight set. Evidence:
        results/e3_component_validation_post_audit_fixes_2026-09-14/a2_diagnosis_deterministic_pso/.
        """
        settled = self.outcomes.record_decision(vehicle_id, sim_time, route)
        if settled is not None:
            self._learn_from(settled, update_mean=False)

    # ------------------------------------------------------------------
    # the decision
    # ------------------------------------------------------------------
    def score_candidates(
        self,
        vehicle_id: str,
        outgoing_edges: Sequence[str],
        destination_edge: str,
        messages: Iterable[ProtocolMessage],
        is_emergency_vehicle: bool = False,
        corridor_edge: Optional[str] = None,
    ) -> List[Tuple[str, HybridContext]]:
        """Build the scored context for every candidate edge at this junction.

        `corridor_edge` is the edge an active responder is about to enter; the router gives it the
        responder corridor weight (2026-09-14; see `HybridRouter.protocol_weight_for_edge`).
        """
        # The heuristic is computed for the whole candidate set at once, because its two
        # halves - congestion and distance to destination - are scaled against the other
        # candidates before being mixed. See blend_heuristic.
        heuristics = self.blend_heuristic(
            [self.congestion_suitability(edge) for edge in outgoing_edges],
            [self.progress_suitability(edge, destination_edge) for edge in outgoing_edges],
        )

        messages = list(messages)
        candidates: List[Tuple[str, HybridContext]] = []
        for candidate_edge, heuristic in zip(outgoing_edges, heuristics):
            pheromone_value = pheromone.get(candidate_edge, 1.0)
            # Audit C8. This used to be `velocity.get(candidate_edge, 0.0)` against a
            # dictionary nothing in the P3 path ever wrote, so the PSO term was 0.0 for
            # every candidate - identical, therefore incapable of changing which edge wins.
            # Audit S5 (2026-09-14): it then called `update_velocity` here, once per scoring,
            # so the value depended on how many vehicles scored the edge and in what order. It
            # is now a read; the velocity advances once per step in `decay`.
            velocity_value = velocity_of(candidate_edge)
            candidates.append((
                candidate_edge,
                self.router.build_context(
                    pheromone=pheromone_value,
                    heuristic=heuristic,
                    edge_id=candidate_edge,
                    messages=messages,
                    velocity=velocity_value,
                    is_emergency_vehicle=is_emergency_vehicle,
                    corridor_edge=corridor_edge,
                ),
            ))
        return candidates

    def select_edge(
        self,
        vehicle_id: str,
        outgoing_edges: Sequence[str],
        destination_edge: str,
        messages: Iterable[ProtocolMessage],
        is_emergency_vehicle: bool = False,
        corridor_edge: Optional[str] = None,
    ) -> Tuple[Optional[str], int]:
        """Pick the next edge. Returns it and how many candidates were scored."""
        candidates = self.score_candidates(
            vehicle_id, outgoing_edges, destination_edge, messages, is_emergency_vehicle,
            corridor_edge=corridor_edge,
        )
        if not candidates:
            return None, 0

        selected_edge, _ = self.router.select_best_edge(candidates)
        if selected_edge:
            # PSO's personal and global bests are raised inside `update_velocity`, after the
            # velocity has been computed - the textbook ordering, and what lets a velocity
            # mean "this road improved" rather than always being zero or negative. Since S5
            # that iteration runs once per step in `advance_velocities`, not here.
            self.router.update_exploration_memory(outgoing_edges, selected_edge)
        return selected_edge, len(candidates)

    def decide(
        self,
        vehicle_id: str,
        sim_time: float,
        messages: Optional[Iterable[ProtocolMessage]] = None,
        is_emergency_vehicle: bool = False,
        corridor_edge: Optional[str] = None,
    ) -> dict:
        """One complete routing decision, from candidates to an applied route.

        Returns a small report so a caller can record its own metrics without re-deriving
        anything: `{"applied", "selected_edge", "candidates", "route", "reason"}`.
        """
        report = {
            "applied": False,
            "selected_edge": None,
            "candidates": 0,
            "route": None,
            "reason": "",
        }
        try:
            current_edge = traci.vehicle.getRoadID(vehicle_id)
            outgoing_edges = get_outgoing_edges(current_edge, veh_id=vehicle_id)
            if not outgoing_edges:
                report["reason"] = "no_candidates"
                return report

            current_route = list(traci.vehicle.getRoute(vehicle_id))
            if len(current_route) < 2:
                report["reason"] = "no_route"
                return report

            # The clean design (2026-09-15): while a closure is on, a road the vehicle cannot reach its
            # destination from is not a candidate. If none can, the list is kept.
            outgoing_edges = reachable_candidates(vehicle_id, outgoing_edges, current_route[-1])

            selected_edge, scored = self.select_edge(
                vehicle_id,
                outgoing_edges,
                current_route[-1],
                messages if messages is not None else (),
                is_emergency_vehicle=is_emergency_vehicle,
                corridor_edge=corridor_edge,
            )
            report["candidates"] = scored
            report["selected_edge"] = selected_edge

            if not selected_edge:
                report["reason"] = "no_selection"
                return report
            if selected_edge == current_edge:
                report["reason"] = "stay_on_edge"
                return report

            # Audit C1: this used to build current_route[:index + 1] + [selected_edge],
            # which moved the vehicle's destination to the end of the selected edge instead
            # of rerouting it. Contract section 5 requires a destination-valid route, and
            # set_route_checked refuses to apply anything that changes the destination.
            new_route = build_destination_valid_route(
                vehicle_id, current_route, current_edge, selected_edge
            )
            report["route"] = new_route

            if set_route_checked(vehicle_id, new_route, current_route):
                report["applied"] = True
                # The deposit used to happen here, flat and immediate: Q = 1.0 on every edge
                # of the new route the moment it was applied, before the vehicle had driven
                # a metre of it. An edge was reinforced for being *chosen*. The route is now
                # watched instead, and reinforced when its realised quality is known.
                self.note_applied_route(vehicle_id, sim_time, new_route)
            else:
                report["reason"] = "rejected"
        except traci.TraCIException:
            report["reason"] = "traci_error"
        return report

    # ------------------------------------------------------------------
    # the two halves of the heuristic
    # ------------------------------------------------------------------
    @staticmethod
    def congestion_suitability(edge_id: str) -> float:
        """The congestion half of the heuristic: emptier roads score higher."""
        try:
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
        except traci.TraCIException:
            vehicle_count = 0
        return 1.0 / (1.0 + max(vehicle_count, 0))

    def edge_end_point(self, edge_id: str):
        """Coordinates of the far end of an edge, cached. None if SUMO will not say."""
        if edge_id in self._edge_end_cache:
            return self._edge_end_cache[edge_id]

        point = None
        try:
            shape = traci.lane.getShape(f"{edge_id}_0")
            if shape:
                point = shape[-1]
        except traci.TraCIException:
            point = None
        self._edge_end_cache[edge_id] = point
        return point

    def progress_suitability(self, edge_id: str, destination_edge: str) -> float:
        """The distance half of the heuristic: roads that end closer to the destination
        score higher.

        The straight-line distance from the *far end* of the candidate edge - where the
        vehicle will actually be once it has driven it - to the end of its destination
        edge. Straight-line rather than a routed distance because this runs tens of
        thousands of times per simulated hour, and it is the same admissible estimate A*
        uses. Returns the neutral 0.5 when SUMO will not give a shape for either edge, so
        a missing geometry cannot silently favour or punish a candidate.
        """
        here = self.edge_end_point(edge_id)
        there = self.edge_end_point(destination_edge)
        if here is None or there is None:
            return 0.5
        metres = math.dist(here, there)
        return 1.0 / (1.0 + metres / 1000.0)

    @staticmethod
    def blend_heuristic(congestion: Sequence[float], progress: Sequence[float]) -> List[float]:
        """Combine the two halves of the heuristic across one candidate set.

        `design_phase.md` section 2.3 defines the heuristic as "short-term local
        suitability such as lower congestion **and shorter distance**". The distance half
        was specified but never implemented, so E3 chose its next edge without any
        reference to where the vehicle was going, and `build_destination_valid_route` then
        repaired a path from wherever that landed. Measured consequence: 87.8% of applied
        reroutes lengthened the remaining trip, by 229 m on average.

        Each half is min-max scaled across the candidate set before they are mixed, for the
        same reason audit A3 scales the five scoring components: the two halves live on
        different numeric ranges, and on a long trip every candidate has almost the same
        absolute distance to the destination, so an unscaled mix would let congestion
        swamp the distance signal entirely. Scaling first gives both halves an equal say in
        the ranking. A half that is identical for every candidate contributes the neutral
        constant and cannot move the choice.
        """
        def scaled(values: Sequence[float]) -> List[float]:
            low, high = min(values), max(values)
            width = high - low
            if width <= 0.0:
                return [0.5] * len(values)
            return [(value - low) / width for value in values]

        congestion_scaled = scaled(congestion)
        progress_scaled = scaled(progress)
        return [
            (1.0 - HEURISTIC_PROGRESS_BLEND) * c + HEURISTIC_PROGRESS_BLEND * p
            for c, p in zip(congestion_scaled, progress_scaled)
        ]

    # ------------------------------------------------------------------
    # reporting
    # ------------------------------------------------------------------
    def state_summary(self) -> Dict[str, object]:
        """A summary of the shared search state, for monitoring and diagnostics."""
        from algorithms import aco_pso

        values = list(pheromone.values())
        return {
            "pheromone_edges": len(values),
            "avg_pheromone": sum(values) / len(values) if values else 0.0,
            "pheromone_at_ceiling": sum(1 for v in values if v >= aco_pso.TAU_MAX - 1e-9),
            "velocity_edges": len(aco_pso.velocity),
            "avg_velocity": (
                sum(aco_pso.velocity.values()) / len(aco_pso.velocity)
                if aco_pso.velocity else 0.0
            ),
            "edge_fitness_edges": len(aco_pso.edge_fitness),
            "outcomes_recorded": aco_pso.outcomes_recorded,
            "outcomes_in_mean": aco_pso.outcomes_in_mean,
            "quality_mean": aco_pso.quality_mean,
            "global_best_score": aco_pso.global_best_score,
            "scout_memory_edges": len(self.router.scout_memory),
            "avg_scout": (
                sum(self.router.scout_memory.values()) / len(self.router.scout_memory)
                if self.router.scout_memory else 0.0
            ),
            "hybrid_weights": dict(self.router.weights),
            "outcome_tracker": self.outcomes.stats(),
        }
