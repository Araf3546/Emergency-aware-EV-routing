"""The per-step routing loop of the comparison methods, in one place (2026-09-17, decisions B2 and B5).

Moved unchanged from `metrics/metrics_logger.py`, which now calls it, so that the V1 comparison runner
(`tune_e3_weights.evaluate_comparator`) drives Dijkstra, A*, ACO and ACO-PSO with exactly the same code instead of
a copy. The order of operations inside a simulation step is the one `metrics_logger` always used:

    emergency manager update (caller)
    -> before_step: tell the router which roads are closed, feed ACO-PSO its settled outcomes,
       advance E3's clock (legacy arm only), note each vehicle's type
    -> responder arrivals (caller)
    -> route_vehicles: the shared reroute trigger, then the method's own route choice, applied only through
       `route_utils.set_route_checked`

"e3" here is `metrics_logger`'s legacy E3 arm (no V2V messages). It is kept for that script and is not the E3 of
the thesis comparison, which runs through `P3RuntimeCoordinator` (see `tune_e3_weights.evaluate_weights`).
"""

import traci

from controller.reroute_logic import should_reroute, get_vehicle_type
from algorithms.aco import select_route as aco_select, update_pheromone
from algorithms.dijkstra import select_route as dijkstra_select
from algorithms.astar import select_route as astar_select
from algorithms.aco_pso import (
    select_route as aco_pso_select,
    update_pheromone as aco_pso_update,
    decay_fitness as aco_pso_decay_fitness,
    record_outcome as aco_pso_record_outcome,
)
from algorithms.route_outcomes import RouteOutcomeTracker
from algorithms.e3 import (
    select_route as e3_select,
    initialize as e3_initialize,
    note_applied_route as e3_note_applied,
    step as e3_step,
)
from algorithms.route_utils import set_blocked_edges, set_route_checked

# The four routing comparators of the V1 comparison (contract section 1 plus the ACO-PSO arm).
COMPARATORS = ("dijkstra", "astar", "aco", "aco_pso")
# Everything this loop can drive. "baseline" routes nothing; "e3" is metrics_logger's legacy arm.
METHODS = ("baseline",) + COMPARATORS + ("e3",)


class RoutingLoop:
    """One run's routing state and counters. Create it right after `traci.start`."""

    def __init__(self, algo_name: str) -> None:
        if algo_name not in METHODS:
            raise ValueError(f"unknown routing method {algo_name!r}; expected one of {METHODS}")
        self.algo_name = algo_name
        if algo_name == "e3":
            e3_initialize()
        # M2/C12: separate the vehicles that met the rerouting trigger from the vehicles whose
        # route SUMO actually replaced. The old code populated both from the same condition,
        # which is why rerouting efficiency was always exactly 100%.
        self.vehicles_needing_reroute = set()
        self.vehicles_rerouted = set()
        # Work order item 1. PSO's fitness is now a realised travel outcome rather than a copy
        # of the pheromone table, so a caller of aco_pso.select_route has to feed it outcomes or
        # its velocity term stays at the neutral mean, goes to zero, and the ACO-PSO arm quietly
        # degenerates into plain ACO. This arm's *pheromone* rule is deliberately left alone -
        # unifying the two execution paths is work order item 2, not this one.
        self.outcome_tracker = RouteOutcomeTracker() if algo_name == "aco_pso" else None
        self.reroute_attempts = 0
        self.reroutes_applied = 0
        self.per_vehicle_reroutes = {}
        self.vehicle_types = {}

    def before_step(self, sim_time, blocked_edges) -> None:
        """Everything a step does after the emergency manager's update and before responder tracking."""
        set_blocked_edges(blocked_edges)

        if self.outcome_tracker is not None:
            aco_pso_decay_fitness(seconds=1.0)
            for outcome in self.outcome_tracker.update(sim_time):
                aco_pso_record_outcome(outcome.travelled_edges, outcome.quality)

        # Work order item 2. E3 is now one implementation shared with the P3 runtime,
        # and its three memories decay on the clock rather than on routing events, so
        # this arm has to advance that clock exactly as a P3 run does. Without it the
        # comparison table would run E3 with pheromone that never evaporates.
        if self.algo_name == "e3":
            e3_step(sim_time)

        for veh in traci.vehicle.getIDList():
            if veh not in self.vehicle_types:
                self.vehicle_types[veh] = get_vehicle_type(veh)

    def route_vehicles(self, sim_time) -> None:
        """The shared reroute trigger and the method's own choice, for every vehicle in the network."""
        algo_name = self.algo_name
        if algo_name == "baseline":
            return

        for veh in traci.vehicle.getIDList():
            try:
                if not should_reroute(veh):
                    continue
                self.vehicles_needing_reroute.add(veh)

                current_edge = traci.vehicle.getRoadID(veh)
                route = traci.vehicle.getRoute(veh)
                if len(route) < 2:
                    continue

                self.reroute_attempts += 1
                applied = False

                if algo_name == "aco":
                    new_route = aco_select(veh, current_edge)
                    applied = set_route_checked(veh, new_route, route)
                    if applied:
                        update_pheromone(new_route)
                elif algo_name == "dijkstra":
                    applied = set_route_checked(veh, dijkstra_select(veh, current_edge), route)
                elif algo_name == "astar":
                    applied = set_route_checked(veh, astar_select(veh, current_edge), route)
                elif algo_name == "aco_pso":
                    new_route = aco_pso_select(veh, current_edge)
                    applied = set_route_checked(veh, new_route, route)
                    if applied:
                        aco_pso_update(new_route)
                        self.outcome_tracker.record_decision(veh, sim_time, new_route)
                elif algo_name == "e3":
                    new_route = e3_select(veh, current_edge)
                    applied = set_route_checked(veh, new_route, route)
                    if applied:
                        # Pheromone is no longer laid down when a route is chosen.
                        # The route is watched, and reinforced once its realised
                        # travel-time efficiency has been measured (item 1).
                        e3_note_applied(veh, sim_time, new_route)

                # M2: counted only once SUMO has accepted a changed route.
                if applied:
                    self.reroutes_applied += 1
                    self.vehicles_rerouted.add(veh)
                    self.per_vehicle_reroutes[veh] = self.per_vehicle_reroutes.get(veh, 0) + 1

            except traci.TraCIException:
                continue

    def counters(self) -> dict:
        """The loop's reroute counters, as metrics_logger reports them."""
        return {
            "vehicles_seen_in_network": len(self.vehicle_types),
            "vehicles_needing_reroute": len(self.vehicles_needing_reroute),
            "vehicles_rerouted": len(self.vehicles_rerouted),
            "reroute_attempts": self.reroute_attempts,
            "reroutes_applied": self.reroutes_applied,
        }
