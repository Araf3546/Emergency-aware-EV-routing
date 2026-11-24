"""Junction-triggered rerouting - condition B of the trigger comparison (2026-09-12).

EXPERIMENTAL. Not used by any production runner; `scripts/diagnostics/trigger_comparison.py`
swaps it in for one run. The production trigger is still `controller/reroute_logic.should_reroute`.

The current trigger (condition A) asks two questions: *when* to look - on a 30-second clock,
wherever the vehicle happens to be - and *whether* rerouting is warranted - the vehicle's edge
is slow (mean speed < 2 m/s) or busy (3 or more vehicles). This trigger changes only the first:

* **when**: while the vehicle is on an edge whose end junction offers a real routing choice
  (`controller/decision_points.py`: at least two distinct, loop-free, destination-reachable
  options within 50% of the best), at most once per visit to that edge;
* **whether**: exactly the current congestion condition, unchanged;
* the emergency-type clause of the current trigger (an emergency-type vehicle on a closed edge
  is sent to the router every step) is carried over unchanged - it belongs to Pending Supervisor
  Decision PD-1 and must not be settled by a trigger experiment.

E3 itself, and everything else, is untouched: when this fires, the same `E3Core.decide` runs
with the same candidates it would have had.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import traci

from algorithms.route_utils import get_outgoing_edges
from controller.decision_points import DecisionPoints
from controller.reroute_logic import MIN_SPEED_THRESHOLD, QUEUE_THRESHOLD
from emergency_manager import get_emergency_manager


class JunctionTrigger:
    def __init__(self, decision_points: Optional[DecisionPoints] = None) -> None:
        self.points = decision_points or DecisionPoints()
        # vehicle -> (edge, route index) of the visit on which it last fired
        self._fired: Dict[str, Tuple[str, int]] = {}
        # vehicle -> destination edge; reroutes preserve the destination (audit C1)
        self._destination: Dict[str, str] = {}
        # (candidates, destination) -> real choice?
        self._choice: Dict[Tuple[Tuple[str, ...], str], bool] = {}

    def _real_choice(self, vehicle_id: str, edge: str) -> bool:
        candidates = tuple(get_outgoing_edges(edge, count_stats=False, veh_id=vehicle_id))
        if len(candidates) < 2:
            return False
        destination = self._destination.get(vehicle_id)
        if destination is None:
            destination = traci.vehicle.getRoute(vehicle_id)[-1]
            self._destination[vehicle_id] = destination
        key = (candidates, destination)
        if key not in self._choice:
            self._choice[key] = self.points.has_real_choice(candidates, destination)
        return self._choice[key]

    def should_reroute(self, vehicle_id: str) -> bool:
        try:
            current_edge = traci.vehicle.getRoadID(vehicle_id)
        except traci.TraCIException:
            return False
        if current_edge.startswith(":"):
            return False

        # Same clause as the current trigger, which PD-1 (2026-09-13) keyed on the active-
        # responder flag instead of the vehicle type. This experimental trigger is rejected
        # and not used by any production runner; it is kept consistent for re-running item 3.
        manager = get_emergency_manager()
        if manager.is_active_responder(vehicle_id) and manager.is_edge_blocked(current_edge):
            return True

        try:
            visit = (current_edge, traci.vehicle.getRouteIndex(vehicle_id))
            if self._fired.get(vehicle_id) == visit:
                return False
            if not self._real_choice(vehicle_id, current_edge):
                return False
            # The current trigger's congestion condition, unchanged.
            edge_speed = traci.edge.getLastStepMeanSpeed(current_edge)
            edge_queue = traci.edge.getLastStepVehicleNumber(current_edge)
        except traci.TraCIException:
            return False
        if edge_speed < MIN_SPEED_THRESHOLD or edge_queue >= QUEUE_THRESHOLD:
            self._fired[vehicle_id] = visit
            return True
        return False
