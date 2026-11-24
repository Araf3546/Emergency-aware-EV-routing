"""What a route choice actually cost after it was driven.

`design_phase.md` section 2.5 states the principle this module exists to satisfy:
pheromone must update "from **successful** route choices", and "the system must learn from
actual outcomes, not just from predicted scores". Before this module nothing in the
codebase recorded what happened after a reroute was applied. `deposit()` laid down a flat
`Q = 1.0` on every edge of every applied route the instant it was applied, so an edge was
reinforced for being *chosen*, not for being *good*: a reroute that detoured a kilometre
into a jam was reinforced exactly as strongly as one that found a clear road. PSO's fitness
was then read from that same pheromone table, so both halves of ACO+PSO were learning
frequency.

The tracker watches each vehicle for a short window after its route is replaced and reports
what the vehicle actually got.

**Quality** is the realised mean speed over that window divided by the mean speed limit of
the roads the vehicle actually drove, clipped to [0, 1]. It is dimensionless, so windows of
different lengths are comparable, and it is exactly the classic ACO cost ratio: with
`L = actual time / free-flow time for the same path`, quality is `1 / L`, so a deposit
proportional to quality is the textbook `Q / L`.

**The observation window** ends at whichever comes first: the vehicle's next applied
reroute (the old route is no longer what is being judged), the vehicle leaving the network,
or `OBSERVATION_HORIZON_S`. A window shorter than `MIN_OBSERVATION_S` is discarded rather
than reported, because a two-second sample of a stop-start queue is noise.

Nothing here writes to the pheromone table or to PSO state. It reports outcomes and the
caller decides what to do with them, so the same outcomes can feed both learners and so the
measurement can be tested without SUMO.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import traci

# How long a route choice is watched before its outcome is reported, in simulated seconds.
# Two reroute intervals: long enough that a vehicle crawling at 1-2 m/s covers several of
# this network's very short edges (the median edge is 6.2 m), short enough that the deposit
# still describes the traffic the decision was made in.
OBSERVATION_HORIZON_S = 60.0

# Below this the sample is discarded. A window this short mostly measures where in a
# traffic-light cycle the vehicle happened to be.
MIN_OBSERVATION_S = 10.0

# A realised speed above this multiple of the free-flow speed is not a fast trip, it is a
# SUMO teleport moving the vehicle across the map and taking the odometer with it. Audit
# M12 showed a teleport can manufacture a response time; it can manufacture a route quality
# the same way. Such windows are discarded and counted.
IMPLAUSIBLE_SPEED_RATIO = 1.5


@dataclass
class RouteOutcome:
    """One completed observation window."""

    vehicle_id: str
    decided_at_s: float
    settled_at_s: float
    elapsed_s: float
    distance_m: float
    travelled_edges: List[str]
    planned_route: List[str]
    realised_speed_ms: float
    free_flow_speed_ms: float
    quality: float
    reason: str


@dataclass
class _Pending:
    vehicle_id: str
    decided_at_s: float
    start_distance_m: float
    planned_route: List[str]
    travelled_edges: List[str] = field(default_factory=list)
    last_seen_s: float = 0.0
    last_distance_m: float = 0.0


class RouteOutcomeTracker:
    """Watch applied reroutes and report what each one actually achieved."""

    def __init__(
        self,
        horizon_s: float = OBSERVATION_HORIZON_S,
        min_observation_s: float = MIN_OBSERVATION_S,
    ) -> None:
        self.horizon_s = horizon_s
        self.min_observation_s = min_observation_s
        self._pending: Dict[str, _Pending] = {}
        self._speed_limit_cache: Dict[str, Optional[float]] = {}
        self.counters = {
            "decisions_watched": 0,
            "outcomes_reported": 0,
            "discarded_too_short": 0,
            "discarded_no_geometry": 0,
            "discarded_implausible": 0,
            "settled_by_new_decision": 0,
            "settled_by_horizon": 0,
            "settled_by_leaving": 0,
            "unsettled_at_end": 0,
        }
        self.quality_sum = 0.0

    # ------------------------------------------------------------------
    # recording
    # ------------------------------------------------------------------
    def record_decision(
        self,
        vehicle_id: str,
        sim_time: float,
        planned_route: Sequence[str],
        distance_m: Optional[float] = None,
    ) -> Optional[RouteOutcome]:
        """Start watching `vehicle_id` after its route was replaced.

        If the vehicle is already being watched, the earlier window is closed first and its
        outcome returned - the route it was judging has just been replaced, so continuing to
        attribute the vehicle's progress to it would credit the old choice with the new
        one's driving.
        """
        settled = None
        if vehicle_id in self._pending:
            settled = self._settle(self._pending.pop(vehicle_id), "new_decision")
            self.counters["settled_by_new_decision"] += 1

        if distance_m is None:
            try:
                distance_m = traci.vehicle.getDistance(vehicle_id)
            except traci.TraCIException:
                return settled

        current_edge = ""
        try:
            current_edge = traci.vehicle.getRoadID(vehicle_id)
        except traci.TraCIException:
            current_edge = ""

        pending = _Pending(
            vehicle_id=vehicle_id,
            decided_at_s=sim_time,
            start_distance_m=distance_m,
            planned_route=[edge for edge in planned_route if not edge.startswith(":")],
            last_seen_s=sim_time,
            last_distance_m=distance_m,
        )
        if current_edge and not current_edge.startswith(":"):
            pending.travelled_edges.append(current_edge)
        self._pending[vehicle_id] = pending
        self.counters["decisions_watched"] += 1
        return settled

    def update(self, sim_time: float, present_ids: Optional[Sequence[str]] = None) -> List[RouteOutcome]:
        """Poll every watched vehicle and return the windows that closed this step.

        Call once per simulation step. The cost is two TraCI calls per *watched* vehicle,
        and only vehicles whose route was actually replaced are watched - a few dozen at a
        time, not the whole fleet.
        """
        if not self._pending:
            return []

        if present_ids is None:
            present_ids = traci.vehicle.getIDList()
        present = set(present_ids)

        outcomes: List[RouteOutcome] = []
        for vehicle_id in list(self._pending.keys()):
            pending = self._pending[vehicle_id]

            if vehicle_id not in present:
                # Arrived, or removed by SUMO. The last poll is at most one step stale.
                self._pending.pop(vehicle_id, None)
                self.counters["settled_by_leaving"] += 1
                outcome = self._settle(pending, "left_network")
                if outcome:
                    outcomes.append(outcome)
                continue

            try:
                road = traci.vehicle.getRoadID(vehicle_id)
                distance = traci.vehicle.getDistance(vehicle_id)
            except traci.TraCIException:
                continue

            if road and not road.startswith(":") and (
                not pending.travelled_edges or pending.travelled_edges[-1] != road
            ):
                pending.travelled_edges.append(road)
            pending.last_seen_s = sim_time
            pending.last_distance_m = distance

            if sim_time - pending.decided_at_s >= self.horizon_s:
                self._pending.pop(vehicle_id, None)
                self.counters["settled_by_horizon"] += 1
                outcome = self._settle(pending, "horizon")
                if outcome:
                    outcomes.append(outcome)

        return outcomes

    # ------------------------------------------------------------------
    # settlement
    # ------------------------------------------------------------------
    def _settle(self, pending: _Pending, reason: str) -> Optional[RouteOutcome]:
        elapsed = pending.last_seen_s - pending.decided_at_s
        if elapsed < self.min_observation_s:
            self.counters["discarded_too_short"] += 1
            return None

        distance = max(0.0, pending.last_distance_m - pending.start_distance_m)
        free_flow = self._free_flow_speed(pending.travelled_edges)
        if free_flow is None or free_flow <= 0.0:
            self.counters["discarded_no_geometry"] += 1
            return None

        realised = distance / elapsed
        if realised > IMPLAUSIBLE_SPEED_RATIO * free_flow:
            self.counters["discarded_implausible"] += 1
            return None

        quality = max(0.0, min(1.0, realised / free_flow))
        self.counters["outcomes_reported"] += 1
        self.quality_sum += quality
        return RouteOutcome(
            vehicle_id=pending.vehicle_id,
            decided_at_s=pending.decided_at_s,
            settled_at_s=pending.last_seen_s,
            elapsed_s=elapsed,
            distance_m=distance,
            travelled_edges=list(pending.travelled_edges),
            planned_route=list(pending.planned_route),
            realised_speed_ms=realised,
            free_flow_speed_ms=free_flow,
            quality=quality,
            reason=reason,
        )

    def _free_flow_speed(self, edges: Sequence[str]) -> Optional[float]:
        """Mean speed limit of the roads actually driven, cached per edge.

        The mean rather than a distance weighting: this network's edges are very short
        (median 6.2 m) and a window covers a handful of them, so the weighting would change
        the number by far less than the noise in a 60-second speed sample.
        """
        limits = []
        for edge_id in edges:
            limit = self._speed_limit_cache.get(edge_id, "miss")
            if limit == "miss":
                try:
                    limit = traci.lane.getMaxSpeed(f"{edge_id}_0")
                except traci.TraCIException:
                    limit = None
                self._speed_limit_cache[edge_id] = limit
            if limit:
                limits.append(limit)
        if not limits:
            return None
        return sum(limits) / len(limits)

    # ------------------------------------------------------------------
    # reporting
    # ------------------------------------------------------------------
    def stats(self) -> dict:
        report = dict(self.counters)
        report["unsettled_at_end"] = len(self._pending)
        reported = report["outcomes_reported"] or 1
        report["mean_quality"] = round(self.quality_sum / reported, 4)
        return report

    def reset(self) -> None:
        self._pending.clear()
        for key in self.counters:
            self.counters[key] = 0
        self.quality_sum = 0.0
