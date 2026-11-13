"""Runtime bridge that connects the P3 protocol layer to SUMO decision making."""

from __future__ import annotations

import math
import os
import sys
import traceback
from typing import Optional

import traci

from algorithms.e3_core import E3Core
from algorithms.route_utils import (
    remaining_route,
    get_route_stats,
    reset_route_stats,
    seed_route_choice,
    set_blocked_edges,
)
from controller.reroute_logic import should_reroute
from emergency_manager import get_emergency_manager
from p3_hybrid_router import HybridRouter
from p3_metrics import P3MetricsLogger
from p3_protocol import ProtocolStateManager, build_emergency_message, build_message
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES


# ===============================
# YIELDING RULE (the protocol's own path-clearing mechanism)
# ===============================
#
# SUMO's built-in emergency behaviour (the bluelight device) is deliberately NOT used.
# Clearing a path for a responder is the mechanism this thesis proposes, so it is
# implemented here and driven by received V2V messages rather than by omniscience.
#
# An ordinary vehicle is asked to yield only when all of these hold:
#   1. a fresh broadcast of an active responder names the road the vehicle is on: the
#      responder's own road, or the road it announced it enters next;
#   2. it is within YIELD_RADIUS_M of that responder;
#   3. it is ahead of the responder, not behind it;
#   4. it is still moving, so it actually has room to give;
#   5. it has not been asked in the last YIELD_COOLDOWN_S seconds.
#
# What it then does is the unified rule (user decision, 2026-09-15): make room if you can,
# never brake in the responder's lane. See `_yield_to_emergency`. In short:
#   - it never brakes;
#   - on a road with two or more lanes it may use, a vehicle in the responder's lane path
#     with a usable lane to its right asks for a safe move right. SUMO refuses an unsafe
#     lane change;
#   - otherwise it keeps driving.
# Nothing stops traffic dead and no road is cleared.

YIELD_RADIUS_M = 250.0           # config/scenario.yaml, protocol.message_radius_m
YIELD_DURATION_S = 3.0           # how long a move-aside request lasts, and how long the car then says it is yielding
YIELD_COOLDOWN_S = 5.0           # a vehicle is not asked to yield again within this time
YIELD_MOVING_THRESHOLD_MS = 1.0  # slower than this is already stuck; leave it alone
# YIELD_MIN_SPEED_MS (2.0) and YIELD_SPEED_FACTOR (0.4) were removed on 2026-09-15. They defined the old action, a
# brake to max(2 m/s, 40% of speed), which the unified rule drops.

# The E3 scoring parameters and the decision logic now live in `algorithms/e3_core.py`,
# which is the single implementation this runtime and the six-algorithm comparison table
# both drive (work order item 2). HEURISTIC_PROGRESS_BLEND is re-exported above so existing
# diagnostics that import it from here keep working.

# Which edges a realised outcome reinforces (work order item 1).
#
#   "travelled" - only the roads the vehicle actually drove during the observation window.
#                 The strict reading of design_phase.md 2.5: an edge is reinforced by what
#                 happened on it, never by a prediction about it.
#   "planned"   - every edge of the route that was applied, scaled by the quality measured
#                 on its opening stretch. This is the classic ACO shape, where the whole
#                 constructed solution is reinforced in proportion to its cost, and it keeps
#                 the wide pheromone coverage the old flat rule had.
#
# The two differ by roughly an order of magnitude in how many edges receive a deposit, so
# the choice is not cosmetic and was settled by measurement, not by argument. Both were run
# for a full hour at two seeds:
#
#   completed / teleports    seed 20260722        seed 20260721
#   travelled                3,239 / 28           2,794 / 103
#   planned                  3,363 / 17           3,237 / 25
#
# "planned" wins at both seeds and "travelled" is fragile - at the second seed it lost 445
# completed trips and teleported 103 times. The mechanism is visible in the pheromone table:
# reinforcing only the driven stretch leaves the table with 335 entries against 1,156, and
# pheromone then tells candidate edges apart in 52.8% of selections against 86.1%. The
# stricter rule starves the term it is supposed to be improving.
#
# So the deposit target stays the whole applied route, as classic ACO reinforces a whole
# constructed tour, and it is the *strength* that now comes from a realised outcome. The
# honest limitation: the quality is measured on the route's first sixty seconds of driving
# and credited to all of it. Override with P3_DEPOSIT_SCOPE to reproduce the comparison.
DEPOSIT_SCOPE = os.environ.get("P3_DEPOSIT_SCOPE", "planned")


class P3RuntimeCoordinator:
    """Coordinate protocol broadcasts and hybrid decisions during a SUMO run."""

    def __init__(
        self,
        freshness_window: float = 3.0,
        emergency_layer: bool = True,
        route_choice_seed: Optional[int] = None,
    ) -> None:
        # Contract section 5: seed from config/scenario.yaml, and start the run's route
        # validity counters from zero (audit C1, C13). A five-seed evaluation run passes its
        # evaluation seed instead (decision 13.15, see evaluation_seeds.py); None keeps the
        # configured seed.
        self.route_choice_seed = seed_route_choice(route_choice_seed)
        reset_route_stats()
        self.protocol_state = ProtocolStateManager(freshness_window=freshness_window)
        # The matched ablation of 2026-09-08: both compared arms route normal traffic with
        # E3, and only the emergency layer differs, so the untuned router cancels out
        # instead of confounding the comparison. `emergency_layer=False` is the control.
        # It gates exactly three things and nothing else:
        #   1. the protocol term in edge scoring, forced to the neutral constant 0.5;
        #   2. the yielding rule in _apply_yield_behaviour;
        #   3. the emergency corridor weight a responder claims for itself.
        # Demand, seed, network, incidents, the broadcast schedule and every other E3 term are
        # identical between the two. Since the clean design (2026-09-15) a broadcast's yield flag
        # follows gate 2: nobody yields with the layer off, so nobody raises it. It has no routing
        # effect in either arm.
        # The closure reachability filter (`route_utils.reachable_candidates`) is not on this list.
        # It is a routing-feasibility rule, not part of the V2V emergency mechanism, and it runs in
        # both arms (user decision 2026-09-15). tests/test_closure_filter_both_arms.py checks that
        # the switch is read only at these three gates.
        self.emergency_layer = emergency_layer
        # Work order item 2: the routing algorithm itself. One implementation, shared with
        # the six-algorithm comparison table through `algorithms/e3.py`, over one pheromone
        # and PSO state. This coordinator's job is the protocol layer - broadcasts, the
        # emergency gate, yielding and metrics - not the routing.
        self.e3 = E3Core(
            emergency_layer=emergency_layer,
            deposit_scope=DEPOSIT_SCOPE,
            reset_state=True,
        )
        self.metrics = P3MetricsLogger()
        self.last_broadcast: dict[str, float] = {}
        self.message_interval = 2.0
        # Audit M11: blockage near emergency zones. One sample per step per active
        # incident, over that incident's own edges only - a handful of TraCI calls, not a
        # network scan.
        self.blockage_samples: list[dict] = []
        # Two times per vehicle since the unified yield rule (2026-09-15):
        # - `last_asked_to_yield`: when a responder's broadcast last made it respond, whatever it could do. The
        #   YIELD_COOLDOWN_S cooldown runs from it, as it always ran from every yield.
        # - `last_yield`: when it last actually moved aside. Only that raises `yield_request` and counts in
        #   `yield_actions`.
        self.last_asked_to_yield: dict[str, float] = {}
        self.last_yield: dict[str, float] = {}
        self._active_responders: set[str] = set()
        self.yield_actions = 0

    # The routing state lives on the shared E3Core. These read-only views keep the
    # diagnostics and tuning scripts that reach for `coordinator.router`,
    # `coordinator.outcomes` and `coordinator.deposit_scope` working unchanged.
    @property
    def router(self) -> HybridRouter:
        return self.e3.router

    @property
    def outcomes(self):
        return self.e3.outcomes

    @property
    def deposit_scope(self) -> str:
        return self.e3.deposit_scope

    @staticmethod
    def _log_exception(context: str, exc: Exception) -> None:
        print(f"[P3Runtime] {context}: {type(exc).__name__}: {exc}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)

    def update_step(self, sim_time: float) -> None:
        """Broadcast protocol messages and apply hybrid route decisions for the current step."""

        # Tell the routing layer which roads an incident currently has closed, so ordinary
        # traffic is not routed through them (they would be rejected by SUMO with only a
        # warning). Emergency vehicles stay exempt - contract decision 13.2.
        set_blocked_edges(get_emergency_manager().blocked_edges)

        # All three of E3's memories fade on the clock rather than on routing events -
        # pheromone (audit A10), BCO's scout memory (the A6 follow-up) and PSO's personal
        # and global bests (work order item 1). Each used to decay per decision or per
        # applied reroute, which made the real rate depend on how busy the router was.
        # Settling finished observation windows stays a separate call below, because the
        # outcome-learning diagnostic replaces `_settle_route_outcomes` to inspect it.
        self.e3.decay(seconds=1.0)

        # Audit P9: emergency behaviour is driven by who is *actually responding to an
        # incident*, not by vehicle type. Refreshed once per step and read by the
        # broadcast, corridor-scoring and yielding paths below.
        self._active_responders = get_emergency_manager().get_active_responder_ids(sim_time)

        try:
            self._broadcast_vehicle_messages(sim_time)
        except Exception as exc:
            self._log_exception("broadcast_vehicle_messages failed", exc)
        try:
            self._track_emergency_responses(sim_time)
        except Exception as exc:
            self._log_exception("track_emergency_responses failed", exc)
        try:
            self._record_vehicle_metrics(sim_time)
        except Exception as exc:
            self._log_exception("record_vehicle_metrics failed", exc)
        try:
            self._settle_route_outcomes(sim_time)
        except Exception as exc:
            self._log_exception("settle_route_outcomes failed", exc)
        try:
            for vehicle_id in traci.vehicle.getIDList():
                try:
                    if not should_reroute(vehicle_id):
                        continue
                    self._apply_hybrid_decision(vehicle_id, sim_time)
                except traci.TraCIException:
                    continue
                except Exception as exc:
                    self._log_exception(f"apply_hybrid_decision failed for {vehicle_id}", exc)
                    continue
        except Exception as exc:
            self._log_exception("vehicle iteration failed", exc)
        try:
            self._record_arrivals(sim_time)
        except Exception as exc:
            self._log_exception("record_arrivals failed", exc)
        try:
            self._apply_yield_behaviour(sim_time)
        except Exception as exc:
            self._log_exception("apply_yield_behaviour failed", exc)

        try:
            self._sample_emergency_zone_blockage(sim_time)
        except Exception as exc:
            self._log_exception("sample_emergency_zone_blockage failed", exc)

        # Audit M3 and M4: this used to average getLastStepMeanSpeed over all 21,385
        # non-internal edges every step and report 1 - mean/15.0 as "congestion". An empty
        # edge reports its speed limit, so that number described the road graph, not the
        # traffic - it varied by 3.5e-5 across a whole run and was identical to 16 decimal
        # places in all 25 tuning trials. It also cost about 1.5 s per step, which was most
        # of the runtime.
        #
        # Congestion is now mean time loss in seconds per vehicle, taken from tripinfo.xml
        # after the run (contract section 8, decision 13.4). The per-step time series comes
        # from SUMO's own summary.xml, which costs nothing.

    def _settle_route_outcomes(self, sim_time: float) -> None:
        """Turn finished observation windows into pheromone and PSO fitness.

        Delegated to the shared core, but kept as a named method on the coordinator
        because the outcome-learning diagnostic replaces it in order to record what each
        settlement produced.
        """
        self.e3.settle_outcomes(sim_time)

    def _apply_yield_behaviour(self, sim_time: float) -> None:
        """Ask ordinary vehicles near a responder they have heard from to make room for it.

        Driven entirely by fresh V2V messages, so a vehicle only reacts to a responder it
        could actually have received a broadcast from. Who is asked, and when, is decided here;
        what the vehicle then does is `_yield_to_emergency` (the unified rule, 2026-09-15).
        """
        if not self.emergency_layer:
            # Control arm: ordinary traffic does not give way. Broadcasting still happens,
            # so message counts stay comparable between the arms; nothing acts on them.
            return

        emergency_messages = self.protocol_state.get_emergency_neighbors(sim_time)
        if not emergency_messages:
            return

        # A message can still be fresh after its sender has left the network, so check
        # membership once per step rather than letting every lookup raise.
        present = set(traci.vehicle.getIDList())

        for message in emergency_messages:
            responder_id = message.vehicle_id
            if responder_id not in present:
                continue
            try:
                responder_position = traci.vehicle.getPosition(responder_id)
                responder_edge = traci.vehicle.getRoadID(responder_id)
                responder_lane_position = traci.vehicle.getLanePosition(responder_id)
            except traci.TraCIException:
                continue

            # Only the road the responder is on, and the one it is about to enter.
            for edge_id in [e for e in (responder_edge, message.next_edge) if e]:
                try:
                    candidates = traci.edge.getLastStepVehicleIDs(edge_id)
                except traci.TraCIException:
                    continue

                for vehicle_id in candidates:
                    if vehicle_id == responder_id:
                        continue
                    # Audit P9: only active responders are exempt from yielding.
                    # Ambient emergency vehicles are ordinary traffic here.
                    if vehicle_id in self._active_responders:
                        continue
                    if sim_time - self.last_asked_to_yield.get(vehicle_id, -1e9) < YIELD_COOLDOWN_S:
                        continue
                    try:
                        speed = traci.vehicle.getSpeed(vehicle_id)
                        # Already stopped in traffic: it has no room to give.
                        if speed < YIELD_MOVING_THRESHOLD_MS:
                            continue

                        position = traci.vehicle.getPosition(vehicle_id)
                        distance = math.hypot(
                            position[0] - responder_position[0],
                            position[1] - responder_position[1],
                        )
                        if distance > YIELD_RADIUS_M:
                            continue

                        # Only vehicles in front of the responder can be in its way.
                        if edge_id == responder_edge:
                            ahead = traci.vehicle.getLanePosition(vehicle_id)
                            if ahead <= responder_lane_position:
                                continue

                        self._yield_to_emergency(vehicle_id, edge_id, responder_id, sim_time)
                    except traci.TraCIException:
                        continue

    def _yield_to_emergency(
        self, vehicle_id: str, edge_id: str, responder_id: str, sim_time: float
    ) -> Optional[str]:
        """Make room for the responder if the car can, and never brake. The unified rule (user decision, 2026-09-15).

        - A single-lane road (fewer than two lanes the car's class may use): the car keeps driving. It cannot move
          aside, and braking would only hold up a responder that cannot pass. This network has no opposite-lane
          overtaking and the simulation no sublane model.
        - A road with two or more such lanes: a car in the responder's lane path ahead of it (`_in_responder_path`)
          with a lane it may use to its right is asked to move to the rightmost such lane for YIELD_DURATION_S.
          That is lane 0 whenever it may use it. SUMO changes lanes only into a safe gap. Every other car keeps
          driving.
        - It never brakes, not even to open a gap for the lane change.

        Every response starts the cooldown (`last_asked_to_yield`), so who is asked and when is unchanged. Only a
        move aside is a yield: it sets `last_yield`, which raises `yield_request` for YIELD_DURATION_S, and it counts
        in `yield_actions`.

        Returns what the car did, for diagnostics: "move_right", "keep_driving_single_lane",
        "keep_driving_not_in_lane" or "keep_driving_cannot_move". Returns None when SUMO refused a query or the
        request; nothing is then recorded, as before.

        This replaces the action of 2026-09-07 (Option B): slow to max(2 m/s, 40% of speed) over 3 s, and on any
        road with more than one lane ask for lane 0. Measured at 3 x 3,600 s:
        - it braked a car that stayed in the responder's path in 30-43% of yields;
        - it braked a car not in the path at all in 46-61%;
        - it moved some cars into the responder's lane.
        Evidence: results/make_room_yield_2026-09-15/.
        """
        try:
            usable = self._lanes_the_vehicle_may_use(edge_id, traci.vehicle.getVehicleClass(vehicle_id))
            if len(usable) < 2:
                action = "keep_driving_single_lane"
            elif not self._in_responder_path(vehicle_id, responder_id):
                action = "keep_driving_not_in_lane"
            else:
                lane_index = traci.vehicle.getLaneIndex(vehicle_id)
                to_the_right = [index for index in usable if index < lane_index]
                if not to_the_right:
                    action = "keep_driving_cannot_move"
                else:
                    traci.vehicle.changeLane(vehicle_id, min(to_the_right), YIELD_DURATION_S)
                    action = "move_right"
        except traci.TraCIException:
            return None

        if action == "move_right":
            self.last_yield[vehicle_id] = sim_time
            self.yield_actions += 1
            try:
                self.metrics.record_yield_action(vehicle_id)
            except Exception:
                pass
        self.last_asked_to_yield[vehicle_id] = sim_time
        return action

    @staticmethod
    def _lanes_the_vehicle_may_use(edge_id: str, vehicle_class: str) -> list[int]:
        """Indices of the lanes of `edge_id` that `vehicle_class` may use, under the live permissions.

        A closure disallows every class but `emergency`, and a bus lane is not a lane a car can move into.
        """
        usable = []
        for index in range(traci.edge.getLaneNumber(edge_id)):
            lane_id = f"{edge_id}_{index}"
            allowed, disallowed = traci.lane.getAllowed(lane_id), traci.lane.getDisallowed(lane_id)
            if (not allowed or vehicle_class in allowed) and vehicle_class not in disallowed:
                usable.append(index)
        return usable

    @staticmethod
    def _in_responder_path(vehicle_id: str, responder_id: str) -> bool:
        """Is the car ahead of the responder, in the lane the responder will drive?

        True when the car is one of these:
        - the responder's immediate leader;
        - ahead of it in its lane;
        - in a lane it continues into (SUMO's best-lane continuation for its current lane).
        """
        try:
            leader = traci.vehicle.getLeader(responder_id, YIELD_RADIUS_M)
        except traci.TraCIException:
            leader = None
        if leader and leader[0] == vehicle_id:
            return True
        car_lane, responder_lane = traci.vehicle.getLaneID(vehicle_id), traci.vehicle.getLaneID(responder_id)
        if car_lane == responder_lane:
            return traci.vehicle.getLanePosition(vehicle_id) > traci.vehicle.getLanePosition(responder_id)
        try:
            for entry in traci.vehicle.getBestLanes(responder_id):
                if entry[0] == responder_lane:
                    return car_lane in entry[5]
        except traci.TraCIException:
            pass
        return False

    def _sample_emergency_zone_blockage(self, sim_time: float) -> None:
        """Record how much traffic on each active incident's edges is standing still.

        Audit M11 requires a conflict / blockage rate near emergency zones. Only the
        incident's own edges are sampled, so the cost is a few TraCI calls per step.
        """
        emergency_mgr = get_emergency_manager()
        for event_id in list(getattr(emergency_mgr, "active_events", []) or []):
            event = emergency_mgr.events.get(event_id)
            if event is None:
                continue
            vehicles = 0
            halting = 0
            for edge_id in event.blocked_edges:
                try:
                    vehicles += traci.edge.getLastStepVehicleNumber(edge_id)
                    halting += traci.edge.getLastStepHaltingNumber(edge_id)
                except traci.TraCIException:
                    continue
            self.blockage_samples.append(
                {
                    "event_id": event_id,
                    "sim_time": sim_time,
                    "vehicles": vehicles,
                    "halting": halting,
                }
            )

    def _broadcast_vehicle_messages(self, sim_time: float) -> None:
        for vehicle_id in traci.vehicle.getIDList():
            try:
                last_time = self.last_broadcast.get(vehicle_id, -float("inf"))
                if sim_time - last_time < self.message_interval:
                    continue

                vehicle_type = traci.vehicle.getTypeID(vehicle_id)
                current_edge = traci.vehicle.getRoadID(vehicle_id)
                speed = traci.vehicle.getSpeed(vehicle_id)
                # Audit P9: was `vehicle_type in EMERGENCY_VEHICLE_TYPES`, which made all
                # 240 ambient emergency vehicles broadcast emergency priority all hour.
                emergency_flag = vehicle_id in self._active_responders
                priority_level = 4 if emergency_flag else 0
                # Audit P6: this used to be route[1], the second edge of the *whole*
                # route, which is stale as soon as the vehicle drives past it. The
                # yielding rule needs the edge the vehicle is actually about to enter.
                remaining = remaining_route(vehicle_id)
                next_edge = remaining[1] if len(remaining) > 1 else None
                congestion_estimate = self._local_congestion(current_edge)
                distance_to_emergency = None
                emergency_mgr = get_emergency_manager()
                nearest = emergency_mgr.get_nearest_emergency(vehicle_id)
                if nearest:
                    _, distance, _ = nearest
                    distance_to_emergency = float(distance)
                # The clean design (user decision, 2026-09-15). `yield_request` means only "I am actually
                # yielding": this car was asked to move aside for a responder in the last YIELD_DURATION_S
                # seconds (`_yield_to_emergency`, which sets `last_yield` only for that manoeuvre since the
                # unified rule of 2026-09-15). A car that kept driving because it could not make room, or
                # was not in the way, does not say it is yielding. That is the design document's meaning,
                # and priority 1 is its "yield/clearance state".
                # It used to be raised whenever the car was within 200 m of an open incident, which says
                # nothing about yielding. The incident distance is still carried as its own field.
                yield_request = (not emergency_flag) and (
                    0.0 < sim_time - self.last_yield.get(vehicle_id, -float("inf")) <= YIELD_DURATION_S)
                if yield_request:
                    priority_level = max(priority_level, 1)

                if emergency_flag:
                    message = build_emergency_message(
                        vehicle_id=vehicle_id,
                        current_time=sim_time,
                        vehicle_type=vehicle_type,
                        position_edge=current_edge,
                        speed=speed,
                        route_intent=next_edge or "",
                        next_edge=next_edge,
                    )
                else:
                    message = build_message(
                        vehicle_id=vehicle_id,
                        current_time=sim_time,
                        vehicle_type=vehicle_type,
                        position_edge=current_edge,
                        speed=speed,
                        priority_level=priority_level,
                        emergency_flag=emergency_flag,
                        route_intent=next_edge or "",
                        position_lane=traci.vehicle.getLaneID(vehicle_id),
                        congestion_estimate=congestion_estimate,
                        yield_request=yield_request,
                        next_edge=next_edge,
                        distance_to_emergency=distance_to_emergency,
                    )

                self.protocol_state.update(sim_time, message)
                self.last_broadcast[vehicle_id] = sim_time
                self.metrics.record_broadcast(vehicle_id, emergency_flag, yield_request)
            except traci.TraCIException:
                continue

    def _apply_hybrid_decision(self, vehicle_id: str, sim_time: float) -> None:
        """Ask the shared E3 implementation for a decision, and record what it did.

        Work order item 2: the scoring, selection and learning used to be written out
        here, which is how this file and `algorithms/e3.py` drifted into being two
        different algorithms with the same name. Everything routing-related now lives in
        `algorithms/e3_core.E3Core`. What stays here is what genuinely belongs to the
        protocol layer: which V2V messages this vehicle can hear, whether it is actively
        responding, whether the emergency layer is on, and the metrics.
        """
        try:
            emergency_mgr = get_emergency_manager()
            emergency_context = emergency_mgr.get_active_emergency_zones()
            # Audit A1 (2026-09-14): a vehicle's own broadcast is not neighbour information.
            messages = self.protocol_state.get_fresh_neighbors(sim_time, exclude=vehicle_id)
            emergency_neighbors = self.protocol_state.get_emergency_neighbors(sim_time, exclude=vehicle_id)

            # Audit P9: only an actively responding vehicle gets the emergency corridor
            # weight. An ambient emergency vehicle routes like ordinary traffic.
            is_emergency = vehicle_id in self._active_responders
            # Gate 3 of the ablation. `is_emergency` is kept for the metrics, which must
            # record who was responding in both arms; only the scoring input is gated.
            scores_as_emergency = is_emergency and self.emergency_layer

            report = self.e3.decide(
                vehicle_id,
                sim_time,
                messages=messages,
                is_emergency_vehicle=scores_as_emergency,
                corridor_edge=self._responder_corridor_edge(vehicle_id) if scores_as_emergency else None,
            )

            # A vehicle with no onward choice, no route, or a TraCI failure never reached
            # any counter before the unification either, and about 92% of calls in this
            # network are exactly that. Recording them here would inflate every per-
            # decision metric and, worse, `reroute_decisions`, which
            # scripts/16_compare_baseline_vs_p3.py uses to refuse a run whose coordinator
            # did nothing.
            if report["reason"] in ("no_candidates", "no_route", "traci_error"):
                return

            self.metrics.record_neighbor_state(len(messages), len(emergency_neighbors))
            self.metrics.record_emergency_zone_snapshot(len(emergency_context))
            self.metrics.record_candidate_scoring(report["candidates"])

            # The router looked and chose to stay on this edge. Not a reroute decision.
            if report["reason"] == "stay_on_edge":
                return

            self.metrics.record_reroute(
                vehicle_id, report["applied"], is_emergency, bool(emergency_context)
            )
            if report["applied"]:
                self.metrics.record_vehicle_reroute(vehicle_id)
        except traci.TraCIException:
            return

    @staticmethod
    def _responder_corridor_edge(vehicle_id: str) -> Optional[str]:
        """The edge an active responder is about to enter on its current route: the corridor it may
        keep using (P8, ablation gate 3). Read from its route at decision time; until 2026-09-14 it
        came from the responder's own broadcast, up to one broadcast interval old (audit A1)."""
        remaining = remaining_route(vehicle_id)
        return remaining[1] if len(remaining) > 1 else None

    @staticmethod
    def _local_congestion(edge_id: str) -> float:
        """Local congestion estimate for one edge, broadcast inside a protocol message.

        This is a protocol signal, not a reported metric - the reported congestion is mean
        time loss from tripinfo (contract decision 13.4). It is normalised against the
        edge's own speed limit rather than the magic constant 15.0 the audit flagged in M3,
        which matched neither the vType maximum (13.89 m/s) nor the network's limits.
        """
        try:
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
            mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
            allowed = P3RuntimeCoordinator._edge_speed_limit(edge_id)
            if allowed <= 0:
                return 0.0
            congestion = min(1.0, max(0.0, 1.0 - (mean_speed / allowed)))
            return round(min(1.0, congestion + vehicle_count * 0.01), 6)
        except traci.TraCIException:
            return 0.0

    _speed_limit_cache: dict[str, float] = {}

    @staticmethod
    def _edge_speed_limit(edge_id: str) -> float:
        """Speed limit of an edge's first lane, cached; lane limits do not change here."""
        cache = P3RuntimeCoordinator._speed_limit_cache
        if edge_id not in cache:
            try:
                cache[edge_id] = float(traci.lane.getMaxSpeed(f"{edge_id}_0"))
            except traci.TraCIException:
                cache[edge_id] = 0.0
        return cache[edge_id]

    def save_metrics(self, run_name: str = "p3_hybrid") -> None:
        self.metrics.save_results(run_name=run_name)
        self.save_route_stats(run_name=run_name)

    def save_route_stats(self, run_name: str = "p3_hybrid") -> None:
        """Write the route-validity counters. Contract section 5 requires them reported."""
        import json
        import os

        stats = get_route_stats()
        stats["route_choice_seed"] = self.route_choice_seed
        stats["pythonhashseed"] = os.environ.get("PYTHONHASHSEED", "unset")
        base = os.path.join("results", run_name)
        os.makedirs(base, exist_ok=True)
        with open(os.path.join(base, f"{run_name}_route_stats.json"), "w") as handle:
            json.dump(stats, handle, indent=2)
        print(f"[P3Runtime] route validity: {stats}")

    def _record_vehicle_metrics(self, sim_time: float) -> None:
        for vehicle_id in traci.vehicle.getIDList():
            try:
                vehicle_type = traci.vehicle.getTypeID(vehicle_id)
                is_emergency = vehicle_type in EMERGENCY_VEHICLE_TYPES
                speed = traci.vehicle.getSpeed(vehicle_id)
                waiting_time = traci.vehicle.getWaitingTime(vehicle_id)
                distance = traci.vehicle.getDistance(vehicle_id)

                self.metrics.record_vehicle_step(
                    vehicle_id=vehicle_id,
                    sim_time=sim_time,
                    vehicle_type=vehicle_type,
                    is_emergency=is_emergency,
                    speed=speed,
                    waiting_time=waiting_time,
                    distance=distance,
                )
            except traci.TraCIException:
                continue

    def _record_arrivals(self, sim_time: float) -> None:
        try:
            arrived_vehicles = traci.simulation.getArrivedIDList()
            for vehicle_id in arrived_vehicles:
                try:
                    self.metrics.record_vehicle_arrival(vehicle_id, sim_time)
                except Exception as e:
                    self._log_exception(f"record_vehicle_arrival failed for {vehicle_id}", e)
        except Exception as exc:
            self._log_exception("getArrivedIDList failed", exc)

    def _track_emergency_responses(self, sim_time: float) -> None:
        """Detection is delegated so every runner uses one definition of arrival.

        Note there is no `if not active_events: return` guard any more. Response time is
        incident activation until the dispatched responder reaches the centre edge
        (contract sections 7 and 8); the incident window does not have to still be open.
        """
        emergency_mgr = get_emergency_manager()
        newly_recorded = emergency_mgr.record_responder_arrivals(
            sim_time,
            vehicle_ids=traci.vehicle.getIDList(),
            arrived_ids=traci.simulation.getArrivedIDList(),
        )
        for event_id, response_time in newly_recorded.items():
            responder = emergency_mgr.events[event_id].first_arrival_vehicle
            self.metrics.record_emergency_event_response(event_id, response_time, responder)
            try:
                event_data = emergency_mgr.get_event_snapshot(event_id) or {}
                event = emergency_mgr.events[event_id]
                self.metrics.record_emergency_event(
                    event_id,
                    event_data,
                    response_time,
                    responder,
                    arrived_after_incident_end=event.arrived_after_incident_end,
                    center_edge=event.center_edge,
                )
            except Exception:
                pass

    @staticmethod
    def _is_emergency_vehicle(vehicle_id: str) -> bool:
        try:
            return traci.vehicle.getTypeID(vehicle_id) in EMERGENCY_VEHICLE_TYPES
        except traci.TraCIException:
            return False

    @staticmethod
    def _edge_is_near_emergency(edge_id: str, emergency_context: dict) -> bool:
        for event_data in emergency_context.values():
            blocked_edges = event_data.get("blocked_edges", [])
            if edge_id in blocked_edges:
                return True
        return False
