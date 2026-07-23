"""Runtime bridge that connects the P3 protocol layer to SUMO decision making."""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import traci

from algorithms.aco_pso import get_outgoing_edges, pheromone, update_pheromone, velocity
from controller.reroute_logic import should_reroute
from emergency_manager import get_emergency_manager
from p3_hybrid_router import HybridRouter, HybridContext
from p3_metrics import P3MetricsLogger
from p3_protocol import ProtocolStateManager, build_emergency_message, build_message
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES


class P3RuntimeCoordinator:
    """Coordinate protocol broadcasts and hybrid decisions during a SUMO run."""

    def __init__(self, freshness_window: float = 3.0) -> None:
        self.protocol_state = ProtocolStateManager(freshness_window=freshness_window)
        self.router = HybridRouter()
        self.metrics = P3MetricsLogger()
        self.last_broadcast: dict[str, float] = {}
        self.message_interval = 2.0

    def update_step(self, sim_time: float) -> None:
        """Broadcast protocol messages and apply hybrid route decisions for the current step."""

        try:
            self._broadcast_vehicle_messages(sim_time)
        except Exception:
            pass
        try:
            self._track_emergency_responses(sim_time)
        except Exception:
            pass
        try:
            self._record_vehicle_metrics(sim_time)
        except Exception:
            pass
        try:
            for vehicle_id in traci.vehicle.getIDList():
                try:
                    if not should_reroute(vehicle_id):
                        continue
                    self._apply_hybrid_decision(vehicle_id, sim_time)
                except traci.TraCIException:
                    continue
                except Exception:
                    continue
        except Exception:
            pass
        try:
            self._record_arrivals(sim_time)
        except Exception:
            pass

    def _broadcast_vehicle_messages(self, sim_time: float) -> None:
        for vehicle_id in traci.vehicle.getIDList():
            try:
                last_time = self.last_broadcast.get(vehicle_id, -float("inf"))
                if sim_time - last_time < self.message_interval:
                    continue

                vehicle_type = traci.vehicle.getTypeID(vehicle_id)
                current_edge = traci.vehicle.getRoadID(vehicle_id)
                speed = traci.vehicle.getSpeed(vehicle_id)
                emergency_flag = vehicle_type in EMERGENCY_VEHICLE_TYPES
                priority_level = 4 if emergency_flag else 0
                route = traci.vehicle.getRoute(vehicle_id)
                next_edge = route[1] if len(route) > 1 else None
                congestion_estimate = self._local_congestion(current_edge)
                yield_request = False
                distance_to_emergency = None
                emergency_mgr = get_emergency_manager()
                nearest = emergency_mgr.get_nearest_emergency(vehicle_id)
                if nearest:
                    _, distance, _ = nearest
                    distance_to_emergency = float(distance)
                    if distance_to_emergency <= 200 and not emergency_flag:
                        yield_request = True
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
        try:
            current_edge = traci.vehicle.getRoadID(vehicle_id)
            outgoing_edges = get_outgoing_edges(current_edge)
            if not outgoing_edges:
                return

            current_route = list(traci.vehicle.getRoute(vehicle_id))
            if len(current_route) < 2:
                return

            emergency_mgr = get_emergency_manager()
            emergency_context = emergency_mgr.get_active_emergency_zones()
            messages = self.protocol_state.get_fresh_neighbors(sim_time)
            emergency_neighbors = self.protocol_state.get_emergency_neighbors(sim_time)
            self.metrics.record_neighbor_state(len(messages), len(emergency_neighbors))
            self.metrics.record_emergency_zone_snapshot(len(emergency_context))
            is_emergency = self._is_emergency_vehicle(vehicle_id)
            candidates: List[Tuple[str, HybridContext]] = []
            for candidate_edge in outgoing_edges:
                heuristic = self._heuristic(candidate_edge)
                protocol_weight = self.router.protocol_weight_for_edge(
                    candidate_edge,
                    messages,
                    is_emergency_vehicle=is_emergency,
                )

                if emergency_context and self._edge_is_near_emergency(candidate_edge, emergency_context):
                    protocol_weight = min(1.0, protocol_weight + 0.25)

                pheromone_value = pheromone.get(candidate_edge, 1.0)
                velocity_value = velocity.get(candidate_edge, 0.0)
                context = self.router.build_context(
                    pheromone=pheromone_value,
                    heuristic=heuristic,
                    edge_id=candidate_edge,
                    messages=messages,
                    velocity=velocity_value,
                    is_emergency_vehicle=is_emergency,
                )
                candidates.append((candidate_edge, context))

            self.metrics.record_candidate_scoring(len(candidates))
            selected_edge, _ = self.router.select_best_edge(candidates)
            if not selected_edge:
                self.metrics.record_reroute(vehicle_id, False, is_emergency, bool(emergency_context))
                return

            self.router.update_exploration_memory(outgoing_edges, selected_edge)
            self.router.decay_scout_memory()

            if selected_edge == current_edge:
                return

            if current_edge in current_route:
                index = current_route.index(current_edge)
                new_route = current_route[: index + 1] + [selected_edge]
            else:
                new_route = current_route

            if new_route != current_route:
                self.metrics.record_reroute(vehicle_id, True, is_emergency, bool(emergency_context))
                self.metrics.record_vehicle_reroute(vehicle_id)
                try:
                    traci.vehicle.setRoute(vehicle_id, new_route)
                    update_pheromone(new_route)
                except traci.TraCIException:
                    pass
            else:
                self.metrics.record_reroute(vehicle_id, False, is_emergency, bool(emergency_context))
        except traci.TraCIException:
            return

    @staticmethod
    def _heuristic(edge_id: str) -> float:
        try:
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
        except traci.TraCIException:
            vehicle_count = 0
        return 1.0 / (1.0 + max(vehicle_count, 0))

    @staticmethod
    def _local_congestion(edge_id: str) -> float:
        try:
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
            mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
            congestion = min(1.0, max(0.0, 1.0 - (mean_speed / 15.0)))
            return round(min(1.0, congestion + vehicle_count * 0.01), 6)
        except traci.TraCIException:
            return 0.0

    def save_metrics(self, run_name: str = "p3_hybrid") -> None:
        self.metrics.save_results(run_name=run_name)

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
                    pass
        except Exception:
            pass

    def _track_emergency_responses(self, sim_time: float) -> None:
        emergency_mgr = get_emergency_manager()
        active_events = emergency_mgr.get_active_emergency_zones()
        if not active_events:
            return

        for vehicle_id in traci.vehicle.getIDList():
            if not self._is_emergency_vehicle(vehicle_id):
                continue

            try:
                current_edge = traci.vehicle.getRoadID(vehicle_id)
            except traci.TraCIException:
                continue

            for event_id, event_data in active_events.items():
                if current_edge in event_data.get("blocked_edges", []):
                    response_time = emergency_mgr.register_response_vehicle(vehicle_id, event_id, sim_time)
                    if response_time is not None:
                        self.metrics.record_emergency_event_response(event_id, response_time)
                    break

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
