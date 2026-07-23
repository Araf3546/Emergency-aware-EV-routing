import csv
import os
from typing import Dict, Optional, Set


class P3MetricsLogger:
    """Collect runtime metrics for the E3 Hybrid protocol and reroute behavior."""

    def __init__(self) -> None:
        self.protocol_messages = 0
        self.emergency_messages = 0
        self.yield_requests = 0
        self.vehicle_broadcasts = 0
        self.reroute_decisions = 0
        self.reroute_executed = 0
        self.emergency_reroutes = 0
        self.fresh_neighbor_messages = 0
        self.emergency_neighbor_messages = 0
        self.emergency_zones_observed = 0
        self.candidate_edges_scored = 0
        self.emergency_event_responses = 0
        self.emergency_response_times: list[float] = []
        self.vehicles_rerouted: Set[str] = set()
        self.per_vehicle: Dict[str, Dict[str, int]] = {}
        self.vehicle_logs: Dict[str, Dict[str, Optional[float]]] = {}

    def record_broadcast(
        self,
        vehicle_id: str,
        emergency_flag: bool,
        yield_request: bool,
    ) -> None:
        self.vehicle_broadcasts += 1
        if emergency_flag:
            self.emergency_messages += 1
        if yield_request:
            self.yield_requests += 1

        vehicle_metrics = self.per_vehicle.setdefault(vehicle_id, {
            "broadcasts": 0,
            "emergency_broadcasts": 0,
            "yield_requests": 0,
            "reroutes": 0,
        })
        vehicle_metrics["broadcasts"] += 1
        if emergency_flag:
            vehicle_metrics["emergency_broadcasts"] += 1
        if yield_request:
            vehicle_metrics["yield_requests"] += 1

    def record_reroute(
        self,
        vehicle_id: str,
        did_reroute: bool,
        is_emergency_vehicle: bool,
        emergency_context_active: bool,
    ) -> None:
        self.reroute_decisions += 1
        if did_reroute:
            self.reroute_executed += 1
            self.vehicles_rerouted.add(vehicle_id)
            if is_emergency_vehicle or emergency_context_active:
                self.emergency_reroutes += 1

        vehicle_metrics = self.per_vehicle.setdefault(vehicle_id, {
            "broadcasts": 0,
            "emergency_broadcasts": 0,
            "yield_requests": 0,
            "reroutes": 0,
        })
        if did_reroute:
            vehicle_metrics["reroutes"] += 1

    def record_vehicle_step(
        self,
        vehicle_id: str,
        sim_time: float,
        vehicle_type: str,
        is_emergency: bool,
        speed: float,
        waiting_time: float,
        distance: float,
    ) -> None:
        vehicle_log = self.vehicle_logs.setdefault(vehicle_id, {
            "start_time": sim_time,
            "end_time": None,
            "vehicle_type": vehicle_type,
            "is_emergency": is_emergency,
            "waiting_time": waiting_time,
            "distance": distance,
            "max_speed_observed": speed,
            "total_reroutes": 0,
            "emergency_response_time": None,
        })

        if vehicle_log["start_time"] is None:
            vehicle_log["start_time"] = sim_time

        vehicle_log["vehicle_type"] = vehicle_type
        vehicle_log["is_emergency"] = is_emergency
        vehicle_log["waiting_time"] = waiting_time
        vehicle_log["distance"] = distance
        vehicle_log["max_speed_observed"] = max(vehicle_log.get("max_speed_observed", 0.0), speed)

    def record_vehicle_arrival(self, vehicle_id: str, sim_time: float) -> None:
        vehicle_log = self.vehicle_logs.get(vehicle_id)
        if vehicle_log is None:
            return
        if vehicle_log["end_time"] is None:
            vehicle_log["end_time"] = sim_time

    def record_vehicle_reroute(self, vehicle_id: str) -> None:
        vehicle_log = self.vehicle_logs.setdefault(vehicle_id, {
            "start_time": None,
            "end_time": None,
            "vehicle_type": "unknown",
            "is_emergency": False,
            "waiting_time": 0.0,
            "distance": 0.0,
            "max_speed_observed": 0.0,
            "total_reroutes": 0,
            "emergency_response_time": None,
        })
        vehicle_log["total_reroutes"] += 1

    def record_neighbor_state(
        self,
        fresh_count: int,
        emergency_count: int,
    ) -> None:
        self.fresh_neighbor_messages += fresh_count
        self.emergency_neighbor_messages += emergency_count

    def record_emergency_zone_snapshot(self, active_zone_count: int) -> None:
        self.emergency_zones_observed += active_zone_count

    def record_candidate_scoring(self, candidate_count: int) -> None:
        self.candidate_edges_scored += candidate_count

    def record_emergency_event_response(
        self,
        event_id: str,
        response_time: float,
    ) -> None:
        self.emergency_event_responses += 1
        self.emergency_response_times.append(response_time)

    def save_results(self, run_name: str = "p3_hybrid") -> None:
        os.makedirs("results", exist_ok=True)

        csv_path = os.path.join("results", f"{run_name}_protocol_metrics.csv")
        summary_path = os.path.join("results", f"{run_name}_summary.txt")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "VehicleID",
                "VehicleType",
                "IsEmergency",
                "StartTime",
                "EndTime",
                "TravelTime",
                "WaitingTime",
                "Distance",
                "MaxSpeedObserved",
                "Broadcasts",
                "EmergencyBroadcasts",
                "YieldRequests",
                "Reroutes",
                "EmergencyResponseTime"
            ])
            for vehicle_id, vehicle_metrics in sorted(self.per_vehicle.items()):
                vehicle_log = self.vehicle_logs.get(vehicle_id, {})
                travel_time = None
                if vehicle_log.get("end_time") is not None and vehicle_log.get("start_time") is not None:
                    travel_time = vehicle_log["end_time"] - vehicle_log["start_time"]

                writer.writerow([
                    vehicle_id,
                    vehicle_log.get("vehicle_type", "unknown"),
                    "Yes" if vehicle_log.get("is_emergency") else "No",
                    vehicle_log.get("start_time"),
                    vehicle_log.get("end_time"),
                    round(travel_time, 2) if travel_time is not None else "N/A",
                    round(vehicle_log.get("waiting_time", 0.0), 2),
                    round(vehicle_log.get("distance", 0.0), 2),
                    round(vehicle_log.get("max_speed_observed", 0.0), 2),
                    vehicle_metrics.get("broadcasts", 0),
                    vehicle_metrics.get("emergency_broadcasts", 0),
                    vehicle_metrics.get("yield_requests", 0),
                    vehicle_metrics.get("reroutes", 0),
                    round(vehicle_log.get("emergency_response_time", 0.0), 2) if vehicle_log.get("emergency_response_time") is not None else "N/A",
                ])

        with open(summary_path, "w") as f:
            f.write(f"=== E3 HYBRID METRICS SUMMARY ===\n\n")
            f.write(f"Total broadcasts: {self.vehicle_broadcasts}\n")
            f.write(f"Emergency broadcasts: {self.emergency_messages}\n")
            f.write(f"Yield requests: {self.yield_requests}\n")
            f.write(f"Fresh neighbor message count: {self.fresh_neighbor_messages}\n")
            f.write(f"Emergency neighbor message count: {self.emergency_neighbor_messages}\n")
            f.write(f"Emergency zone snapshots observed: {self.emergency_zones_observed}\n")
            f.write(f"Candidate edge scores evaluated: {self.candidate_edges_scored}\n")
            f.write(f"Emergency event responses recorded: {self.emergency_event_responses}\n")
            if self.emergency_response_times:
                avg_response = sum(self.emergency_response_times) / len(self.emergency_response_times)
                f.write(f"Average emergency response time: {avg_response:.2f} seconds\n")
            f.write(f"Reroute decisions considered: {self.reroute_decisions}\n")
            f.write(f"Reroutes executed: {self.reroute_executed}\n")
            f.write(f"Vehicles rerouted: {len(self.vehicles_rerouted)}\n")
            f.write(f"Emergency-influenced reroutes: {self.emergency_reroutes}\n")
            f.write("\n=== NOTES ===\n")
            f.write("This log captures protocol broadcasts, neighbor state, and reroute execution counts for the E3 Hybrid runtime.\n")
