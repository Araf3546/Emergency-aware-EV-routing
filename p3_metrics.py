import csv
import os
from typing import Dict, Optional, Set
import json
import time


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
        self.yield_actions_performed = 0
        self.candidate_edges_scored = 0
        self.emergency_event_responses = 0
        self.emergency_response_times: list[float] = []
        self.vehicles_rerouted: Set[str] = set()
        self.per_vehicle: Dict[str, Dict[str, int]] = {}
        self.vehicle_logs: Dict[str, Dict[str, Optional[float]]] = {}
        self.congestion_timeseries: list[tuple[float, float]] = []
        self.emergency_events: list[Dict[str, object]] = []
        self.run_metadata: Dict[str, object] = {}

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

    def record_yield_action(self, vehicle_id: str) -> None:
        """Count a yield the protocol actually performed.

        This counts the times an ordinary vehicle was asked to move aside for a responder. Since the
        unified rule (2026-09-15) that is the only yielding manoeuvre: nothing is braked, and a car
        that keeps driving because it cannot make room, or is not in the way, is not counted. Before
        that, every yield braked, so runs recorded then count every car asked.
        `yield_requests` counts the broadcasts that carried the flag. Since the clean design
        (2026-09-15) that means broadcasts made in the 3 s after such a yield, so the two counts are
        tied. Before that, the flag meant "within 200 m of an open incident", and runs recorded
        then show far higher counts.
        """
        self.yield_actions_performed += 1
        vehicle_metrics = self.per_vehicle.setdefault(vehicle_id, {
            "broadcasts": 0,
            "emergency_broadcasts": 0,
            "yield_requests": 0,
            "reroutes": 0,
        })
        vehicle_metrics["yield_actions"] = vehicle_metrics.get("yield_actions", 0) + 1

    def record_emergency_zone_snapshot(self, active_zone_count: int) -> None:
        self.emergency_zones_observed += active_zone_count

    def record_candidate_scoring(self, candidate_count: int) -> None:
        self.candidate_edges_scored += candidate_count

    def record_emergency_event_response(
        self,
        event_id: str,
        response_time: float,
        responder_id: Optional[str] = None,
    ) -> None:
        """Record a measured response, and attach it to the responder's own log.

        Audit M10: `vehicle_log["emergency_response_time"]` was initialised to None in two
        places and never assigned anywhere, so the CSV's EmergencyResponseTime column read
        "N/A" for every vehicle in every run even when a response had been detected.
        """
        self.emergency_event_responses += 1
        self.emergency_response_times.append(response_time)
        if responder_id:
            vehicle_log = self.vehicle_logs.setdefault(responder_id, {})
            if vehicle_log.get("emergency_response_time") is None:
                vehicle_log["emergency_response_time"] = float(response_time)

    def record_emergency_event(
        self,
        event_id: str,
        event_data: Dict[str, object],
        response_time: Optional[float],
        first_arrival_vehicle: Optional[str] = None,
        arrived_after_incident_end: Optional[bool] = None,
        center_edge: Optional[str] = None,
    ) -> None:
        """Append one incident to the event log.

        The field names match the log `metrics/metrics_logger.py` writes, so both runners
        produce the same schema. They used to differ - this one wrote `first_arrival_vehicle`
        where the other wrote `vehicle_id` - which left the responder-delay metric of audit
        M11 with nothing to look up.
        """
        self.emergency_events.append(
            {
                "event_id": event_id,
                "start_time": event_data.get("start_time"),
                "end_time": event_data.get("end_time"),
                "blocked_edges": event_data.get("blocked_edges"),
                "center_edge": center_edge or event_data.get("center_edge"),
                "vehicle_id": first_arrival_vehicle,
                "first_arrival_vehicle": first_arrival_vehicle,
                "response_time": response_time,
                "arrived_after_incident_end": arrived_after_incident_end,
            }
        )

    def record_congestion(self, sim_time: float, congestion: float) -> None:
        self.congestion_timeseries.append((sim_time, round(float(congestion), 6)))

    def save_results(self, run_name: str = "p3_hybrid") -> None:
        base = os.path.join("results", run_name)
        os.makedirs(base, exist_ok=True)

        csv_path = os.path.join(base, f"{run_name}_protocol_metrics.csv")
        summary_path = os.path.join(base, f"{run_name}_summary.txt")
        congestion_path = os.path.join(base, f"{run_name}_congestion_timeseries.csv")
        emergency_events_path = os.path.join(base, f"{run_name}_emergency_events.jsonl")
        metadata_path = os.path.join(base, f"{run_name}_metadata.yaml")

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

        # Write congestion time-series CSV
        try:
            with open(congestion_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["sim_time", "congestion"])
                for sim_time, congestion in self.congestion_timeseries:
                    writer.writerow([sim_time, congestion])
        except Exception:
            pass

        # Write emergency events as JSONL
        try:
            with open(emergency_events_path, "w", newline="") as f:
                for ev in self.emergency_events:
                    f.write(json.dumps(ev) + "\n")
        except Exception:
            pass

        # Write run metadata (simple YAML-like file)
        try:
            now = time.time()
            metadata = {
                "run_name": run_name,
                "generated_at": now,
                "total_vehicles_tracked": len(self.per_vehicle),
                "total_reroutes_executed": self.reroute_executed,
                "total_emergency_events": len(self.emergency_events),
            }
            with open(metadata_path, "w") as f:
                for k, v in metadata.items():
                    f.write(f"{k}: {v}\n")
        except Exception:
            pass

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
