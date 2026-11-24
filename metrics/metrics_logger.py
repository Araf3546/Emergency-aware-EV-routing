"""Comparison runner: one SUMO run per algorithm, with contract-defined metrics.

Rewritten for Step 4 of the audit repair plan. What changed and why:

* M1 - waiting time was accumulated with `+=` every step from `getWaitingTime()`, which is
  already a running counter, so a 10-second stop contributed 55 seconds. It now comes from
  `tripinfo.xml`'s `waitingTime`, which is the contract's "SUMO accumulated waiting time".
* M3, M4, M5 - congestion was `1 - mean_edge_speed / 15.0` averaged over all 21,385 edges,
  which is a property of the road graph rather than the traffic, plus a separate confounded
  emergency/normal speed ratio written into every row. Congestion is now mean `timeLoss` in
  seconds per vehicle, which is what contract section 8 and decision 13.4 require.
* M2, C12 - reroutes were counted before the algorithm ran, so `baseline` reported reroutes
  and rerouting efficiency was always 100%. They are now counted only when SUMO accepts a
  changed route, via `route_utils.set_route_checked`.
* M6 - teleports come from `summary.xml`'s cumulative counter, not a per-step maximum.
* M7 - travel time is `duration + departDelay` from tripinfo, so time spent waiting to be
  inserted is included, as contract section 8 requires.
* R3 - every algorithm used to write its SUMO output to the same directory, overwriting the
  previous run. Each algorithm now writes to `results/<algo>/`.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse
import csv
import json

import traci

# 2026-09-17: the routing loop moved, unchanged, into algorithms/comparator_routing.py so that the V1
# comparison runner drives the same code (decisions B2 and B5).
from algorithms.comparator_routing import METHODS, RoutingLoop
from algorithms.route_utils import (
    get_route_stats,
    reset_route_stats,
    seed_route_choice,
)
from emergency_manager import get_emergency_manager
from evaluation_seeds import route_choice_seed, sumo_seed_args
from metrics.trip_metrics import build_report, load_tripinfo
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES

SUMO_BINARY = "sumo"   # use "sumo-gui" for visualization
SUMO_CFG = "simulation.sumocfg"
# The contracted scenario is 3,600 s. The old bound of 7,000 exceeded SUMO's own end time
# (audit R2); the run length now comes from the scenario duration unless overridden.
DEFAULT_SIM_TIME = 3600


def run_metrics(algo_name, duration=DEFAULT_SIM_TIME, sumo_cfg=SUMO_CFG, seed=None):
    # `seed` is an evaluation seed (contract 13.15): SUMO's seed, that seed's frozen demand
    # file and the route-choice seed together, via evaluation_seeds.py. None runs
    # simulation.sumocfg as it is, with the route-choice seed from config/scenario.yaml.
    seed_args = sumo_seed_args(seed, sumo_cfg)

    # Contract section 5: randomness uses an explicit seed from config/scenario.yaml.
    route_seed = seed_route_choice(route_choice_seed(seed))
    reset_route_stats()
    print(f"[INFO] route-choice seed: {route_seed}; PYTHONHASHSEED={os.environ.get('PYTHONHASHSEED', 'unset')}")

    # R3: per-algorithm output directory, so runs cannot overwrite each other's tripinfo.
    # Each evaluation seed gets its own subdirectory for the same reason.
    run_dir = os.path.join("results", algo_name)
    if seed is not None:
        run_dir = os.path.join(run_dir, f"seed_{seed}")
    os.makedirs(run_dir, exist_ok=True)
    tripinfo_path = os.path.join(run_dir, "tripinfo.xml")
    summary_path = os.path.join(run_dir, "summary.xml")

    traci.start([
        SUMO_BINARY, "-c", sumo_cfg,
        "--no-step-log", "true",
        "--end", str(duration),
        "--tripinfo-output", tripinfo_path,
        "--summary-output", summary_path,
        "--error-log", os.path.join(run_dir, "sumo_errors.log"),
        *seed_args,
    ])

    # The routing state and counters; E3's legacy arm is initialised here, as before.
    loop = RoutingLoop(algo_name)

    emergency_events = []
    emergency_mgr = get_emergency_manager()
    emergency_mgr.load_canonical_scenario_file()
    emergency_mgr.generate_emergencies()

    while traci.simulation.getTime() < duration:
        traci.simulationStep()
        sim_time = traci.simulation.getTime()

        emergency_mgr.update(sim_time)
        loop.before_step(sim_time, emergency_mgr.blocked_edges)

        # Emergency response tracking, shared with every other runner so there is one
        # definition of arrival (audit M9). Not gated on the incident still being active.
        for event_id, response_time in emergency_mgr.record_responder_arrivals(
            sim_time,
            vehicle_ids=traci.vehicle.getIDList(),
            arrived_ids=traci.simulation.getArrivedIDList(),
        ).items():
            event_state = emergency_mgr.get_response_summary()[event_id]
            emergency_events.append({
                "event_id": event_id,
                "start_time": event_state["start_time"],
                "end_time": event_state["end_time"],
                "first_arrival_time": sim_time,
                "response_time": response_time,
                "vehicle_id": event_state["first_arrival_vehicle"],
                "center_edge": event_state["center_edge"],
                "arrived_after_incident_end": event_state["arrived_after_incident_end"],
            })

        # ---------------- ROUTING ----------------
        loop.route_vehicles(sim_time)

    traci.close()

    # ============ METRICS FROM SUMO'S OWN OUTPUT ================
    route_stats = get_route_stats()
    route_stats["route_choice_seed"] = route_seed
    route_stats["pythonhashseed"] = os.environ.get("PYTHONHASHSEED", "unset")

    vehicles_needing_reroute = loop.vehicles_needing_reroute
    vehicles_rerouted = loop.vehicles_rerouted
    reroute_attempts = loop.reroute_attempts
    reroutes_applied = loop.reroutes_applied
    per_vehicle_reroutes = loop.per_vehicle_reroutes
    vehicle_types = loop.vehicle_types

    report = build_report(
        tripinfo_path,
        summary_path,
        emergency_types=EMERGENCY_VEHICLE_TYPES,
        applied_reroutes=reroutes_applied,
    )
    trips = load_tripinfo(tripinfo_path)

    rerouting_coverage = (
        len(vehicles_rerouted) / len(vehicle_types) * 100 if vehicle_types else 0.0
    )
    # M2: the share of vehicles that met the trigger and actually got a new route. This can
    # now be less than 100%, which is the point.
    rerouting_efficiency = (
        len(vehicles_rerouted) / len(vehicles_needing_reroute) * 100
        if vehicles_needing_reroute
        else 0.0
    )

    run_metadata = {
        "algorithm": algo_name,
        "sumo_cfg": sumo_cfg,
        # None means simulation.sumocfg's own seed and demand file (20260722).
        "evaluation_seed": seed,
        "duration_seconds": duration,
        "vehicles_seen_in_network": len(vehicle_types),
        "vehicles_needing_reroute": len(vehicles_needing_reroute),
        "vehicles_rerouted": len(vehicles_rerouted),
        "reroute_attempts": reroute_attempts,
        "reroutes_applied": reroutes_applied,
        "rerouting_coverage_percent": round(rerouting_coverage, 2),
        "rerouting_efficiency_percent": round(rerouting_efficiency, 2),
        "emergency_events_measured": len(emergency_events),
        "route_stats": route_stats,
        "trip_metrics": report,
    }

    # ============ PER-VEHICLE CSV, STRAIGHT FROM tripinfo ================
    output_file = os.path.join(run_dir, f"{algo_name}_results.csv")
    with open(output_file, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "VehicleID",
            "VehicleType",
            "IsEmergency",
            "TravelTime",        # arrival - scheduled departure (M7)
            "InNetworkDuration",
            "DepartDelay",
            "WaitingTime",       # SUMO accumulated waiting time (M1)
            "TimeLoss",          # contract congestion metric, decision 13.4 (M3)
            "Distance",
            "Rerouted",
            "TotalReroutes",     # successful route replacements only (M2)
            "SumoRerouteNo",
            "EmergencyResponseTime",
        ])
        response_by_vehicle = {
            event["vehicle_id"]: event["response_time"] for event in emergency_events
        }
        for veh, record in sorted(trips.items()):
            vehicle_type = record.get("vehicle_type", "")
            writer.writerow([
                veh,
                vehicle_type,
                "Yes" if vehicle_type in EMERGENCY_VEHICLE_TYPES else "No",
                round(float(record["scheduled_travel_time"]), 2),
                round(float(record.get("duration", 0.0)), 2),
                round(float(record.get("departDelay", 0.0)), 2),
                round(float(record.get("waitingTime", 0.0)), 2),
                round(float(record.get("timeLoss", 0.0)), 2),
                round(float(record.get("routeLength", 0.0)), 2),
                "Yes" if per_vehicle_reroutes.get(veh) else "No",
                per_vehicle_reroutes.get(veh, 0),
                int(record.get("rerouteNo", 0)),
                round(response_by_vehicle[veh], 2) if veh in response_by_vehicle else "N/A",
            ])

    # ============ SUMMARY ================
    summary_file = os.path.join(run_dir, f"{algo_name}_summary.txt")
    with open(summary_file, "w") as handle:
        handle.write(f"=== SIMULATION SUMMARY: {algo_name.upper()} ===\n\n")
        handle.write(f"Algorithm: {algo_name}\n")
        handle.write(f"Simulation duration: {duration} s\n")
        handle.write(f"Route-choice seed: {route_seed}\n\n")

        handle.write("=== DEMAND AND COMPLETION (contract section 5) ===\n")
        for key in (
            "loaded_vehicles", "inserted_vehicles", "ended_vehicles", "still_running_at_end",
            "never_inserted_vehicles", "incomplete_trips", "teleports_total",
            "collisions_total", "insertion_rate", "completion_rate",
        ):
            if key in report:
                handle.write(f"{key}: {report[key]}\n")
        handle.write("\n")

        handle.write("=== TRIP METRICS, FROM tripinfo.xml (contract section 8) ===\n")
        for key in (
            "completed_trips", "mean_travel_time_s", "median_travel_time_s",
            "mean_in_network_duration_s", "mean_depart_delay_s",
            "mean_waiting_time_s", "median_waiting_time_s",
            "mean_time_loss_s", "median_time_loss_s", "mean_route_length_m",
        ):
            handle.write(f"{key}: {report.get(key)}\n")
        handle.write("\nCongestion is mean time loss in seconds per vehicle, per contract\n")
        handle.write("section 8 and decision 13.4. It is not a speed proxy.\n\n")

        handle.write("=== EMERGENCY VEHICLES ===\n")
        handle.write(f"completed_emergency_trips: {report.get('completed_emergency_trips')}\n")
        handle.write(f"mean_emergency_travel_time_s: {report.get('mean_emergency_travel_time_s')}\n")
        handle.write(f"mean_emergency_time_loss_s: {report.get('mean_emergency_time_loss_s')}\n")
        handle.write(f"mean_normal_time_loss_s: {report.get('mean_normal_time_loss_s')}\n")
        handle.write(f"incidents_with_measured_response: {len(emergency_events)}\n")
        for event in emergency_events:
            handle.write(
                f"  event {event['event_id']}: {event['response_time']:.1f} s "
                f"by {event['vehicle_id']}\n"
            )
        handle.write("\n")

        handle.write("=== REROUTING (successful replacements only, audit M2/C12) ===\n")
        handle.write(f"Vehicles seen: {len(vehicle_types)}\n")
        handle.write(f"Vehicles meeting the reroute trigger: {len(vehicles_needing_reroute)}\n")
        handle.write(f"Vehicles actually rerouted: {len(vehicles_rerouted)}\n")
        handle.write(f"Reroute attempts: {reroute_attempts}\n")
        handle.write(f"Reroutes applied: {reroutes_applied}\n")
        handle.write(f"Rerouting coverage: {rerouting_coverage:.2f}%\n")
        handle.write(f"Rerouting efficiency: {rerouting_efficiency:.2f}%\n\n")

        handle.write("=== ROUTE VALIDITY (contract section 5) ===\n")
        for key, value in route_stats.items():
            handle.write(f"{key}: {value}\n")

    with open(os.path.join(run_dir, f"{algo_name}_metrics.json"), "w") as handle:
        json.dump(run_metadata, handle, indent=2)

    with open(os.path.join(run_dir, f"{algo_name}_emergency_events.jsonl"), "w", newline="") as handle:
        for event in emergency_events:
            handle.write(json.dumps(event) + "\n")

    print(f"[INFO] Results:  {output_file}")
    print(f"[INFO] Summary:  {summary_file}")
    print(f"[INFO] Completed trips: {report.get('completed_trips')}; "
          f"mean travel time {report.get('mean_travel_time_s')} s; "
          f"mean time loss {report.get('mean_time_loss_s')} s")
    print(f"[INFO] Reroutes applied: {reroutes_applied} of {reroute_attempts} attempts "
          f"({rerouting_efficiency:.1f}% of vehicles needing a reroute)")
    return run_metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--algo",
        required=True,
        choices=list(METHODS),
        help="Routing algorithm to evaluate",
    )
    parser.add_argument("--duration", type=int, default=DEFAULT_SIM_TIME)
    parser.add_argument("--sumo-cfg", default=SUMO_CFG)
    parser.add_argument("--seed", type=int, default=None,
                        help="evaluation seed (contract 13.15); results under results/<algo>/seed_<seed>/")
    args = parser.parse_args()

    run_metrics(args.algo, duration=args.duration, sumo_cfg=args.sumo_cfg, seed=args.seed)
