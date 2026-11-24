#!/usr/bin/env python3
"""Design-choice comparison: the current reroute trigger vs junction-triggered rerouting.

One run of one condition, fully instrumented. The two conditions differ ONLY in which trigger
decides when E3 is asked for a decision:

  --trigger current    controller/reroute_logic.should_reroute, exactly as it is
  --trigger junction   controller/junction_trigger.JunctionTrigger, patched in for this run
                       only - the production runners are not changed
  --trigger none       the plain baseline: no coordinator, no E3, no rerouting at all. Incidents
                       still close roads and responder arrivals are still measured, because
                       those are the scenario and the metric, not the treatment

Everything else is the P3 runtime as it stands: same E3 (scoring, weights, BCO, PSO,
pheromone, heuristic), same emergency layer switch, same frozen demand, seed, incidents and
duration. `--seed` selects an evaluation or tuning seed through evaluation_seeds.py.

Both conditions are instrumented identically:

* every trigger firing ("reroute request"), split into the emergency-type clause (PD-1) and
  the ordinary condition;
* every E3 decision: candidates, how many distinct meaningful options it had
  (controller/decision_points.py, tolerance 0.5), whether a route was applied, and the vehicle's
  remaining route length immediately before and after (so every reroute's detour is known);
* network outcomes from tripinfo/summary (metrics/trip_metrics.build_report), total distance
  driven including vehicles still running, emergency responses, teleports, yields;
* time spent in the trigger and in E3 decisions;
* every vehicle's speed factor when first seen, compared with the seed's frozen file (pairing).

    python scripts/diagnostics/trigger_comparison.py --trigger junction --layer off \
        --seed 20260721 --duration 3600 --out DIR

Writes only into DIR.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402


class _ZeroMetrics:
    """The baseline arm runs no coordinator, so every routing counter is zero."""
    reroute_decisions = reroute_executed = emergency_reroutes = 0
    yield_actions_performed = yield_requests = 0
    vehicles_rerouted: set = set()
    candidate_edges_scored = 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trigger", choices=("current", "junction", "none"), required=True,
                        help="none = the plain baseline: no E3, no coordinator, no rerouting")
    parser.add_argument("--layer", choices=("on", "off"), required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--duration", type=int, default=3600)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    import p3_runtime
    from algorithms import e3_core
    from algorithms.route_utils import get_route_stats, remaining_route
    from controller.decision_points import DecisionPoints
    from emergency_manager import get_emergency_manager
    from evaluation_seeds import frozen_demand_paths, route_choice_seed, sumo_seed_args
    from metrics.trip_metrics import build_report
    from p3_simulation_config import EMERGENCY_VEHICLE_TYPES

    seed_args = sumo_seed_args(args.seed)
    points = DecisionPoints()
    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    # ------------------------------------------------------------ the one difference
    if args.trigger == "none":
        trigger = None                      # baseline arm: nothing ever triggers a reroute
    elif args.trigger == "junction":
        from controller.junction_trigger import JunctionTrigger
        trigger = JunctionTrigger(points).should_reroute
    else:
        from controller.reroute_logic import should_reroute as trigger

    requests = Counter()
    timers = Counter()

    def counted_trigger(vehicle_id):
        started = time.perf_counter()
        fired = trigger(vehicle_id)
        timers["trigger_seconds"] += time.perf_counter() - started
        if fired:
            try:
                clause = (traci.vehicle.getTypeID(vehicle_id) in EMERGENCY_VEHICLE_TYPES
                          and manager.is_edge_blocked(traci.vehicle.getRoadID(vehicle_id)))
            except traci.TraCIException:
                clause = False
            requests["emergency_type_on_closed_edge" if clause else "ordinary_condition"] += 1
        return fired

    if trigger is not None:
        p3_runtime.should_reroute = counted_trigger

    # ------------------------------------------------------------ decision recording
    decisions = []
    captured = {}
    original_select = e3_core.E3Core.select_edge
    original_decide = e3_core.E3Core.decide

    def select_edge(self, vehicle_id, outgoing, destination, messages, is_emergency_vehicle=False):
        captured["candidates"] = list(outgoing)
        captured["destination"] = destination
        return original_select(self, vehicle_id, outgoing, destination, messages,
                               is_emergency_vehicle=is_emergency_vehicle)

    def decide(self, vehicle_id, sim_time, messages=None, is_emergency_vehicle=False):
        captured.clear()
        before = remaining_route(vehicle_id)
        started = time.perf_counter()
        report = original_decide(self, vehicle_id, sim_time, messages=messages,
                                 is_emergency_vehicle=is_emergency_vehicle)
        timers["e3_decide_seconds"] += time.perf_counter() - started
        candidates = captured.get("candidates")
        edge = before[0] if before else None
        # record: [time, vehicle, reason, candidates, meaningful options, applied, detour m, edge]
        if candidates is None:          # never reached select_edge: no candidates / no route
            decisions.append([sim_time, vehicle_id, report["reason"], 0, None, False, None, edge])
            return report
        options = points.classify(candidates, captured["destination"])["options"]
        delta = None
        if report["applied"] and before:
            delta = round(points.route_length(report["route"]) - points.route_length(before), 2)
        decisions.append([sim_time, vehicle_id, report["reason"], len(candidates), options,
                          bool(report["applied"]), delta, edge])
        return report

    e3_core.E3Core.select_edge = select_edge
    e3_core.E3Core.decide = decide

    coordinator = None if args.trigger == "none" else p3_runtime.P3RuntimeCoordinator(
        emergency_layer=(args.layer == "on"), route_choice_seed=route_choice_seed(args.seed))

    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
        "--end", str(args.duration),
        "--tripinfo-output", str(out / "tripinfo.xml"),
        "--summary-output", str(out / "summary.xml"),
        "--emission-output", str(out / "emissions.xml"),
        "--queue-output", str(out / "queue.xml"),
        "--message-log", str(out / "messages.log"),
        "--error-log", str(out / "errors.log"),
        *seed_args,
    ])
    speed_factors = {}
    running_distance = 0.0
    wall_started = time.perf_counter()
    try:
        while traci.simulation.getTime() < args.duration:
            traci.simulationStep()
            t = traci.simulation.getTime()
            manager.update(t)
            if coordinator is not None:
                coordinator.update_step(t)
            else:
                # The baseline still measures responder arrivals - that is the metric, shared
                # by every arm, and it changes no vehicle behaviour (scripts/16_compare does
                # exactly this).
                manager.record_responder_arrivals(
                    t, vehicle_ids=traci.vehicle.getIDList(),
                    arrived_ids=traci.simulation.getArrivedIDList())
            for vehicle_id in traci.simulation.getDepartedIDList():
                try:
                    speed_factors[vehicle_id] = traci.vehicle.getSpeedFactor(vehicle_id)
                except traci.TraCIException:
                    pass
        for vehicle_id in traci.vehicle.getIDList():
            try:
                running_distance += traci.vehicle.getDistance(vehicle_id)
            except traci.TraCIException:
                pass
        still_running = len(traci.vehicle.getIDList())
    finally:
        try:
            traci.close()
        except Exception:
            pass
    wall_seconds = time.perf_counter() - wall_started

    # ------------------------------------------------------------ summarise
    trips = build_report(out / "tripinfo.xml", out / "summary.xml", emergency_types=EMERGENCY_VEHICLE_TYPES)
    import xml.etree.ElementTree as ET
    completed_distance = sum(float(e.get("routeLength", 0.0))
                             for e in ET.parse(out / "tripinfo.xml").getroot().iter("tripinfo"))

    scored = [d for d in decisions if d[3] > 0]
    applied = [d for d in decisions if d[5]]
    deltas = [d[6] for d in applied if d[6] is not None]
    longer = [x for x in deltas if x > 1.0]
    shorter = [x for x in deltas if x < -1.0]
    metrics = coordinator.metrics if coordinator is not None else _ZeroMetrics()

    manifest_seed = args.seed if args.seed is not None else json.loads(
        (REPO_ROOT / "demand" / "porto.speed_factors.json").read_text(encoding="utf-8"))["sumo_seed"]
    frozen = {k: float(v) for k, v in json.loads(
        frozen_demand_paths(manifest_seed)[1].read_text(encoding="utf-8"))["speed_factors"].items()}
    common = [v for v in frozen if v in speed_factors]

    summary = {
        "trigger": args.trigger, "layer": args.layer, "seed": manifest_seed, "duration_s": args.duration,
        "network": {
            **{k: trips.get(k) for k in (
                "loaded_vehicles", "inserted_vehicles", "completed_trips", "completion_rate",
                "incomplete_trips", "never_inserted_vehicles", "still_running_at_end",
                "teleports_total", "collisions_total", "mean_travel_time_s", "median_travel_time_s",
                "mean_time_loss_s", "mean_waiting_time_s", "mean_route_length_m")},
            "distance_completed_trips_km": round(completed_distance / 1000.0, 1),
            "distance_still_running_km": round(running_distance / 1000.0, 1),
            "distance_total_km": round((completed_distance + running_distance) / 1000.0, 1),
        },
        "routing": {
            "reroute_requests": sum(requests.values()),
            "requests_by_clause": dict(requests),
            "decide_calls": len(decisions),
            "decide_without_candidates": len(decisions) - len(scored),
            "e3_decisions": len(scored),
            "coordinator_reroute_decisions": metrics.reroute_decisions,
            "decisions_one_candidate": sum(1 for d in scored if d[3] == 1),
            "decisions_two_plus_candidates": sum(1 for d in scored if d[3] >= 2),
            "decisions_real_choice": sum(1 for d in scored if d[4] >= 2),
            "pct_one_candidate": round(100.0 * sum(1 for d in scored if d[3] == 1) / len(scored), 2) if scored else None,
            "pct_real_choice": round(100.0 * sum(1 for d in scored if d[4] >= 2) / len(scored), 2) if scored else None,
            "reroutes_applied": len(applied),
            "coordinator_reroute_executed": metrics.reroute_executed,
            "applied_at_real_choice": sum(1 for d in applied if d[4] >= 2),
            "applied_without_real_choice": sum(1 for d in applied if d[4] < 2),
            "reroutes_shorten_pct": round(100.0 * len(shorter) / len(deltas), 1) if deltas else None,
            "reroutes_lengthen_pct": round(100.0 * len(longer) / len(deltas), 1) if deltas else None,
            "reroutes_unchanged_pct": round(100.0 * (len(deltas) - len(longer) - len(shorter)) / len(deltas), 1) if deltas else None,
            "detour_mean_m": round(statistics.mean(deltas), 1) if deltas else None,
            "detour_median_m": round(statistics.median(deltas), 1) if deltas else None,
            "detour_mean_when_longer_m": round(statistics.mean(longer), 1) if longer else None,
            "total_added_distance_m": round(sum(deltas), 1),
            "route_stats": get_route_stats(),
        },
        "emergency": {
            "responses": {str(k): v for k, v in manager.get_response_summary().items()},
            "teleports": manager.get_teleport_summary(),
            "yield_actions": metrics.yield_actions_performed,
            "yield_requests": metrics.yield_requests,
            "emergency_reroutes": metrics.emergency_reroutes,
        },
        "cost": {
            "wall_seconds": round(wall_seconds, 1),
            "trigger_seconds": round(timers["trigger_seconds"], 1),
            "e3_decide_seconds": round(timers["e3_decide_seconds"], 1),
            "destinations_cached": len(points._to_destination),
        },
        "pairing": {
            "ordinary_vehicles_compared": len(common),
            "speed_factor_differences": sum(1 for v in common if speed_factors[v] != frozen[v]),
        },
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    (out / "decisions.json").write_text(json.dumps(decisions), encoding="utf-8")
    print(json.dumps(summary["network"]))
    print(json.dumps({k: v for k, v in summary["routing"].items() if k != "route_stats"}))
    print(json.dumps(summary["cost"]), json.dumps(summary["pairing"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
