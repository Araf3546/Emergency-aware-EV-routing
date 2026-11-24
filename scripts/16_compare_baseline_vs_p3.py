#!/usr/bin/env python3
"""Focused comparison: true baseline against the P3/E3-enabled scenario.

**Baseline** is plain SUMO with the same 4,000-vehicle demand, the same emergency scenario
and the same seed, with the P3 layer disabled. Incidents still close roads and responder
arrivals are still measured, because those are the scenario and the measurement, not the
treatment. No rerouting, no broadcasting, no emergency scoring and no yielding happen.

**P3-enabled** is the identical environment with `P3RuntimeCoordinator` driving V2V
broadcasts, E3 hybrid rerouting and the yielding rule.

Reports response times for each incident and the traffic conditions on the roads the
responders actually used, so the two arms can be compared on equal terms.

Usage:
    python scripts/16_compare_baseline_vs_p3.py
    python scripts/16_compare_baseline_vs_p3.py --duration 1300 --arm baseline
    python scripts/16_compare_baseline_vs_p3.py --seed 20260723      # one evaluation seed

`--seed` selects an evaluation seed (contract 13.15): SUMO's seed, that seed's frozen demand
file and the route-choice seed together, via evaluation_seeds.py. Its results go to
results/baseline_vs_p3/seed_<seed>/ so seeds never overwrite each other. Without it the run is
exactly as before, under simulation.sumocfg.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import traci  # noqa: E402
import yaml  # noqa: E402

from emergency_manager import get_emergency_manager  # noqa: E402
from evaluation_seeds import route_choice_seed, sumo_seed_args  # noqa: E402
from metrics.trip_metrics import build_report, load_tripinfo  # noqa: E402
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES  # noqa: E402

OUTPUT_DIR = REPO_ROOT / "results" / "baseline_vs_p3"
SCENARIO_CONFIG = REPO_ROOT / "config" / "scenario.yaml"

# The roads the dispatched responders travel. Read from demand/porto.dispatch.rou.xml
# rather than copied into this file, because a hardcoded copy silently went stale when the
# incident set changed to v3 - it still named events 402 and 403, which no longer exist.
#
# Work order item 4: this used to list 7, 2 and 2 edges per responder. Two sampled edges
# cannot explain a responder's journey - event 403's endpoints held 0.03-0.05 vehicles in
# *both* arms while its response time differed by 363 s, which said only that the two edges
# sampled were not where the difference happened. Every edge of every responder route is
# now sampled.
DISPATCH_ROUTES = REPO_ROOT / "demand" / "porto.dispatch.rou.xml"


def responder_route_edges() -> dict:
    """{event_id: [edges in route order]} for every dispatched responder."""
    import xml.etree.ElementTree as ET

    routes = {}
    for vehicle in ET.parse(DISPATCH_ROUTES).getroot().iter("vehicle"):
        vehicle_id = vehicle.get("id")
        if not vehicle_id.startswith("emergency_dispatch_"):
            continue
        routes[int(vehicle_id.rsplit("_", 1)[1])] = vehicle.find("route").get("edges").split()
    return routes


RESPONDER_ROUTE_EDGES = responder_route_edges()

def contracted_duration() -> int:
    with open(SCENARIO_CONFIG, "r", encoding="utf-8") as handle:
        return int(yaml.safe_load(handle)["simulation"]["duration_seconds"])


def output_dir_for(seed: int | None) -> Path:
    return OUTPUT_DIR if seed is None else OUTPUT_DIR / f"seed_{seed}"


def run_arm(arm: str, duration: int, sumo_cfg: str, emergency_layer: bool = True,
            seed: int | None = None) -> dict:
    """Run one arm and collect response times plus traffic on the responder routes.

    `emergency_layer=False` is the control arm of the matched ablation: E3 routes normal
    traffic exactly as in the treatment arm, but the protocol term is pinned to its neutral
    constant, nothing yields, and a responder claims no corridor. Comparing it against the
    treatment arm separates the emergency layer's cost from the router's.

    `seed` is an evaluation seed (contract 13.15); None runs simulation.sumocfg as it is.
    """
    run_dir = output_dir_for(seed) / arm
    run_dir.mkdir(parents=True, exist_ok=True)
    # Resolved before anything starts, so a missing or edited frozen file stops the run here.
    seed_args = sumo_seed_args(seed, Path(sumo_cfg))

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    coordinator = None
    # Any P3 arm runs the coordinator - the treatment arm "p3" and the ablation control
    # "p3_layer_off" alike. Testing `arm == "p3"` here silently turned the control arm into
    # a second baseline: no coordinator, no rerouting, no broadcasts, and results identical
    # to baseline down to the last decimal, which looks like a finding rather than a bug.
    if arm.startswith("p3"):
        from p3_runtime import P3RuntimeCoordinator

        coordinator = P3RuntimeCoordinator(
            emergency_layer=emergency_layer, route_choice_seed=route_choice_seed(seed)
        )

    traci.start([
        "sumo", "-c", sumo_cfg,
        "--no-step-log", "true",
        "--end", str(duration),
        "--tripinfo-output", str(run_dir / "tripinfo.xml"),
        "--summary-output", str(run_dir / "summary.xml"),
        "--error-log", str(run_dir / "sumo_errors.log"),
        *seed_args,
    ])

    watched = sorted({edge for edges in RESPONDER_ROUTE_EDGES.values() for edge in edges})
    occupancy = {edge: {"vehicle_steps": 0, "halting_steps": 0, "samples": 0} for edge in watched}

    # Work order item 4. Averaging occupancy over the whole hour answers the wrong
    # question: event 403's responder is on the road for a few minutes near t=2760, and an
    # average over 3,600 s buries whatever it met there. These accumulators run only while
    # a responder is actually travelling to its own incident, so they describe the traffic
    # that responder faced rather than the traffic on those roads in general.
    window_occupancy = {
        event_id: {edge: {"vehicle_steps": 0, "halting_steps": 0, "samples": 0} for edge in edges}
        for event_id, edges in RESPONDER_ROUTE_EDGES.items()
    }

    started = time.time()
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)

            if coordinator is not None:
                coordinator.update_step(sim_time)
            else:
                # Baseline still measures responder arrivals - that is the metric, shared by
                # both arms, and it changes no vehicle behaviour.
                manager.record_responder_arrivals(
                    sim_time,
                    vehicle_ids=traci.vehicle.getIDList(),
                    arrived_ids=traci.simulation.getArrivedIDList(),
                )

            for edge in watched:
                try:
                    occupancy[edge]["vehicle_steps"] += traci.edge.getLastStepVehicleNumber(edge)
                    occupancy[edge]["halting_steps"] += traci.edge.getLastStepHaltingNumber(edge)
                    occupancy[edge]["samples"] += 1
                except traci.TraCIException:
                    continue

            for event_id, edges in RESPONDER_ROUTE_EDGES.items():
                event = manager.events.get(event_id)
                if event is None or sim_time < event.start_time:
                    continue
                if event.first_arrival_time is not None:
                    continue  # this responder has arrived; its window is closed
                for edge in edges:
                    try:
                        counts = window_occupancy[event_id][edge]
                        counts["vehicle_steps"] += traci.edge.getLastStepVehicleNumber(edge)
                        counts["halting_steps"] += traci.edge.getLastStepHaltingNumber(edge)
                        counts["samples"] += 1
                    except traci.TraCIException:
                        continue
    finally:
        try:
            traci.close()
        except Exception:
            pass

    responses = manager.get_response_summary()
    trips = load_tripinfo(run_dir / "tripinfo.xml")

    responder_trips = {}
    for event_id, state in responses.items():
        responder = state.get("first_arrival_vehicle")
        record = trips.get(responder) if responder else None
        responder_trips[str(event_id)] = {
            "response_time_s": state.get("response_time_s"),
            "responder": responder,
            # Audit M12: a responder that SUMO teleported can be dropped on the incident's
            # own centre edge, which registers as an arrival on a trip that was never
            # driven. Such an event reports response_time_s = None and is shown here as
            # invalid, with the number that was observed kept for inspection.
            "response_valid": state.get("response_valid"),
            "responder_teleported": state.get("responder_teleported"),
            "responder_teleport_times_s": state.get("responder_teleport_times_s"),
            "invalid_reason": state.get("invalid_reason"),
            "unvalidated_response_time_s": state.get("unvalidated_response_time_s"),
            "arrived_after_incident_end": state.get("arrived_after_incident_end"),
            "responder_time_loss_s": round(float(record["timeLoss"]), 2) if record else None,
            "responder_waiting_time_s": round(float(record["waitingTime"]), 2) if record else None,
            "responder_duration_s": round(float(record["duration"]), 2) if record else None,
        }

    route_traffic = {}
    for edge, counts in occupancy.items():
        samples = counts["samples"] or 1
        route_traffic[edge] = {
            "mean_vehicles": round(counts["vehicle_steps"] / samples, 2),
            "mean_halting": round(counts["halting_steps"] / samples, 2),
        }

    responder_window_traffic = {}
    for event_id, edges in RESPONDER_ROUTE_EDGES.items():
        event = manager.events.get(event_id)
        per_edge = {}
        for edge in edges:
            counts = window_occupancy[event_id][edge]
            samples = counts["samples"] or 1
            per_edge[edge] = {
                "mean_vehicles": round(counts["vehicle_steps"] / samples, 3),
                "mean_halting": round(counts["halting_steps"] / samples, 3),
                "samples": counts["samples"],
            }
        responder_window_traffic[str(event_id)] = {
            "window_from_s": event.start_time if event else None,
            "window_to_s": event.first_arrival_time if event else None,
            "route_edges_in_order": edges,
            "per_edge": per_edge,
        }

    teleports = manager.get_teleport_summary()

    # A P3 arm that made no routing decision did not run its coordinator. That happened
    # once already, when the control arm was renamed and the coordinator check still tested
    # for the exact name "p3": the arm silently became a second baseline and its output was
    # identical to baseline on every metric, which reads as a finding rather than a bug.
    # Fail loudly instead of publishing it.
    if coordinator is not None and coordinator.metrics.reroute_decisions == 0:
        raise RuntimeError(
            f"arm '{arm}' ran the coordinator but recorded 0 reroute decisions over "
            f"{duration}s. The coordinator did nothing - this run is a baseline in disguise "
            f"and must not be reported."
        )

    return {
        "arm": arm,
        "emergency_layer": "on" if emergency_layer else "off",
        # None means simulation.sumocfg's own seed and demand file (20260722).
        "evaluation_seed": seed,
        "duration_seconds": duration,
        "wall_clock_minutes": round((time.time() - started) / 60, 1),
        "responses": responder_trips,
        "responder_window_traffic": responder_window_traffic,
        "responder_teleport_count": teleports["responder_teleport_count"],
        "responder_teleports": teleports["responder_teleports"],
        "events_invalidated_by_teleport": teleports["events_invalidated_by_teleport"],
        "route_traffic": route_traffic,
        "trip_metrics": build_report(
            run_dir / "tripinfo.xml",
            run_dir / "summary.xml",
            emergency_types=EMERGENCY_VEHICLE_TYPES,
        ),
        "yield_actions_performed": (
            coordinator.metrics.yield_actions_performed if coordinator else 0
        ),
        "yield_requests_broadcast": (
            coordinator.metrics.yield_requests if coordinator else 0
        ),
        # Reroute counts were missing from this report entirely, so a run could be compared
        # on response time without anyone being able to see how much rerouting produced it.
        # `reroute_decisions` counts every time the router was asked; `reroute_executed`
        # counts the ones SUMO accepted and applied.
        "reroute_decisions": (coordinator.metrics.reroute_decisions if coordinator else 0),
        "reroute_executed": (coordinator.metrics.reroute_executed if coordinator else 0),
        "emergency_reroutes": (coordinator.metrics.emergency_reroutes if coordinator else 0),
        "vehicles_rerouted": (len(coordinator.metrics.vehicles_rerouted) if coordinator else 0),
        "candidate_edges_scored": (
            coordinator.metrics.candidate_edges_scored if coordinator else 0
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=None)
    parser.add_argument("--sumo-cfg", default="simulation.sumocfg")
    parser.add_argument("--arm", choices=("baseline", "p3"), help="run only one arm")
    parser.add_argument("--emergency-layer", choices=("on", "off"), default="on",
                        help="'off' runs the ablation control: E3 routing with the emergency "
                             "layer disabled. Saved under a separate arm name so it never "
                             "overwrites the treatment arm.")
    parser.add_argument("--seed", type=int, default=None,
                        help="evaluation seed (contract 13.15): SUMO seed, frozen demand file and "
                             "route-choice seed together; results under seed_<seed>/")
    arguments = parser.parse_args()

    duration = arguments.duration or contracted_duration()
    output_dir = output_dir_for(arguments.seed)
    output_dir.mkdir(parents=True, exist_ok=True)

    emergency_layer = arguments.emergency_layer == "on"
    arms = [arguments.arm] if arguments.arm else ["baseline", "p3"]
    results = {}
    for arm in arms:
        # The control arm gets its own name and its own files. Writing it to p3.json would
        # silently destroy the treatment arm it is meant to be compared against.
        label = arm if emergency_layer or arm == "baseline" else "p3_layer_off"
        print(f"[16] running {label} for {duration}s ...", flush=True)
        results[label] = run_arm(label, duration, arguments.sumo_cfg,
                                 emergency_layer=emergency_layer, seed=arguments.seed)
        arm = label
        path = output_dir / f"{arm}.json"
        path.write_text(json.dumps(results[arm], indent=2), encoding="utf-8")
        print(f"[16] {arm}: {results[arm]['wall_clock_minutes']} min", flush=True)

    if not emergency_layer:
        # The baseline-versus-treatment comparison is not what this run measured; writing
        # one here would pair the control arm's numbers with the wrong label.
        print("[16] control arm saved. Compare it against p3.json yourself, or re-run "
              "the treatment arm with --emergency-layer on.", flush=True)
        return 0

    # Merge with any previously saved arm so a single-arm run still produces a comparison.
    for arm in ("baseline", "p3"):
        path = output_dir / f"{arm}.json"
        if arm not in results and path.exists():
            results[arm] = json.loads(path.read_text(encoding="utf-8"))

    if len(results) == 2 and results["baseline"]["duration_seconds"] != results["p3"]["duration_seconds"]:
        # Merging a saved arm lets a short single-arm test run be compared against a full
        # run saved days earlier. That comparison is nonsense in a specific and convincing
        # way: the shorter arm simply has not reached the later incidents, so it reports
        # them as unanswered and looks catastrophically worse. Refuse rather than write it.
        print(
            f"[16] NOT writing comparison.json: baseline ran "
            f"{results['baseline']['duration_seconds']}s and p3 ran "
            f"{results['p3']['duration_seconds']}s. Re-run both arms at the same duration.",
            flush=True,
        )
    elif len(results) == 2:
        comparison = {
            "report_version": 1,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "response_times": {
                event: {
                    "baseline_s": results["baseline"]["responses"].get(event, {}).get("response_time_s"),
                    "p3_s": results["p3"]["responses"].get(event, {}).get("response_time_s"),
                    # Audit M12: null with invalid_* set means the responder teleported and
                    # the arrival was never driven. It is not a slow response, it is no
                    # response, and it must not be averaged with the others.
                    "baseline_invalid_reason": results["baseline"]["responses"].get(event, {}).get("invalid_reason"),
                    "p3_invalid_reason": results["p3"]["responses"].get(event, {}).get("invalid_reason"),
                }
                for event in sorted(results["baseline"]["responses"])
            },
            "responder_teleports": {
                arm: {
                    "count": results[arm].get("responder_teleport_count", 0),
                    "events_invalidated": results[arm].get("events_invalidated_by_teleport", []),
                    "detail": results[arm].get("responder_teleports", []),
                }
                for arm in ("baseline", "p3")
            },
            "responder_time_loss": {
                event: {
                    "baseline_s": results["baseline"]["responses"].get(event, {}).get("responder_time_loss_s"),
                    "p3_s": results["p3"]["responses"].get(event, {}).get("responder_time_loss_s"),
                }
                for event in sorted(results["baseline"]["responses"])
            },
            "route_traffic": {
                edge: {
                    "baseline": results["baseline"]["route_traffic"].get(edge),
                    "p3": results["p3"]["route_traffic"].get(edge),
                }
                for edge in sorted(results["baseline"]["route_traffic"])
            },
            "network_wide": {
                arm: {
                    key: results[arm]["trip_metrics"].get(key)
                    for key in ("completed_trips", "mean_travel_time_s", "mean_time_loss_s",
                                "mean_waiting_time_s", "teleports_total")
                }
                for arm in ("baseline", "p3")
            },
            "yield_actions_performed": results["p3"]["yield_actions_performed"],
            "yield_requests_broadcast": results["p3"]["yield_requests_broadcast"],
            "rerouting": {
                key: results["p3"].get(key)
                for key in ("reroute_decisions", "reroute_executed", "emergency_reroutes",
                            "vehicles_rerouted", "candidate_edges_scored")
            },
        }
        (output_dir / "comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
        print(json.dumps(comparison["response_times"], indent=2))
        print(f"[16] comparison written to {output_dir / 'comparison.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
