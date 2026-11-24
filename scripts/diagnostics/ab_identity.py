#!/usr/bin/env python3
"""Record everything a run does, so two builds can be compared for behavioural identity.

Written for A/B checks where a change must provably alter nothing: it captures, per step, the
vehicle count and total metres driven, a hash of Python's random state (E3's stream), and each
vehicle's SUMO speed factor when first seen; plus every routing decision, every route write,
every incident-distance query with its answer, and the run counters and response times.

Used on 2026-09-11 to show the incident-distance fix changed nothing: old and fixed code were
bit-for-bit identical over a full hour with the emergency layer both OFF and ON. It is the
tool for validating the incident-distance cache the same way.

Run the same arm against two trees (or one tree with `--distance-cache on` and `off`) and
compare the two output directories with `ab_compare.py`: every recorded field must match, and
tripinfo.xml must match record for record.

`--seed` runs an evaluation seed (contract 13.15, evaluation_seeds.py): SUMO seed, that seed's
frozen demand file and the route-choice seed together. Without it: simulation.sumocfg as is.

2026-09-12, for the incident-distance cache: speed factors are recorded at full precision (so
they can be compared exactly with a frozen-demand manifest), the random-state hash covers the
whole state, route writes carry a hash of their edges, every incident-distance answer carries
the set of incidents open when it was asked, and every findRoute call is tagged with whether
the incident-distance code made it - so the comparison can prove the cache asks SUMO exactly
once per key and otherwise leaves every query alone.

Arms: p3 or p3off (emergency layer OFF), p3on (ON), e3 (comparison-table path), e3_replay,
e3_typed, e3_untyped (see route_query_drift.py for what the replay arms are for).

Read-only: changes nothing in the project. Runs one arm per process so no module state is
shared, and every arm writes its own SUMO outputs so parallel arms cannot collide.

Arms
----
p3            P3RuntimeCoordinator, emergency layer OFF. Also logs every findRoute call made
              by EmergencyManager._route_distance, with full arguments and step, so it can
              be replayed.
e3            The comparison table's loop through algorithms.e3.
e3_replay     The e3 loop, plus - at the point in each step where the P3 coordinator
              broadcasts - the exact findRoute calls the p3 arm's broadcasts made, results
              thrown away. If this arm matches p3 physically, those calls are the complete
              explanation of the divergence. If it does not, something else is involved.
e3_typed      The e3 loop plus ONE extra findRoute(..., "normal_passenger") at t=300.
e3_untyped    The e3 loop plus ONE extra findRoute(..., "") at t=300.

Recorded per step: vehicle count and total metres driven (physical checksum), a hash of
Python's `random` state (the E3 router's random stream), and the SUMO speed factor of every
vehicle the first time it is seen - which exposes SUMO's own random stream, because
`normal_passenger` sets no speedFactor and SUMO therefore draws one for each vehicle.

    python scripts/diagnostics/ab_identity.py --arm p3on --duration 3600 --out DIR
    python scripts/diagnostics/ab_identity.py --arm p3on --seed 20260723 --distance-cache off --out DIR
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import sys
import time
from pathlib import Path

REPO_ROOT = Path(os.environ.get("PROBE_ROOT", Path(__file__).resolve().parent.parent.parent))
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402

PROBE_TIME = 300.0
PROBE_FROM = None   # filled from the first edge of the first vehicle seen


def rng_hash():
    return hashlib.md5(repr(random.getstate()).encode()).hexdigest()[:12]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", required=True)
    parser.add_argument("--duration", type=int, default=700)
    parser.add_argument("--out", required=True)
    parser.add_argument("--replay", default=None)
    parser.add_argument("--seed", type=int, default=None,
                        help="evaluation seed (contract 13.15); default: simulation.sumocfg as is")
    parser.add_argument("--distance-cache", choices=("on", "off"), default=None,
                        help="force EmergencyManager's incident-distance cache on or off; "
                             "default: whatever this build does")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    from algorithms import e3
    from algorithms.route_utils import (
        get_route_stats, reset_route_stats, seed_route_choice, set_blocked_edges, set_route_checked,
    )
    from controller.reroute_logic import should_reroute
    import emergency_manager as em_module
    from emergency_manager import get_emergency_manager
    from evaluation_seeds import route_choice_seed, sumo_seed_args

    seed_args = sumo_seed_args(args.seed)

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    if args.distance_cache is not None:
        manager.cache_incident_distances = args.distance_cache == "on"

    # ------------------------------------------------------------------ logging hooks
    step_ref = {"t": 0.0}
    route_distance_calls = []          # every incident-distance answer, cached or not
    find_route_calls = []              # every findRoute, any caller, tagged by caller
    set_route_calls = []
    inside_distance = {"depth": 0, "seconds": 0.0}

    original_find = traci.simulation.findRoute

    def logged_find(*a, **k):
        source = "distance" if inside_distance["depth"] else "other"
        find_route_calls.append([step_ref["t"], list(map(str, a[:3])), source])
        return original_find(*a, **k)

    traci.simulation.findRoute = logged_find

    original_set = traci.vehicle.setRoute

    def logged_set(vehicle_id, edges):
        edges = list(edges)
        set_route_calls.append([step_ref["t"], vehicle_id, len(edges),
                                hashlib.md5("|".join(edges).encode()).hexdigest()[:12]])
        return original_set(vehicle_id, edges)

    traci.vehicle.setRoute = logged_set

    original_rd = em_module.EmergencyManager._route_distance

    def logged_rd(self, from_edge, to_edge, vehicle_type=""):
        inside_distance["depth"] += 1
        started = time.perf_counter()
        try:
            answer = original_rd(self, from_edge, to_edge, vehicle_type)
        finally:
            inside_distance["seconds"] += time.perf_counter() - started
            inside_distance["depth"] -= 1
        route_distance_calls.append([step_ref["t"], from_edge, to_edge, vehicle_type,
                                     sorted(map(str, self.active_events)),
                                     None if answer == float("inf") else answer])
        return answer

    em_module.EmergencyManager._route_distance = logged_rd

    replay = {}
    if args.replay:
        for t, frm, to, vt, *_ in json.loads(Path(args.replay).read_text())["route_distance_calls"]:
            replay.setdefault(t, []).append((frm, to, vt))

    # ------------------------------------------------------------------ arm set-up
    coordinator = None
    if args.arm in ("p3", "p3off", "p3on"):
        from p3_runtime import P3RuntimeCoordinator
        coordinator = P3RuntimeCoordinator(emergency_layer=(args.arm == "p3on"),
                                           route_choice_seed=route_choice_seed(args.seed))
        core = coordinator.e3
    else:
        reset_route_stats()
        seed_route_choice(route_choice_seed(args.seed))
        e3.initialize()
        core = e3.get_core()
        core.emergency_layer = False
        core.router.emergency_layer_enabled = False

    decisions = []
    original_select = type(core).select_edge

    def select_edge(self, vehicle_id, outgoing, destination, messages, is_emergency_vehicle=False):
        chosen, n = original_select(self, vehicle_id, outgoing, destination, messages,
                                    is_emergency_vehicle=is_emergency_vehicle)
        decisions.append([step_ref["t"], vehicle_id, "|".join(outgoing), chosen])
        return chosen, n

    type(core).select_edge = select_edge

    trace = []
    speed_factors = {}
    first_edge = {}

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
    wall_started = time.perf_counter()
    try:
        while traci.simulation.getTime() < args.duration:
            traci.simulationStep()
            t = traci.simulation.getTime()
            step_ref["t"] = t
            manager.update(t)

            if coordinator is not None:
                coordinator.update_step(t)
            else:
                set_blocked_edges(manager.blocked_edges)
                e3.step(t)
                manager.record_responder_arrivals(
                    t, vehicle_ids=traci.vehicle.getIDList(),
                    arrived_ids=traci.simulation.getArrivedIDList(),
                )
                # Where the P3 coordinator broadcasts: before its routing loop.
                for frm, to, vt in replay.get(t, []):
                    try:
                        original_find(frm, to, vt)
                    except traci.TraCIException:
                        pass
                if args.arm in ("e3_typed", "e3_untyped") and t == PROBE_TIME:
                    ids = traci.vehicle.getIDList()
                    frm = traci.vehicle.getRoadID(ids[0])
                    to = traci.vehicle.getRoute(ids[0])[-1]
                    vt = "normal_passenger" if args.arm == "e3_typed" else ""
                    try:
                        original_find(frm, to, vt)
                    except traci.TraCIException:
                        pass
                for vehicle_id in traci.vehicle.getIDList():
                    try:
                        if not should_reroute(vehicle_id):
                            continue
                        edge = traci.vehicle.getRoadID(vehicle_id)
                        route = list(traci.vehicle.getRoute(vehicle_id))
                        if len(route) < 2:
                            continue
                        new_route = e3.select_route(vehicle_id, edge)
                        if set_route_checked(vehicle_id, new_route, route):
                            e3.note_applied_route(vehicle_id, t, new_route)
                    except traci.TraCIException:
                        continue

            ids = traci.vehicle.getIDList()
            total = 0.0
            for vehicle_id in ids:
                try:
                    total += traci.vehicle.getDistance(vehicle_id)
                    if vehicle_id not in speed_factors:
                        # Full precision, so it can be compared exactly with a manifest.
                        speed_factors[vehicle_id] = [t, traci.vehicle.getSpeedFactor(vehicle_id)]
                except traci.TraCIException:
                    pass
            trace.append([t, len(ids), round(total, 4), rng_hash()])
    finally:
        try:
            traci.close()
        except Exception:
            pass
    wall_seconds = time.perf_counter() - wall_started

    counters = None
    if coordinator is not None:
        metrics = coordinator.metrics
        counters = {
            "reroute_decisions": metrics.reroute_decisions,
            "reroute_executed": metrics.reroute_executed,
            "emergency_reroutes": metrics.emergency_reroutes,
            "vehicles_rerouted": len(metrics.vehicles_rerouted),
            "candidate_edges_scored": metrics.candidate_edges_scored,
            "yield_requests": metrics.yield_requests,
            "yield_actions": metrics.yield_actions_performed,
            "coordinator_yield_actions": coordinator.yield_actions,
            "route_stats": get_route_stats(),
            "responses": {str(k): v.get("response_time_s") for k, v in manager.get_response_summary().items()},
            "response_summary": {str(k): v for k, v in manager.get_response_summary().items()},
            "teleports": manager.get_teleport_summary(),
            "incident_errors": list(manager.incident_errors),
        }

    distance_find_route = sum(1 for call in find_route_calls if call[2] == "distance")
    (out / "result.json").write_text(json.dumps({
        "arm": args.arm,
        "evaluation_seed": args.seed,
        "distance_cache": getattr(manager, "cache_incident_distances", "not in this build"),
        "trace": trace,
        "decisions": decisions,
        "speed_factors": speed_factors,
        "route_distance_calls": route_distance_calls,
        "find_route_calls": find_route_calls,
        "set_route_calls": set_route_calls,
        "counters": counters,
        # Timing is information, not part of the identity check.
        "timing": {
            "wall_seconds": round(wall_seconds, 1),
            "incident_distance_seconds": round(inside_distance["seconds"], 1),
            "incident_distance_answers": len(route_distance_calls),
            "incident_distance_find_route_calls": distance_find_route,
        },
    }), encoding="utf-8")
    print(f"[{args.arm}] done: {len(trace)} steps, {len(decisions)} decisions, "
          f"{len(find_route_calls)} findRoute ({distance_find_route} for incident distances), "
          f"{len(route_distance_calls)} incident-distance answers, {wall_seconds:.0f} s wall")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
