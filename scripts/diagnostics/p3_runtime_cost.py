#!/usr/bin/env python3
"""Where does a P3 run spend its time, and can the incident-distance answers be reused?

Wall-clock shares within one run are meaningful; totals between runs on different days are not
- on 2026-09-11 every component of one run was 40% faster than another purely because the
machine was less loaded. Run it alone.

Measured shares of a treatment run (layer ON, 3,600 s): incident-distance findRoute 14.1%,
get_nearest_emergency in total 15.7%, broadcasting apart from it about 40%, the vehicle-metrics
recorder about 20%, SUMO's own step about 5%, E3 routing decisions about 3%.

queries.json is the evidence for the cache: 0 of 8,145 distinct keys (from edge, to edge,
vehicle type, active incidents) ever returned two different answers over 129,309 queries, and
93.7% of queries would be answered from memory.

Read-only. One full treatment-arm run (emergency layer ON) with timers wrapped around the
pieces of each step, plus a record of every route-distance query the broadcasts make.

The record is what decides whether a cache would be exact. For every query it keeps the
arguments, the answer, and the incident state it was asked under (which lanes are closed and
slowed). If the same arguments under the same incident state ever produced two different
answers, SUMO's answer depends on something else - live travel times, for instance - and no
cache keyed on those arguments could be behaviour-preserving.

Run it alone. Wall-clock shares are only meaningful without other SUMO processes competing
for the CPU.

    python scripts/diagnostics/p3_runtime_cost.py --duration 3600 --out DIR
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(os.environ.get("PROBE_ROOT", Path(__file__).resolve().parent.parent.parent))
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402

timers = defaultdict(float)
counts = defaultdict(int)


def timed(name, fn):
    def wrapper(*a, **k):
        start = time.perf_counter()
        try:
            return fn(*a, **k)
        finally:
            timers[name] += time.perf_counter() - start
            counts[name] += 1
    return wrapper


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=3600)
    parser.add_argument("--out", required=True)
    parser.add_argument("--layer", choices=("on", "off"), default="on")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    import emergency_manager as em
    from emergency_manager import get_emergency_manager
    import p3_runtime

    manager = get_emergency_manager()
    queries = []

    original_rd = em.EmergencyManager._route_distance

    def recorded_rd(self, from_edge, to_edge, vehicle_type=""):
        start = time.perf_counter()
        result = original_rd(self, from_edge, to_edge, vehicle_type)
        timers["route_distance(findRoute)"] += time.perf_counter() - start
        counts["route_distance(findRoute)"] += 1
        queries.append([
            state["t"], from_edge, to_edge, vehicle_type,
            None if result == float("inf") else round(result, 6),
            state["incident_key"],
        ])
        return result

    em.EmergencyManager._route_distance = recorded_rd
    em.EmergencyManager.get_nearest_emergency = timed(
        "get_nearest_emergency (total)", em.EmergencyManager.get_nearest_emergency)

    C = p3_runtime.P3RuntimeCoordinator
    C.update_step = timed("update_step (all P3 work)", C.update_step)
    C._broadcast_vehicle_messages = timed("  broadcast", C._broadcast_vehicle_messages)
    C._apply_hybrid_decision = timed("  routing decisions", C._apply_hybrid_decision)
    C._apply_yield_behaviour = timed("  yielding", C._apply_yield_behaviour)
    C._record_vehicle_metrics = timed("  vehicle metrics", C._record_vehicle_metrics)
    C._sample_emergency_zone_blockage = timed("  blockage sampling", C._sample_emergency_zone_blockage)

    state = {"t": 0.0, "incident_key": ""}

    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = C(emergency_layer=(args.layer == "on"))

    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
        "--end", str(args.duration),
        "--tripinfo-output", str(out / "tripinfo.xml"),
        "--summary-output", str(out / "summary.xml"),
        "--emission-output", str(out / "emissions.xml"),
        "--queue-output", str(out / "queue.xml"),
        "--message-log", str(out / "messages.log"),
        "--error-log", str(out / "errors.log"),
    ])
    wall_start = time.perf_counter()
    try:
        while traci.simulation.getTime() < args.duration:
            s = time.perf_counter()
            traci.simulationStep()
            timers["SUMO simulationStep"] += time.perf_counter() - s
            t = traci.simulation.getTime()
            state["t"] = t
            s = time.perf_counter()
            manager.update(t)
            timers["emergency_manager.update"] += time.perf_counter() - s
            # The incident state a route query is answered under: which events are active,
            # hence which lanes are closed and slowed.
            state["incident_key"] = ",".join(str(e) for e in sorted(manager.active_events))
            coordinator.update_step(t)
    finally:
        wall = time.perf_counter() - wall_start
        try:
            traci.close()
        except Exception:
            pass

    report = {
        "duration": args.duration,
        "layer": args.layer,
        "wall_seconds": round(wall, 2),
        "timers": {k: round(v, 3) for k, v in timers.items()},
        "counts": dict(counts),
        "reroute_decisions": coordinator.metrics.reroute_decisions,
        "yield_actions": coordinator.metrics.yield_actions_performed,
        "broadcasts": getattr(coordinator.metrics, "broadcasts_sent", None),
    }
    (out / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (out / "queries.json").write_text(json.dumps(queries), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
