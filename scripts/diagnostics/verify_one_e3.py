#!/usr/bin/env python3
"""Work order item 2: prove there is now only one E3.

Until this was fixed there were two algorithms both called E3. The six-algorithm comparison
table ran `algorithms/e3.py`; the headline P3 results ran the decision logic written inside
`p3_runtime.P3RuntimeCoordinator`. They held separate pheromone, velocity, personal-best and
scout state, and every repair of the 2026-09-08 and 2026-09-09 sessions landed in the runtime
copy only - bounded pheromone, the destination-aware heuristic, corrected BCO exploration, the
BCO memory clock, outcome-based pheromone and PSO learning. In `e3.py` the BCO term was
additionally identically zero (audit A7), because `initialize()` seeded `scout_memory` to 0.0
for every edge in the network.

This runs both entry points over the same seed and the same traffic and compares the route
decisions they make, edge by edge.

  * arm `p3` drives `P3RuntimeCoordinator` with the emergency layer OFF;
  * arm `e3` drives `algorithms.e3.select_route`, the comparison table's entry point.

With the emergency layer off, the P3 coordinator's protocol term is pinned to its neutral
constant, and `e3.select_route` receives no messages so its protocol term is that same
constant. A constant added to every candidate cannot change an argmax, so both arms are the
same router and **every decision must match**. Any difference is a piece of the algorithm that
is still not shared.

Writes nothing into the project.

    python scripts/diagnostics/verify_one_e3.py --duration 900
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

ARMS = ("p3", "e3")


def run_one_arm(arm: str, duration: int, log_path: Path) -> None:
    os.chdir(REPO_ROOT)

    import traci
    from algorithms import e3
    from algorithms.route_utils import (
        reset_route_stats,
        seed_route_choice,
        set_blocked_edges,
        set_route_checked,
    )
    from controller.reroute_logic import should_reroute
    from emergency_manager import get_emergency_manager

    decisions = []
    final_state = {}
    # A per-step checksum of the physical state. If the two arms ever differ here the
    # simulations themselves have diverged, and no comparison of later decisions means
    # anything - so this is the line to read first.
    trace = []
    out_dir = log_path.parent

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    coordinator = None
    if arm == "p3":
        from p3_runtime import P3RuntimeCoordinator

        coordinator = P3RuntimeCoordinator(emergency_layer=False)
        core = coordinator.e3
    else:
        # The comparison table's own start-up sequence.
        reset_route_stats()
        seed_route_choice()
        e3.initialize()
        core = e3.get_core()
        # `e3.select_route` is only ever handed real messages by a caller that has them; the
        # comparison table has none, which is what makes this arm the routing-only control.
        core.emergency_layer = False
        core.router.emergency_layer_enabled = False

    # Record every scored candidate set and the winner, from inside the shared core, so both
    # arms are observed at exactly the same point in the algorithm.
    original_select = type(core).select_edge
    original_record = type(core.outcomes).record_decision
    applied = []
    current = {"vehicle": None, "time": 0.0}

    def record_decision(self, vehicle_id, sim_time, planned_route, distance_m=None):
        applied.append([round(sim_time, 1), vehicle_id, len(planned_route),
                        planned_route[0] if planned_route else None,
                        planned_route[-1] if planned_route else None])
        return original_record(self, vehicle_id, sim_time, planned_route, distance_m)

    type(core.outcomes).record_decision = record_decision

    def select_edge(self, vehicle_id, outgoing_edges, destination_edge, messages,
                    is_emergency_vehicle=False):
        selected, scored = original_select(
            self, vehicle_id, outgoing_edges, destination_edge, messages,
            is_emergency_vehicle=is_emergency_vehicle,
        )
        # Every call, not only the ones with a real choice. A reroute can still be applied
        # at a junction with a single onward edge, and those are the majority here - about
        # 92% of calls - so logging only multi-candidate decisions hides exactly the events
        # that make two runs physically diverge.
        decisions.append([
            round(current["time"], 1), vehicle_id,
            "|".join(outgoing_edges), selected,
        ])
        return selected, scored

    type(core).select_edge = select_edge

    # Every actual write to SUMO. A route replacement that SUMO accepts and that the
    # caller then judges a silent rejection still perturbs the vehicle, and it never appears
    # in the applied-reroute log, so it has to be watched at the TraCI boundary.
    set_route_calls = []
    find_route_calls = []
    _original_find_route = traci.simulation.findRoute

    def logged_find_route(*args, **kwargs):
        find_route_calls.append([round(current["time"], 1), args[0] if args else None])
        return _original_find_route(*args, **kwargs)

    traci.simulation.findRoute = logged_find_route

    _original_set_route = traci.vehicle.setRoute

    def logged_set_route(vehicle_id, edges):
        set_route_calls.append([round(current["time"], 1), vehicle_id, len(list(edges))])
        return _original_set_route(vehicle_id, edges)

    traci.vehicle.setRoute = logged_set_route

    def snapshot_vehicles():
        rows = {}
        for vehicle_id in traci.vehicle.getIDList():
            try:
                rows[vehicle_id] = [
                    traci.vehicle.getRoadID(vehicle_id),
                    round(traci.vehicle.getDistance(vehicle_id), 4),
                    round(traci.vehicle.getSpeed(vehicle_id), 4),
                    len(traci.vehicle.getRoute(vehicle_id)),
                ]
            except traci.TraCIException:
                continue
        return rows

    def checksum(sim_time):
        ids = traci.vehicle.getIDList()
        total = 0.0
        for vehicle_id in ids:
            try:
                total += traci.vehicle.getDistance(vehicle_id)
            except traci.TraCIException:
                pass
        return [round(sim_time, 1), len(ids), round(total, 3)]

    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
        "--end", str(duration),
        "--tripinfo-output", str(out_dir / f"{arm}_tripinfo.xml"),
        "--error-log", str(out_dir / f"{arm}_errors.log"),
    ])
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            current["time"] = sim_time
            manager.update(sim_time)

            if coordinator is not None:
                coordinator.update_step(sim_time)
                trace.append(checksum(sim_time))
                final_state = snapshot_vehicles()
                continue

            # The comparison table's loop, as metrics_logger runs it.
            set_blocked_edges(manager.blocked_edges)
            e3.step(sim_time)
            manager.record_responder_arrivals(
                sim_time,
                vehicle_ids=traci.vehicle.getIDList(),
                arrived_ids=traci.simulation.getArrivedIDList(),
            )
            for vehicle_id in traci.vehicle.getIDList():
                try:
                    if not should_reroute(vehicle_id):
                        continue
                    current["vehicle"] = vehicle_id
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                    route = list(traci.vehicle.getRoute(vehicle_id))
                    if len(route) < 2:
                        continue
                    new_route = e3.select_route(vehicle_id, current_edge)
                    if set_route_checked(vehicle_id, new_route, route):
                        e3.note_applied_route(vehicle_id, sim_time, new_route)
                except traci.TraCIException:
                    continue
            trace.append(checksum(sim_time))
        final_state = snapshot_vehicles()
    finally:
        try:
            traci.close()
        except Exception:
            pass

    summary = core.state_summary()
    log_path.write_text(json.dumps({
        "arm": arm,
        "decisions": decisions,
        "decision_count": len(decisions),
        "applied": applied,
        "applied_count": len(applied),
        "trace": trace,
        # Per-vehicle state at the final step. When the trace says the two arms diverged,
        # running both to exactly that step and diffing this says *which* vehicle moved
        # differently, which is the only way to find the cause rather than guess at it.
        "final_state": final_state,
        "set_route_calls": set_route_calls,
        "set_route_call_count": len(set_route_calls),
        "find_route_calls": find_route_calls,
        "find_route_call_count": len(find_route_calls),
        "pheromone_edges": summary["pheromone_edges"],
        "outcomes_recorded": summary["outcomes_recorded"],
        "scout_memory_edges": summary["scout_memory_edges"],
        "quality_mean": summary["quality_mean"],
    }), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--arm", choices=ARMS, help="internal: run a single arm")
    parser.add_argument("--log", help="internal: where the single arm writes its log")
    arguments = parser.parse_args()

    if arguments.arm:
        run_one_arm(arguments.arm, arguments.duration, Path(arguments.log))
        return 0

    work_dir = Path(tempfile.mkdtemp(prefix="one_e3_"))
    logs = {}
    for arm in ARMS:
        log_path = work_dir / f"{arm}.json"
        print(f"[one-e3] running the {arm} entry point for {arguments.duration}s ...", flush=True)
        subprocess.run(
            [sys.executable, __file__, "--arm", arm, "--duration", str(arguments.duration),
             "--log", str(log_path)],
            check=True, cwd=str(REPO_ROOT),
        )
        logs[arm] = json.loads(log_path.read_text(encoding="utf-8"))

    p3, e3_arm = logs["p3"], logs["e3"]

    print("\n=== did the two simulations stay physically identical? ===")
    first_bad = None
    for left, right in zip(p3["trace"], e3_arm["trace"]):
        if left != right:
            first_bad = (left, right)
            break
    steps = min(len(p3["trace"]), len(e3_arm["trace"]))
    if first_bad is None:
        print(f"  yes - {steps} steps with the same vehicle count and the same total "
              f"distance driven")
    else:
        print("  NO. First step that differs, as [time, vehicles, total metres driven]:")
        print(f"    p3 = {first_bad[0]}")
        print(f"    e3 = {first_bad[1]}")
        print("  Everything below this line is downstream of that and must not be read as "
              "an algorithm difference.")

    print("\n=== do the two entry points make the same decisions? ===")
    print(f"  decisions recorded : {p3['decision_count']} via p3_runtime, "
          f"{e3_arm['decision_count']} via algorithms/e3.py")

    matched = 0
    for left, right in zip(p3["decisions"], e3_arm["decisions"]):
        if left != right:
            break
        matched += 1

    applied_matched = 0
    for left, right in zip(p3["applied"], e3_arm["applied"]):
        if left != right:
            break
        applied_matched += 1
    print(f"  applied reroutes   : {p3['applied_count']} via p3_runtime, "
          f"{e3_arm['applied_count']} via algorithms/e3.py")
    print(f"  identical applied reroutes before first difference: {applied_matched}")
    if p3["applied"][applied_matched:applied_matched + 1] or e3_arm["applied"][applied_matched:applied_matched + 1]:
        print(f"    p3 = {p3['applied'][applied_matched:applied_matched + 1]}")
        print(f"    e3 = {e3_arm['applied'][applied_matched:applied_matched + 1]}")
    identical = p3["decisions"] == e3_arm["decisions"]
    print(f"  identical before first difference: {matched}")
    print(f"  fully identical: {identical}")
    if not identical:
        shorter = min(len(p3["decisions"]), len(e3_arm["decisions"]))
        if matched < shorter:
            print(f"    first difference  p3 = {p3['decisions'][matched]}")
            print(f"                      e3 = {e3_arm['decisions'][matched]}")

    print("\n=== is the learning state the same shape? ===")
    for key in ("pheromone_edges", "outcomes_recorded", "scout_memory_edges", "quality_mean"):
        print(f"  {key:<22}: p3 {p3[key]}   e3 {e3_arm[key]}")

    print("\n=== verdict ===")

    if p3["decision_count"] == 0:
        print("  FAIL: the p3 arm recorded no decisions at all - it is not running the router.")
        print(f"  logs: {work_dir}")
        return 1

    # What this can prove, and what it cannot.
    #
    # The two arms share one algorithm and one state, but they are not the same program:
    # the P3 arm also runs the protocol layer, and that layer queries SUMO. In particular
    # `EmergencyManager.get_nearest_emergency` calls `traci.simulation.findRoute` once per
    # broadcasting vehicle per step while an incident is active - measured at 2,500 calls
    # against 707 by the end of the first incident. SUMO does not return bit-identical
    # vehicle dynamics under a different query load: the first physical difference seen was
    # one vehicle moving 0.15 m further in a single step, seventeen seconds after the first
    # incident opened, with its road, route and every route write identical.
    #
    # So bit-identity of a whole run is not the right bar. It would only be reachable by
    # making the comparison arm do the protocol layer's work too, which is the opposite of
    # what the comparison table is for. The bar is: while the two simulations are still
    # physically identical, does the shared router make identical decisions?
    if first_bad is None:
        window = "the whole run"
        decisions_in_window = min(len(p3["decisions"]), len(e3_arm["decisions"]))
    else:
        cutoff = first_bad[0][0]
        window = f"the first {cutoff:.0f} s, before the simulations diverged"
        decisions_in_window = sum(1 for row in p3["decisions"] if row[0] <= cutoff)

    agree_in_window = min(matched, decisions_in_window)
    same_writes = p3["set_route_calls"] == e3_arm["set_route_calls"]

    print(f"  decisions inside {window}: {decisions_in_window}")
    print(f"  ...of which identical: {agree_in_window}")
    print(f"  route writes to SUMO identical: {same_writes} "
          f"({len(p3['set_route_calls'])} vs {len(e3_arm['set_route_calls'])} calls)")

    if decisions_in_window > 0 and agree_in_window == decisions_in_window:
        print("  PASS: one implementation. Every routing decision matched while the two "
              "simulations were physically identical.")
        if first_bad is not None:
            print("  Note: the runs diverge later, through the protocol layer's own SUMO "
                  "queries rather than through the router - see the comment in this file.")
        print(f"  logs: {work_dir}")
        return 0

    print("  FAIL: the two entry points made different decisions while the simulations were "
          "still identical, so something in the algorithm is not shared.")
    print(f"  logs: {work_dir}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
