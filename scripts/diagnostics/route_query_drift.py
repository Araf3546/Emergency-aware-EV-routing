#!/usr/bin/env python3
"""Why do the two E3 entry points drift apart physically? Proves the cause by replay.

Background (2026-09-11). After the two E3 implementations were unified, both entry points
made identical routing decisions but their simulations drifted apart about ten minutes in.
This finds the first point of divergence and tests the cause directly instead of guessing.

What it established: SUMO draws every ordinary vehicle's speed factor from one random
stream when it loads the vehicle, and every `traci.simulation.findRoute` call consumes a
draw from that same stream. The P3 coordinator's broadcasts make about 10,600 extra route
queries in the first 700 s (EmergencyManager.get_nearest_emergency), so every vehicle
loaded afterwards gets a different speed factor. Replaying exactly those queries in the
comparison-table path makes the two paths bit-for-bit identical.

Since contract v0.7 (decision 13.13) the speed factors are frozen in the demand file, and this
diagnostic is the guard for that: it now PASSES only if the two entry points are identical with
no replay at all, and an extra route query changes nothing. Measured after the freeze: identical
for 700 s while one path made 11,293 route queries and the other 942.

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

    python scripts/diagnostics/route_query_drift.py --duration 700

runs every arm as its own process and prints the comparison; --arm runs one arm only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import sys
from pathlib import Path

REPO_ROOT = Path(os.environ.get("PROBE_ROOT", Path(__file__).resolve().parent.parent.parent))
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402

PROBE_TIME = 300.0
PROBE_FROM = None   # filled from the first edge of the first vehicle seen


def rng_hash():
    return hashlib.md5(repr(random.getstate()[1][:8] + random.getstate()[1][-8:]).encode()).hexdigest()[:10]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default=None)
    parser.add_argument("--duration", type=int, default=700)
    parser.add_argument("--out", default=None)
    parser.add_argument("--replay", default=None)
    args = parser.parse_args()

    if args.arm is None:
        return drive(args.duration)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    from algorithms import e3
    from algorithms.route_utils import (
        reset_route_stats, seed_route_choice, set_blocked_edges, set_route_checked,
    )
    from controller.reroute_logic import should_reroute
    import emergency_manager as em_module
    from emergency_manager import get_emergency_manager

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    # ------------------------------------------------------------------ logging hooks
    step_ref = {"t": 0.0}
    route_distance_calls = []          # p3 arm: what the broadcasts asked SUMO
    find_route_calls = []              # every findRoute, any caller
    set_route_calls = []

    original_find = traci.simulation.findRoute

    def logged_find(*a, **k):
        find_route_calls.append([step_ref["t"], list(map(str, a[:3]))])
        return original_find(*a, **k)

    traci.simulation.findRoute = logged_find

    original_set = traci.vehicle.setRoute

    def logged_set(vehicle_id, edges):
        set_route_calls.append([step_ref["t"], vehicle_id, len(list(edges))])
        return original_set(vehicle_id, edges)

    traci.vehicle.setRoute = logged_set

    original_rd = em_module.EmergencyManager._route_distance

    def logged_rd(self, from_edge, to_edge, vehicle_type=""):
        route_distance_calls.append([step_ref["t"], from_edge, to_edge, vehicle_type])
        return original_rd(self, from_edge, to_edge, vehicle_type)

    em_module.EmergencyManager._route_distance = logged_rd

    replay = {}
    if args.replay:
        for t, frm, to, vt in json.loads(Path(args.replay).read_text())["route_distance_calls"]:
            replay.setdefault(t, []).append((frm, to, vt))

    # ------------------------------------------------------------------ arm set-up
    coordinator = None
    if args.arm == "p3":
        from p3_runtime import P3RuntimeCoordinator
        coordinator = P3RuntimeCoordinator(emergency_layer=False)
        core = coordinator.e3
    else:
        reset_route_stats()
        seed_route_choice()
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
    ])
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
                        speed_factors[vehicle_id] = [t, round(traci.vehicle.getSpeedFactor(vehicle_id), 9)]
                except traci.TraCIException:
                    pass
            trace.append([t, len(ids), round(total, 4), rng_hash()])
    finally:
        try:
            traci.close()
        except Exception:
            pass

    (out / "result.json").write_text(json.dumps({
        "arm": args.arm,
        "trace": trace,
        "decisions": decisions,
        "speed_factors": speed_factors,
        "route_distance_calls": route_distance_calls,
        "find_route_calls": find_route_calls,
        "set_route_calls": set_route_calls,
    }), encoding="utf-8")
    print(f"[{args.arm}] done: {len(trace)} steps, {len(decisions)} decisions, "
          f"{len(find_route_calls)} findRoute, {len(route_distance_calls)} from broadcasts")
    return 0


def drive(duration: int) -> int:
    """Run the arms as separate processes and report where and why they diverge."""
    import subprocess
    import tempfile

    work = Path(tempfile.mkdtemp(prefix="route_query_drift_"))
    me = str(Path(__file__).resolve())

    def run(arm, replay=None):
        cmd = [sys.executable, me, "--arm", arm, "--duration", str(duration), "--out", str(work / arm)]
        if replay:
            cmd += ["--replay", str(replay)]
        print(f"[drift] running {arm} ...", flush=True)
        done = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True)
        if done.returncode != 0 or "Traceback" in (done.stderr or ""):
            print((done.stderr or "")[-3000:])
            raise SystemExit(f"arm {arm} failed or logged a traceback - no result may be quoted")
        return json.loads((work / arm / "result.json").read_text(encoding="utf-8"))

    p3 = run("p3")
    e3 = run("e3")
    e3_typed = run("e3_typed")
    replay = run("e3_replay", replay=work / "p3" / "result.json")

    def compare(a, b, name):
        first = next(((x, y) for x, y in zip(a["trace"], b["trace"]) if x[:3] != y[:3]), None)
        sa, sb = a["speed_factors"], b["speed_factors"]
        order = sorted(set(sa) & set(sb), key=lambda v: (sa[v][0], v))
        first_sf = next(((v, sa[v], sb[v]) for v in order if sa[v][1] != sb[v][1]), None)
        print(f"\n=== {name} ===")
        print(f"  physical state identical at every step : {first is None}")
        if first:
            print(f"    first difference [t, vehicles, metres driven, rng]: {first[0]} vs {first[1]}")
        print(f"  every vehicle's SUMO speed factor equal : {first_sf is None}")
        if first_sf:
            print(f"    first vehicle with a different speed factor: {first_sf[0]} "
                  f"(first seen t={first_sf[1][0]}): {first_sf[1][1]} vs {first_sf[2][1]}")
        same_rng = [x[3] for x in a["trace"]] == [y[3] for y in b["trace"]]
        print(f"  E3 random stream equal at every step   : {same_rng}")
        print(f"  routing decisions identical            : {a['decisions'] == b['decisions']} "
              f"({len(a['decisions'])} vs {len(b['decisions'])})")
        print(f"  route writes to SUMO identical         : {a['set_route_calls'] == b['set_route_calls']}")
        return first is None and first_sf is None and a["decisions"] == b["decisions"]

    paired = compare(p3, e3, "P3 entry point vs comparison-table entry point")
    insensitive = compare(e3, e3_typed, "comparison table vs the same plus ONE extra findRoute at t=300")
    explained = compare(p3, replay, "P3 vs comparison table replaying the P3 broadcasts' route queries")

    # Since 2026-09-12 the broadcasts' incident distances are cached, so "answers" (every
    # distance the broadcasts used) exceeds the route queries SUMO actually received.
    print(f"\n  route queries: P3 {len(p3['find_route_calls'])} (incident-distance answers used by "
          f"its broadcasts: {len(p3['route_distance_calls'])}); "
          f"comparison table {len(e3['find_route_calls'])}")
    print("\n=== verdict ===")
    # Since contract v0.7 (decision 13.13) every ordinary car's speed factor is frozen in
    # demand/porto.rou.xml, so route queries can no longer reach the traffic and the two entry
    # points must be identical WITHOUT any replay, however many queries each makes.
    if paired and insensitive:
        print("  PASS: true paired conditions. The two entry points are bit-for-bit identical")
        print(f"  although they make {len(p3['find_route_calls'])} and {len(e3['find_route_calls'])} "
              "route queries, and an extra query changes nothing.")
        print("  Route queries do not reach the traffic (contract decision 13.13).")
        return 0
    if explained:
        print("  FAIL: route queries are reaching vehicle speed factors again. Replaying them")
        print("  restores identity, so the router is fine - most likely demand/porto.rou.xml was")
        print("  regenerated without scripts/08b_freeze_speed_factors.py. Check it with")
        print("  `python scripts/08b_freeze_speed_factors.py --check`.")
        return 1
    print("  FAIL: the entry points differ even with the queries replayed - something other")
    print("  than route queries now differs between them.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
