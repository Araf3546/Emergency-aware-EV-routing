#!/usr/bin/env python3
"""What does the corrected BCO term actually do? (audit A6)

BCO's memory records how recently an edge was used. The score used to read that value
directly, so the edge just chosen became the *most* attractive next time - reinforcement,
not exploration. The correction inverts the reading only.

This runs two arms that differ in exactly one number, the BCO weight, with the emergency
layer off so the router is measured alone:

  bco-on    the weight as shipped
  bco-off   the BCO weight set to 0.0

and answers four questions the correction has to satisfy:

  1. does BCO now steer toward less recently used edges?
  2. does it change route decisions at all, against the arm where it is switched off?
  3. do congestion and destination progress still influence the choice - has BCO
     overwhelmed them?
  4. does traffic get spread across more roads, or does the router keep piling onto the
     same few edges?

Question 4 is measured on the edges the router actually chooses. Concentration is reported
as the share taken by the busiest ten edges and as a Gini coefficient, where 0 is perfectly
even and 1 is everything on one road. Nothing about the algorithm is changed to measure it.

Writes nothing into the project.

    python scripts/diagnostics/bco_effect.py --duration 3600
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

MODES = ("bco-on", "bco-off")

# Half-life arms, in simulated seconds. `bco-on` keeps whatever the code's default is.
# 30 s  - one reroute interval, the minimum for a vehicle to still remember an edge at its
#         own next decision;
# 120 s - about four decision cycles and 40% of a mean trip (293 s);
# 300 s - the pheromone half-life set by A10, so reinforcement and exploration fade
#         together rather than one outlasting the other.
HALF_LIFE_CANDIDATES = (30.0, 120.0, 300.0)


def gini(counts) -> float:
    values = sorted(counts)
    total = sum(values)
    if not values or total == 0:
        return 0.0
    n = len(values)
    weighted = sum((index + 1) * value for index, value in enumerate(values))
    return (2 * weighted) / (n * total) - (n + 1) / n


def run_one_mode(mode: str, duration: int, log_path: Path) -> None:
    os.chdir(REPO_ROOT)

    import traci
    import p3_hybrid_router as H
    from algorithms.e3_core import E3Core
    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager
    from metrics.trip_metrics import build_report

    decisions = []
    chosen_edges = collections.Counter()
    stats = collections.Counter()
    scout_gap = []          # scout memory of the chosen edge minus the mean of the rest

    original_select = H.HybridRouter.select_best_edge
    current = {"id": None, "time": 0.0}

    def select_best_edge(self, candidates):
        best, score = original_select(self, candidates)
        if len(candidates) > 1:
            stats["selections"] += 1
            decisions.append([current["time"], current["id"],
                              "|".join(edge for edge, _ in candidates), best])

            bco = [context.bco_exploration for _, context in candidates]
            if max(bco) - min(bco) > 1e-12:
                stats["bco_discriminates"] += 1

            # Question 1, measured on the raw memory rather than the scored term: is the
            # edge the router picks one that has been used *less* than the alternatives?
            memory = {edge: self.scout_memory.get(edge, 0.0) for edge, _ in candidates}
            others = [value for edge, value in memory.items() if edge != best]
            if others:
                scout_gap.append(memory.get(best, 0.0) - statistics.mean(others))
        return best, score

    H.HybridRouter.select_best_edge = select_best_edge

    original_decide = P3RuntimeCoordinator._apply_hybrid_decision

    def apply_hybrid_decision(self, vehicle_id, sim_time):
        current["id"] = vehicle_id
        current["time"] = sim_time
        return original_decide(self, vehicle_id, sim_time)

    P3RuntimeCoordinator._apply_hybrid_decision = apply_hybrid_decision

    # Work order item 2: the heuristic moved into the one shared E3Core.
    original_blend = E3Core.blend_heuristic

    def blend(congestion, progress):
        if max(congestion) - min(congestion) > 1e-12:
            stats["congestion_discriminates"] += 1
        if max(progress) - min(progress) > 1e-12:
            stats["progress_discriminates"] += 1
        return original_blend(congestion, progress)

    E3Core.blend_heuristic = staticmethod(blend)

    original_set_route = None
    import p3_runtime

    original_set_route = p3_runtime.set_route_checked

    def set_route_checked(vehicle_id, new_route, current_route):
        applied = original_set_route(vehicle_id, new_route, current_route)
        if applied:
            stats["reroutes_applied"] += 1
            for edge in new_route:
                chosen_edges[edge] += 1
        return applied

    p3_runtime.set_route_checked = set_route_checked

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = P3RuntimeCoordinator(emergency_layer=False)
    if mode == "bco-off":
        coordinator.router.weights["bco"] = 0.0
    elif mode.startswith("hl-"):
        coordinator.router.bco_half_life_s = float(mode[3:])

    out_dir = log_path.parent
    tripinfo = out_dir / f"{mode}_tripinfo.xml"
    summary = out_dir / f"{mode}_summary.xml"
    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true", "--end", str(duration),
        "--tripinfo-output", str(tripinfo),
        "--summary-output", str(summary),
        "--error-log", str(out_dir / f"{mode}_errors.log"),
    ])
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            coordinator.update_step(sim_time)
    finally:
        try:
            traci.close()
        except Exception:
            pass

    from p3_simulation_config import EMERGENCY_VEHICLE_TYPES
    report = build_report(tripinfo, summary, emergency_types=EMERGENCY_VEHICLE_TYPES)

    counts = list(chosen_edges.values())
    top_ten = sum(sorted(counts, reverse=True)[:10])
    log_path.write_text(json.dumps({
        "mode": mode,
        "decisions": decisions,
        "decision_count": len(decisions),
        "stats": dict(stats),
        "mean_scout_gap": round(statistics.mean(scout_gap), 5) if scout_gap else None,
        "bco_half_life_s": coordinator.router.bco_half_life_s,
        "live_memory_entries_at_end": len(coordinator.router.scout_memory),
        "distinct_edges_used": len(counts),
        "edge_use_total": sum(counts),
        "top10_share": round(top_ten / sum(counts), 4) if counts else None,
        "gini": round(gini(counts), 4) if counts else None,
        "network": {key: report.get(key) for key in
                    ("completed_trips", "mean_travel_time_s", "mean_time_loss_s",
                     "mean_waiting_time_s", "teleports_total")},
        "responses": {str(e): s["response_time_s"]
                      for e, s in manager.get_response_summary().items()},
    }), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=3600)
    parser.add_argument("--mode")
    parser.add_argument("--half-lives", default=None,
                        help="comma-separated half-lives in seconds; runs one arm each")
    parser.add_argument("--log")
    arguments = parser.parse_args()

    if arguments.mode:
        run_one_mode(arguments.mode, arguments.duration, Path(arguments.log))
        return 0

    if arguments.half_lives:
        values = [v.strip() for v in arguments.half_lives.split(",") if v.strip()]
        arms = [f"hl-{v}" for v in values]
    else:
        arms = list(MODES)

    work_dir = Path(tempfile.mkdtemp(prefix="bco_effect_"))
    logs = {}
    for mode in arms:
        log_path = work_dir / f"{mode}.json"
        print(f"[bco] running {mode} for {arguments.duration}s ...", flush=True)
        subprocess.run(
            [sys.executable, __file__, "--mode", mode, "--duration", str(arguments.duration),
             "--log", str(log_path)],
            check=True, cwd=str(REPO_ROOT),
        )
        logs[mode] = json.loads(log_path.read_text(encoding="utf-8"))

    if arguments.half_lives:
        print("")
        print(f"{'metric':32s} " + " ".join(f"{a:>12}" for a in arms))
        rows = [
            ("BCO half-life (s)", lambda d: d["bco_half_life_s"]),
            ("live memory entries at end", lambda d: d["live_memory_entries_at_end"]),
            ("scout gap (neg = explores)", lambda d: d["mean_scout_gap"]),
            ("selections with a choice", lambda d: d["stats"].get("selections", 0)),
            ("BCO discriminates", lambda d: d["stats"].get("bco_discriminates", 0)),
            ("distinct edges used", lambda d: d["distinct_edges_used"]),
            ("busiest-10 share", lambda d: d["top10_share"]),
            ("Gini (0 even, 1 one road)", lambda d: d["gini"]),
            ("reroutes applied", lambda d: d["stats"].get("reroutes_applied", 0)),
            ("completed trips", lambda d: d["network"]["completed_trips"]),
            ("teleports", lambda d: d["network"]["teleports_total"]),
            ("mean time loss (s)", lambda d: d["network"]["mean_time_loss_s"]),
            ("mean waiting (s)", lambda d: d["network"]["mean_waiting_time_s"]),
            ("mean travel time (s)", lambda d: d["network"]["mean_travel_time_s"]),
        ]
        for label, getter in rows:
            print(f"  {label:30s} " + " ".join(f"{getter(logs[a])!s:>12}" for a in arms))
        print(f"\n  logs: {work_dir}")
        return 0

    on, off = logs["bco-on"], logs["bco-off"]
    s_on = on["stats"]

    print("\n=== 1. does BCO steer toward less recently used edges? ===")
    print(f"  mean scout-memory of the chosen edge minus the alternatives: {on['mean_scout_gap']}")
    print("  negative = the router picks edges used LESS than the alternatives (exploration)")
    print("  positive = it picks edges used MORE (the defect A6 described)")

    print("\n=== 2. does BCO change route decisions? ===")
    a, b = on["decisions"], off["decisions"]
    prefix = 0
    for left, right in zip(a, b):
        if left != right:
            break
        prefix += 1
    print(f"  decisions: {len(a)} on, {len(b)} off")
    print(f"  identical before first difference: {prefix}")
    print(f"  BCO changes decisions: {a != b}")

    print("\n=== 3. do the other terms still discriminate? ===")
    selections = max(1, s_on.get("selections", 0))
    print(f"  selections with a real choice : {s_on.get('selections', 0)}")
    print(f"  BCO discriminates            : {s_on.get('bco_discriminates', 0)}")
    print(f"  congestion discriminates     : {s_on.get('congestion_discriminates', 0)}")
    print(f"  destination progress does    : {s_on.get('progress_discriminates', 0)}")

    print("\n=== 4. is traffic spread more widely? ===")
    print(f"{'':28s} {'BCO on':>12} {'BCO off':>12}")
    for key, label in (("distinct_edges_used", "distinct edges used"),
                       ("edge_use_total", "total edge assignments"),
                       ("top10_share", "share on busiest 10 edges"),
                       ("gini", "Gini (0=even, 1=one road)")):
        print(f"  {label:26s} {on[key]!s:>12} {off[key]!s:>12}")
    print(f"  {'reroutes applied':26s} {s_on.get('reroutes_applied', 0)!s:>12} "
          f"{off['stats'].get('reroutes_applied', 0)!s:>12}")

    print("\n=== network performance ===")
    print(f"{'':28s} {'BCO on':>12} {'BCO off':>12}")
    for key in ("completed_trips", "mean_travel_time_s", "mean_time_loss_s",
                "mean_waiting_time_s", "teleports_total"):
        print(f"  {key:26s} {on['network'][key]!s:>12} {off['network'][key]!s:>12}")
    print("\n  responses:")
    for event in sorted(on["responses"]):
        print(f"    {event}: on={on['responses'][event]}  off={off['responses'][event]}")

    print(f"\n  logs: {work_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
