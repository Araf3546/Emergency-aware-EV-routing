#!/usr/bin/env python3
"""Audit A10: is the ACO pheromone table stable, and what does it do to PSO?

Run before and after the A10 fix to judge it on measurements rather than intent.

What it reports, sampled through a real P3 run with the emergency layer off:

  * the pheromone value distribution over time - does it run away, collapse, or settle?
  * how many edges sit at the extremes;
  * whether pheromone still tells candidate edges apart (if every edge holds the same
    value it cannot steer anything, which would be over-correcting);
  * the knock-on to PSO velocity, which is computed from pheromone and currently pins to
    its -0.5 floor for almost every edge.

There is a specific defect this is meant to expose. Evaporation lives inside
`update_pheromone`, which the P3 path calls only when a reroute is actually applied - about
480 times an hour rather than once per simulation step. The decay rate therefore depends on
how often the router happens to act, which is not a property any ACO should have.

Writes nothing into the project.

    python scripts/diagnostics/pheromone_health.py --duration 1800
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

import traci  # noqa: E402
import p3_hybrid_router as H  # noqa: E402
from algorithms import aco_pso  # noqa: E402


def snapshot(sim_time):
    values = list(aco_pso.pheromone.values())
    if not values:
        return {"t": sim_time, "entries": 0}
    values_sorted = sorted(values)
    return {
        "t": sim_time,
        "entries": len(values),
        "min": round(values_sorted[0], 4),
        "median": round(statistics.median(values_sorted), 4),
        "max": round(values_sorted[-1], 4),
        "p90": round(values_sorted[int(0.9 * (len(values_sorted) - 1))], 4),
        "near_zero": sum(1 for v in values if v < 0.01),
        # Work order item 1: the global best is now PSO's best *realised route quality*
        # in [0, 1], not the highest pheromone value. A number near 5.0 here means an
        # old build; near the mean quality means the current one.
        "global_best": round(aco_pso.global_best_score, 4),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=1800)
    arguments = parser.parse_args()

    timeline = []
    pheromone_varies = 0
    velocity_varies = 0
    selections = 0

    _orig_select = H.HybridRouter.select_best_edge

    def select_best_edge(self, candidates):
        nonlocal pheromone_varies, velocity_varies, selections
        best, score = _orig_select(self, candidates)
        if len(candidates) > 1:
            selections += 1
            taus = [c.pheromone for _, c in candidates]
            # Raw, not max(0, v): the scorer no longer clamps negative velocities
            # either, because min-max normalisation across the candidate set already
            # makes the lowest velocity score 0 whatever its sign.
            vels = [c.velocity for _, c in candidates]
            if max(taus) - min(taus) > 1e-12:
                pheromone_varies += 1
            if max(vels) - min(vels) > 1e-12:
                velocity_varies += 1
        return best, score

    H.HybridRouter.select_best_edge = select_best_edge

    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = P3RuntimeCoordinator(emergency_layer=False)

    out_dir = Path(tempfile.mkdtemp(prefix="pheromone_health_"))
    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true",
        "--end", str(arguments.duration),
        "--tripinfo-output", str(out_dir / "tripinfo.xml"),
        "--error-log", str(out_dir / "errors.log"),
    ])
    try:
        while traci.simulation.getTime() < arguments.duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            coordinator.update_step(sim_time)
            if int(sim_time) % 300 == 0:
                timeline.append(snapshot(sim_time))
    finally:
        try:
            traci.close()
        except Exception:
            pass

    print(f"\n=== pheromone table over {arguments.duration}s ===")
    print(f"{'t':>6} {'entries':>8} {'min':>10} {'median':>10} {'p90':>10} {'max':>10} {'~0':>6} {'gbest':>8}")
    for row in timeline:
        if row.get("entries"):
            print(f"{row['t']:>6.0f} {row['entries']:>8} {row['min']:>10} {row['median']:>10} "
                  f"{row['p90']:>10} {row['max']:>10} {row['near_zero']:>6} {row['global_best']:>8}")

    values = list(aco_pso.pheromone.values())
    print(f"\n=== final table ===")
    if values:
        print(f"  entries                 : {len(values)}")
        print(f"  min / median / max      : {min(values):.4f} / {statistics.median(values):.4f} / {max(values):.4f}")
        print(f"  spread (max/min)        : {max(values)/max(min(values), 1e-12):.1f}x")
        print(f"  edges below 0.01        : {sum(1 for v in values if v < 0.01)} "
              f"({100*sum(1 for v in values if v < 0.01)/len(values):.1f}%)")
        print(f"  global best score       : {aco_pso.global_best_score:.4f}")

    print(f"\n=== can these terms still tell candidates apart? ===")
    print(f"  selections with a choice        : {selections}")
    if selections:
        print(f"  pheromone differs between them  : {pheromone_varies} ({100*pheromone_varies/selections:.1f}%)")
        print(f"  PSO velocity differs (as scored): {velocity_varies} ({100*velocity_varies/selections:.1f}%)")

    print("\n  A healthy result keeps pheromone bounded AND still discriminating. A table")
    print("  where every edge holds the same value is stable but useless.")
    print(f"\n  detail: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
