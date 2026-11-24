#!/usr/bin/env python3
"""Work order item 1: is the swarm actually learning from what happened?

Before this change nothing in the codebase recorded what a route cost after it was driven.
`deposit()` laid a flat Q = 1.0 on every edge of every applied route the instant it was
applied, so pheromone measured how often an edge was *chosen*; PSO's fitness was read out of
that same table, so it could not say anything pheromone did not. `design_phase.md` 2.5 and
2.6 ask for the opposite - "pheromone updates from **successful** route choices", "velocity
updates from route improvement or deterioration", "the system must learn from actual
outcomes, not just from predicted scores".

This reports, from one real P3 run with the emergency layer off:

  1. how many route choices were actually measured, and how many were discarded and why;
  2. the distribution of realised quality - if it is a single value the deposit is flat
     again under a new name;
  3. what that does to the pheromone table: does it still tell candidate edges apart?
  4. whether PSO now carries information of its own, measured as how often its preferred
     candidate differs from pheromone's.

It also cross-checks the two counters that must agree (`selections` against recorded
decisions) and prints whether the run log contains a traceback, because `p3_runtime`
swallows every exception from a routing decision and a broken diagnostic otherwise produces
a healthy-looking but fabricated report.

Writes nothing into the project.

    python scripts/diagnostics/outcome_learning.py --duration 1800
"""

from __future__ import annotations

import argparse
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


def percentiles(values, points=(0, 10, 25, 50, 75, 90, 100)):
    if not values:
        return {}
    ordered = sorted(values)
    return {
        p: round(ordered[min(len(ordered) - 1, int(p / 100 * (len(ordered) - 1)))], 4)
        for p in points
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=1800)
    parser.add_argument("--deposit-scope", choices=("travelled", "planned"), default=None)
    arguments = parser.parse_args()

    if arguments.deposit_scope:
        os.environ["P3_DEPOSIT_SCOPE"] = arguments.deposit_scope

    counters = {
        "selections": 0,
        "decisions_seen": 0,
        "pheromone_varies": 0,
        "velocity_varies_raw": 0,
        "velocity_varies_as_scored": 0,
        "both_discriminate": 0,
        "pso_agrees_with_pheromone": 0,
    }
    velocity_signs = {"positive": 0, "negative": 0, "zero": 0}

    _orig_select = H.HybridRouter.select_best_edge

    def select_best_edge(self, candidates):
        # Every counter touched below must be reachable from here, or `+=` makes it local,
        # raises UnboundLocalError, and p3_runtime silently drops the decision. A dict is
        # used rather than `nonlocal` because that failure has already happened once in this
        # project and produced an entirely fabricated finding.
        best, score = _orig_select(self, candidates)
        if len(candidates) > 1:
            counters["selections"] += 1
            taus = [c.pheromone for _, c in candidates]
            raw = [c.velocity for _, c in candidates]
            scored = [max(0.0, c.velocity) for _, c in candidates]
            for value in raw:
                if value > 1e-12:
                    velocity_signs["positive"] += 1
                elif value < -1e-12:
                    velocity_signs["negative"] += 1
                else:
                    velocity_signs["zero"] += 1
            if max(taus) - min(taus) > 1e-12:
                counters["pheromone_varies"] += 1
            if max(raw) - min(raw) > 1e-12:
                counters["velocity_varies_raw"] += 1
            if max(scored) - min(scored) > 1e-12:
                counters["velocity_varies_as_scored"] += 1
            if max(taus) - min(taus) > 1e-12 and max(raw) - min(raw) > 1e-12:
                counters["both_discriminate"] += 1
                if taus.index(max(taus)) == raw.index(max(raw)):
                    counters["pso_agrees_with_pheromone"] += 1
        return best, score

    H.HybridRouter.select_best_edge = select_best_edge

    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager

    _orig_decide = P3RuntimeCoordinator._apply_hybrid_decision

    def apply_hybrid_decision(self, vehicle_id, sim_time):
        counters["decisions_seen"] += 1
        return _orig_decide(self, vehicle_id, sim_time)

    P3RuntimeCoordinator._apply_hybrid_decision = apply_hybrid_decision

    qualities = []
    strengths = []
    edges_per_outcome = []
    _orig_settle = P3RuntimeCoordinator._settle_route_outcomes

    def settle(self, sim_time):
        for outcome in self.outcomes.update(sim_time):
            aco_pso.record_outcome(outcome.travelled_edges, outcome.quality)
            target = (
                outcome.planned_route
                if self.deposit_scope == "planned"
                else outcome.travelled_edges
            )
            qualities.append(outcome.quality)
            strengths.append(aco_pso.deposit_strength(outcome.quality))
            edges_per_outcome.append(len(target))
            if target:
                aco_pso.deposit(target, quality=outcome.quality)

    P3RuntimeCoordinator._settle_route_outcomes = settle

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = P3RuntimeCoordinator(emergency_layer=False)

    out_dir = Path(tempfile.mkdtemp(prefix="outcome_learning_"))
    timeline = []
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
                values = list(aco_pso.pheromone.values())
                timeline.append({
                    "t": sim_time,
                    "outcomes": aco_pso.outcomes_recorded,
                    "mean_quality": round(aco_pso.quality_mean, 4),
                    "tau_entries": len(values),
                    "tau_median": round(statistics.median(values), 3) if values else None,
                    "tau_max": round(max(values), 3) if values else None,
                    "fitness_entries": len(aco_pso.edge_fitness),
                    "global_best": round(aco_pso.global_best_score, 4),
                })
    finally:
        try:
            traci.close()
        except Exception:
            pass

    scope = coordinator.deposit_scope
    stats = coordinator.outcomes.stats()

    print(f"\n=== deposit scope: {scope} ===")

    print("\n=== 1. were route choices actually measured? ===")
    for key in ("decisions_watched", "outcomes_reported", "settled_by_new_decision",
                "settled_by_horizon", "settled_by_leaving", "discarded_too_short",
                "discarded_no_geometry", "discarded_implausible", "unsettled_at_end"):
        print(f"  {key:<28}: {stats[key]}")
    reported = stats["outcomes_reported"]
    if stats["decisions_watched"]:
        print(f"  share of applied reroutes measured : "
              f"{100 * reported / stats['decisions_watched']:.1f}%")

    print("\n=== 2. what did those routes actually achieve? ===")
    if qualities:
        print(f"  realised quality (1.0 = free flow) : "
              f"mean {statistics.mean(qualities):.4f}, "
              f"median {statistics.median(qualities):.4f}, "
              f"distinct {len(set(round(q, 4) for q in qualities))}")
        print(f"  percentiles                        : {percentiles(qualities)}")
        print(f"  deposit strength (flat rule = 1.0) : "
              f"mean {statistics.mean(strengths):.3f}, "
              f"median {statistics.median(strengths):.3f}, "
              f"max {max(strengths):.3f}")
        print(f"  percentiles                        : {percentiles(strengths)}")
        print(f"  edges reinforced per outcome       : "
              f"mean {statistics.mean(edges_per_outcome):.1f}, "
              f"total {sum(edges_per_outcome)}")
        print("  A distribution collapsed onto one value would mean the deposit is flat")
        print("  again under a new name - the defect this change exists to remove.")
    else:
        print("  FAIL: no outcome was ever measured.")

    print("\n=== 3. does the pheromone table still tell candidates apart? ===")
    values = list(aco_pso.pheromone.values())
    if values:
        print(f"  entries                            : {len(values)}")
        print(f"  min / median / max                 : "
              f"{min(values):.4f} / {statistics.median(values):.4f} / {max(values):.4f}")
        print(f"  at the TAU_MAX ceiling             : "
              f"{sum(1 for v in values if v >= aco_pso.TAU_MAX - 1e-9)}")
    print(f"  selections with a real choice      : {counters['selections']}")
    if counters["selections"]:
        print(f"  pheromone differs between them     : {counters['pheromone_varies']} "
              f"({100 * counters['pheromone_varies'] / counters['selections']:.1f}%)")

    print("\n=== 4. is PSO saying anything pheromone does not? ===")
    print(f"  edges with a realised fitness      : {len(aco_pso.edge_fitness)}")
    print(f"  running mean quality               : {aco_pso.quality_mean:.4f}")
    print(f"  global best (decaying)             : {aco_pso.global_best_score:.4f}")
    print(f"  velocity entries                   : {len(aco_pso.velocity)}")
    if aco_pso.velocity:
        vel = list(aco_pso.velocity.values())
        at_bound = sum(1 for v in vel if abs(abs(v) - aco_pso.VMAX) < 1e-9)
        print(f"  velocity range                     : {min(vel):.4f} .. {max(vel):.4f}")
        print(f"  pinned at +/-VMAX                  : {at_bound} of {len(vel)} "
              f"(was 167 of 202 when velocity came from pheromone)")
    print(f"  candidate velocity signs           : {velocity_signs}")
    if counters["selections"]:
        print(f"  velocity differs, raw              : {counters['velocity_varies_raw']} "
              f"({100 * counters['velocity_varies_raw'] / counters['selections']:.1f}%)")
        print(f"  ...had the old max(0,v) clamp held : "
              f"{counters['velocity_varies_as_scored']} "
              f"({100 * counters['velocity_varies_as_scored'] / counters['selections']:.1f}%)")
        print("  The first line is what the scorer now sees. The second is the counterfactual:")
        print("  `component_values` used to return max(0, velocity), and that clamp is what")
        print("  the gap between the two lines would have thrown away.")
    if counters["both_discriminate"]:
        agree = counters["pso_agrees_with_pheromone"]
        print(f"  both terms discriminate            : {counters['both_discriminate']}")
        print(f"  ...PSO's favourite is pheromone's  : {agree} "
              f"({100 * agree / counters['both_discriminate']:.1f}%)  "
              f"(33.3% when PSO was derived from pheromone)")

    print("\n=== timeline ===")
    for row in timeline:
        print(f"  t={row['t']:6.0f} outcomes={row['outcomes']:5d} "
              f"mean_q={row['mean_quality']:.4f} tau_n={row['tau_entries']:5d} "
              f"tau_med={row['tau_median']} tau_max={row['tau_max']} "
              f"fitness_n={row['fitness_entries']:5d} gbest={row['global_best']:.4f}")

    print("\n=== integrity checks - read these before quoting any number above ===")
    print(f"  selections {counters['selections']} against decisions seen "
          f"{counters['decisions_seen']}: selections must be the smaller "
          f"(most junctions offer one way out)")
    log = out_dir / "errors.log"
    print(f"  SUMO error log                     : {log}")
    print(f"  detail                             : {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
