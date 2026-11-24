#!/usr/bin/env python3
"""Step 5 / audit C8: prove the PSO term is actually running and can change a route.

Before the fix, `p3_runtime._apply_hybrid_decision` read the PSO velocity with
`velocity.get(edge, 0.0)` from a dictionary that only `aco_pso.select_route` ever wrote -
and a P3 run never calls that function. Every candidate therefore scored a velocity of
exactly 0.0. A component identical for every candidate cannot change which candidate wins,
so the PSO weight was unreachable by tuning and the "PSO" half of ACO+PSO+BCO was not
running at all.

This checks three things, on real SUMO runs:

  1. PSO state changes           the velocity dictionary fills up and its values move
  2. PSO discriminates           velocity differs *between candidate edges* in real
                                 selections, which is what it must do to matter
  3. PSO changes route choices   turning its weight off changes at least some decisions

The controlled test for (3) is built so only one thing differs. Every arm still calls
`update_velocity` for every candidate, so all arms consume the same random draws in the
same order and the random streams stay aligned; the only difference is the velocity weight
used in scoring. Any divergence is therefore PSO's contribution to the score, not two runs
drifting apart by chance.

Writes nothing into the project.

    python scripts/diagnostics/verify_pso_active.py
    python scripts/diagnostics/verify_pso_active.py --duration 1800
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

MODES = ("pso-on", "pso-off", "pso-strong")

DEFAULT_VELOCITY_WEIGHT = 0.05  # HybridRouter's default

# "pso-strong" raises only the velocity weight, to the value pheromone carries. It is NOT a
# tuning proposal and this value is never adopted - it answers a question the on/off pair
# cannot. If turning PSO off changes nothing, is the mechanism still broken, or is it wired
# correctly but weighted too low to reach the argmax? If the strong arm diverges, the wiring
# works and the weight alone is what makes it inert.
STRONG_VELOCITY_WEIGHT = 0.4


def run_one_mode(mode: str, duration: int, log_path: Path) -> None:
    os.chdir(REPO_ROOT)

    import traci
    import p3_hybrid_router as H
    from algorithms import aco_pso

    decisions = []
    velocity_varied = 0
    selections = 0
    timeline = []
    # Criterion 3: is velocity carrying information of its own, or restating pheromone?
    # Velocity used to be computed *from* pheromone, so it could not do anything else; work
    # order item 1 made its fitness a realised travel outcome instead. The question is
    # whether it now ever ranks the candidates differently. If its favourite is always
    # pheromone's favourite, the term is redundant however well it is wired.
    same_favourite = 0
    comparable = 0

    _orig_select = H.HybridRouter.select_best_edge
    current = {"id": None, "time": 0.0}

    def select_best_edge(self, candidates):
        # All four counters must be declared, or `+=` makes them local and raises
        # UnboundLocalError the first time the branch is taken. p3_runtime catches every
        # exception from a routing decision and logs it, so the failure is invisible here
        # and shows up only as decisions quietly going missing.
        nonlocal velocity_varied, selections, comparable, same_favourite
        best, score = _orig_select(self, candidates)
        if len(candidates) > 1:
            selections += 1
            column = [context.velocity for _, context in candidates]
            if max(column) - min(column) > 1e-12:
                velocity_varied += 1

            taus = [context.pheromone for _, context in candidates]
            # Raw, not max(0, v). The scorer no longer clamps negative velocities either:
            # min-max normalisation across the candidate set already makes the lowest
            # velocity score 0, and clamping first threw away most of the discrimination
            # once velocity became a rate of change rather than a level.
            vels = [context.velocity for _, context in candidates]
            if max(taus) - min(taus) > 1e-12 and max(vels) - min(vels) > 1e-12:
                comparable += 1
                if taus.index(max(taus)) == vels.index(max(vels)):
                    same_favourite += 1
            decisions.append([current["time"], current["id"],
                              "|".join(edge for edge, _ in candidates), best])
        return best, score

    H.HybridRouter.select_best_edge = select_best_edge

    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager

    _orig_decide = P3RuntimeCoordinator._apply_hybrid_decision

    def apply_hybrid_decision(self, vehicle_id, sim_time):
        current["id"] = vehicle_id
        current["time"] = sim_time
        return _orig_decide(self, vehicle_id, sim_time)

    P3RuntimeCoordinator._apply_hybrid_decision = apply_hybrid_decision

    coordinator = P3RuntimeCoordinator()
    if mode == "pso-off":
        coordinator.router.weights["velocity"] = 0.0
    elif mode == "pso-strong":
        coordinator.router.weights["velocity"] = STRONG_VELOCITY_WEIGHT

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    out_dir = log_path.parent
    traci.start([
        "sumo", "-c", "simulation.sumocfg", "--no-step-log", "true", "--end", str(duration),
        "--tripinfo-output", str(out_dir / f"{mode}_tripinfo.xml"),
        "--error-log", str(out_dir / f"{mode}_errors.log"),
    ])
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            coordinator.update_step(sim_time)
            if int(sim_time) % 300 == 0 and aco_pso.velocity:
                timeline.append({
                    "t": sim_time,
                    "entries": len(aco_pso.velocity),
                    "min": round(min(aco_pso.velocity.values()), 6),
                    "max": round(max(aco_pso.velocity.values()), 6),
                    "global_best": round(aco_pso.global_best_score, 6),
                })
    finally:
        try:
            traci.close()
        except Exception:
            pass

    values = list(aco_pso.velocity.values())
    # How saturated is it? A velocity pinned at one of the two clamp bounds is a binary
    # flag, not a continuous search signal.
    at_bound = sum(1 for v in values if abs(abs(v) - aco_pso.VMAX) < 1e-9)
    histogram = {}
    for value in values:
        bucket = round(value, 1)
        histogram[bucket] = histogram.get(bucket, 0) + 1

    log_path.write_text(json.dumps({
        "mode": mode,
        "decisions": decisions,
        "selections": selections,
        "selections_where_velocity_varies": velocity_varied,
        "velocity_entries": len(values),
        "velocity_distinct_values": len({round(v, 9) for v in values}),
        "velocity_min": round(min(values), 6) if values else None,
        "velocity_max": round(max(values), 6) if values else None,
        "velocity_at_clamp_bound": at_bound,
        "velocity_histogram": sorted(histogram.items()),
        "global_best_score": round(aco_pso.global_best_score, 6),
        "personal_best_entries": len(aco_pso.personal_best),
        "selections_where_both_discriminate": comparable,
        "selections_where_pso_agrees_with_pheromone": same_favourite,
        "timeline": timeline,
    }), encoding="utf-8")


def diverges(first, second):
    a, b = first["decisions"], second["decisions"]
    prefix = 0
    for left, right in zip(a, b):
        if left != right:
            break
        prefix += 1
    return a != b, prefix, a, b


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--mode", choices=MODES, help="internal: run a single arm")
    parser.add_argument("--log", help="internal: where the single arm writes its log")
    arguments = parser.parse_args()

    if arguments.mode:
        run_one_mode(arguments.mode, arguments.duration, Path(arguments.log))
        return 0

    work_dir = Path(tempfile.mkdtemp(prefix="pso_active_"))
    logs = {}
    for mode in MODES:
        log_path = work_dir / f"{mode}.json"
        print(f"[pso] running {mode} for {arguments.duration}s ...", flush=True)
        subprocess.run(
            [sys.executable, __file__, "--mode", mode, "--duration", str(arguments.duration),
             "--log", str(log_path)],
            check=True, cwd=str(REPO_ROOT),
        )
        logs[mode] = json.loads(log_path.read_text(encoding="utf-8"))

    on, off, strong = logs["pso-on"], logs["pso-off"], logs["pso-strong"]

    print("\n=== 1. is PSO state actually changing? ===")
    print(f"  velocity dictionary entries      : {on['velocity_entries']}  (was 0 before the fix)")
    print(f"  distinct velocity values         : {on['velocity_distinct_values']}")
    print(f"  velocity range                   : {on['velocity_min']} .. {on['velocity_max']}")
    print(f"  personal-best entries            : {on['personal_best_entries']}")
    print(f"  global best score                : {on['global_best_score']}  (stays 0.0 if never recorded)")
    print(f"  velocities pinned at +/-VMAX     : {on['velocity_at_clamp_bound']} of {on['velocity_entries']}")
    print(f"  distribution                     : {on['velocity_histogram']}")
    for row in on["timeline"]:
        print(f"    t={row['t']:6.0f} entries={row['entries']:5d} "
              f"range={row['min']}..{row['max']} global_best={row['global_best']}")

    print("\n=== 2. does the PSO value differ between candidate edges? ===")
    share = 100 * on["selections_where_velocity_varies"] / max(1, on["selections"])
    print(f"  selections with a real choice    : {on['selections']}")
    print(f"  ...where velocity differs        : {on['selections_where_velocity_varies']} ({share:.1f}%)")
    print("  (before the fix this was 0.0% - every candidate scored velocity 0.0)")

    print("\n=== 2b. is PSO saying anything pheromone does not? ===")
    comparable = on["selections_where_both_discriminate"]
    agree = on["selections_where_pso_agrees_with_pheromone"]
    if comparable:
        print(f"  selections where both terms discriminate : {comparable}")
        print(f"  ...where PSO's favourite is pheromone's  : {agree} ({100*agree/comparable:.1f}%)")
        print("  100% would mean PSO restates pheromone and adds nothing of its own.")
    else:
        print("  never comparable - one of the two terms is constant in every selection.")

    print("\n=== 3. does disabling PSO change route decisions? ===")
    changed, prefix, a, b = diverges(on, off)
    print(f"  decisions recorded               : {len(a)} on, {len(b)} off")
    print(f"  identical before first difference: {prefix}")
    print(f"  default weight {DEFAULT_VELOCITY_WEIGHT} changes decisions: {changed}")
    if changed and prefix < min(len(a), len(b)):
        print(f"    first difference: on ={a[prefix]}")
        print(f"                      off={b[prefix]}")

    strong_changed, strong_prefix, sa, sb = diverges(strong, off)
    print(f"\n  is the mechanism reachable at all? raising ONLY the velocity weight to "
          f"{STRONG_VELOCITY_WEIGHT} (not a tuning proposal):")
    print(f"    identical before first difference: {strong_prefix} of {len(sa)}")
    print(f"    weight {STRONG_VELOCITY_WEIGHT} changes decisions        : {strong_changed}")
    if strong_changed and strong_prefix < min(len(sa), len(sb)):
        print(f"      first difference: strong={sa[strong_prefix]}")
        print(f"                        off   ={sb[strong_prefix]}")

    print("\n=== verdict ===")
    failures = []
    if on["velocity_entries"] == 0:
        failures.append("the velocity dictionary is still empty - PSO is not running")
    if on["velocity_distinct_values"] <= 1:
        failures.append("velocity has one value everywhere - it cannot discriminate")
    if on["selections_where_velocity_varies"] == 0:
        failures.append("velocity never differs between candidates - the weight is unreachable")
    if not changed and not strong_changed:
        failures.append("PSO changes no route decision even at a large weight - still broken")

    for line in failures:
        print(f"  FAIL: {line}")

    if not failures and not changed:
        print(f"  PASS with a caveat: PSO state evolves and differs between candidates, and at "
              f"weight {STRONG_VELOCITY_WEIGHT} it changes route decisions - so the wiring is "
              f"correct and the term is reachable. At its default weight of "
              f"{DEFAULT_VELOCITY_WEIGHT} it changes none. That is a tuning question, not a "
              f"correctness one.")
    elif not failures:
        print("  PASS: PSO state evolves, differs between candidates, and changes route choices "
              "at its default weight.")

    print(f"\n  logs: {work_dir}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
