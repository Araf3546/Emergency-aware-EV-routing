#!/usr/bin/env python3
"""Do BOTH halves of the heuristic still influence route choice?

The heuristic now mixes congestion with distance to destination, as
`design_phase.md` section 2.3 always specified. Two things can go wrong with that, in
opposite directions, and neither shows up in the detour statistics:

  * the distance half does nothing, and the detour problem is untouched;
  * the distance half does everything, and E3 has quietly become a shortest-path router
    with four decorative extra terms.

Three arms, identical in every other respect, each recording every route decision:

  both        the heuristic as shipped
  no-progress the distance half forced to a constant, leaving congestion alone
  no-congest  the congestion half forced to a constant, leaving distance alone

A constant term cannot change an argmax, so each arm answers one question: if turning a
half off changes decisions, that half was doing work. Both must change decisions. If
`no-congest` matches `both`, congestion has been swallowed and E3 is A* in disguise.

Writes nothing into the project.

    python scripts/diagnostics/verify_heuristic_halves.py --duration 900
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

MODES = ("both", "no-progress", "no-congest")


def run_one_mode(mode: str, duration: int, log_path: Path) -> None:
    os.chdir(REPO_ROOT)

    import traci
    import p3_runtime
    from algorithms.e3_core import E3Core
    from p3_runtime import P3RuntimeCoordinator
    from emergency_manager import get_emergency_manager

    decisions = []
    counts = {"selections": 0, "congestion_varies": 0, "progress_varies": 0,
              "both_vary": 0, "halves_disagree": 0}

    # Work order item 2: these three moved from P3RuntimeCoordinator into the one
    # shared E3Core. Patching them on the coordinator would still run and still print
    # a verdict, while silencing a method nobody calls.
    original_congestion = E3Core.congestion_suitability
    original_progress = E3Core.progress_suitability
    original_blend = E3Core.blend_heuristic

    # Silencing a half by returning a constant is exactly how the arms differ. Everything
    # else - the other four scoring terms, the seed, the demand - is untouched.
    if mode == "no-progress":
        E3Core.progress_suitability = lambda self, edge, dest: 0.5
    elif mode == "no-congest":
        E3Core.congestion_suitability = staticmethod(lambda edge: 0.5)

    def blend(congestion, progress):
        counts["selections"] += 1
        c_varies = max(congestion) - min(congestion) > 1e-12
        p_varies = max(progress) - min(progress) > 1e-12
        counts["congestion_varies"] += c_varies
        counts["progress_varies"] += p_varies
        if c_varies and p_varies:
            counts["both_vary"] += 1
            if congestion.index(max(congestion)) != progress.index(max(progress)):
                counts["halves_disagree"] += 1
        return original_blend(congestion, progress)

    E3Core.blend_heuristic = staticmethod(blend)

    current = {"id": None, "time": 0.0}
    original_decide = P3RuntimeCoordinator._apply_hybrid_decision

    def apply_hybrid_decision(self, vehicle_id, sim_time):
        current["id"] = vehicle_id
        current["time"] = sim_time
        return original_decide(self, vehicle_id, sim_time)

    P3RuntimeCoordinator._apply_hybrid_decision = apply_hybrid_decision

    import p3_hybrid_router as H
    original_select = H.HybridRouter.select_best_edge

    def select_best_edge(self, candidates):
        best, score = original_select(self, candidates)
        if len(candidates) > 1:
            decisions.append([current["time"], current["id"],
                              "|".join(edge for edge, _ in candidates), best])
        return best, score

    H.HybridRouter.select_best_edge = select_best_edge

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    coordinator = P3RuntimeCoordinator(emergency_layer=False)

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
    finally:
        try:
            traci.close()
        except Exception:
            pass

    E3Core.congestion_suitability = original_congestion
    E3Core.progress_suitability = original_progress
    E3Core.blend_heuristic = original_blend

    log_path.write_text(json.dumps({
        "mode": mode,
        "decisions": decisions,
        "decision_count": len(decisions),
        **counts,
    }), encoding="utf-8")


def diverges(first, second):
    a, b = first["decisions"], second["decisions"]
    prefix = 0
    for left, right in zip(a, b):
        if left != right:
            break
        prefix += 1
    return a != b, prefix


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=900)
    parser.add_argument("--mode", choices=MODES, help="internal: run a single arm")
    parser.add_argument("--log", help="internal: where the single arm writes its log")
    arguments = parser.parse_args()

    if arguments.mode:
        run_one_mode(arguments.mode, arguments.duration, Path(arguments.log))
        return 0

    work_dir = Path(tempfile.mkdtemp(prefix="heuristic_halves_"))
    logs = {}
    for mode in MODES:
        log_path = work_dir / f"{mode}.json"
        print(f"[halves] running {mode} for {arguments.duration}s ...", flush=True)
        subprocess.run(
            [sys.executable, __file__, "--mode", mode, "--duration", str(arguments.duration),
             "--log", str(log_path)],
            check=True, cwd=str(REPO_ROOT),
        )
        logs[mode] = json.loads(log_path.read_text(encoding="utf-8"))

    both = logs["both"]
    print("\n=== how often does each half have an opinion? ===")
    selections = max(1, both["selections"])
    print(f"  candidate sets scored          : {both['selections']}")
    print(f"  congestion differs between them: {both['congestion_varies']} "
          f"({100*both['congestion_varies']/selections:.1f}%)")
    print(f"  distance differs between them  : {both['progress_varies']} "
          f"({100*both['progress_varies']/selections:.1f}%)")
    print(f"  both have an opinion           : {both['both_vary']} "
          f"({100*both['both_vary']/selections:.1f}%)")
    print(f"  ...and they disagree           : {both['halves_disagree']} "
          f"({100*both['halves_disagree']/max(1, both['both_vary']):.1f}% of those)")

    print("\n=== does silencing a half change route decisions? ===")
    results = {}
    for mode in ("no-progress", "no-congest"):
        changed, prefix = diverges(both, logs[mode])
        results[mode] = changed
        label = "distance" if mode == "no-progress" else "congestion"
        print(f"  silencing {label:11s}: decisions change = {changed} "
              f"(identical for the first {prefix} of {both['decision_count']})")

    print("\n=== verdict ===")
    failures = []
    if not results["no-progress"]:
        failures.append("the distance half changes nothing - the detour fix is not active")
    if not results["no-congest"]:
        failures.append("the congestion half changes nothing - E3 is now a shortest-path "
                        "router with four decorative terms")
    for line in failures:
        print(f"  FAIL: {line}")
    if not failures:
        print("  PASS: both halves influence route choice. The heuristic weighs congestion")
        print("        and distance together, which is what design_phase.md 2.3 specifies.")
    print(f"\n  logs: {work_dir}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
