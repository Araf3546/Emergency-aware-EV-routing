#!/usr/bin/env python3
"""Verify route validity and determinism for every routing algorithm (audit C1-C14).

Runs each algorithm against the real scenario through the same code path the experiments
use - `should_reroute` -> the algorithm's `select_route` -> `route_utils.set_route_checked`
- and reports:

* destination violations (audit C1) - must be zero;
* route errors and rejected candidates (contract section 5 requires them reported);
* SUMO's own "Invalid route replacement" warnings;
* whether two processes with the same seed and `PYTHONHASHSEED` make byte-identical
  route decisions (audit C13).

`--route-mode truncated` reproduces the pre-fix behaviour on purpose, so the effect of C1
on travel time and distance can be measured instead of argued.

Usage:
    python scripts/13_verify_route_validity.py                       # full check
    python scripts/13_verify_route_validity.py --algo aco --duration 900
    python scripts/13_verify_route_validity.py --algo aco --route-mode truncated
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import traci  # noqa: E402

from algorithms import aco, aco_pso, astar, dijkstra, e3  # noqa: E402
from algorithms.route_utils import (  # noqa: E402
    RouteValidityError,
    build_destination_valid_route,
    get_outgoing_edges,
    get_route_stats,
    passenger_edge_pool,
    remaining_route,
    reset_route_stats,
    seed_route_choice,
    set_blocked_edges,
    set_route_checked,
)
from controller.reroute_logic import should_reroute  # noqa: E402
from emergency_manager import get_emergency_manager  # noqa: E402

ALGORITHMS = ("baseline", "dijkstra", "astar", "aco", "aco_pso", "e3")
DEFAULT_DURATION = 1200
OUTPUT_DIR = REPO_ROOT / "results" / "route_validity"


def select_route(algo_name: str, veh_id: str, current_edge: str):
    if algo_name == "aco":
        return aco.select_route(veh_id, current_edge)
    if algo_name == "aco_pso":
        return aco_pso.select_route(veh_id, current_edge)
    if algo_name == "dijkstra":
        return dijkstra.select_route(veh_id, current_edge)
    if algo_name == "astar":
        return astar.select_route(veh_id, current_edge)
    if algo_name == "e3":
        return e3.select_route(veh_id, current_edge)
    return None


def truncated_route(veh_id: str, current_edge: str, algo_name: str):
    """Reproduce the pre-fix behaviour of audit C1 for measurement purposes only.

    ACO and ACO-PSO used to hand SUMO `current_route[:idx + 1] + [selected_edge]`, which
    changes the vehicle's destination to the end of the chosen edge.
    """
    import random

    module = aco_pso if algo_name == "aco_pso" else aco
    if not module.pheromone:
        module.initialize_pheromone()
    outgoing = get_outgoing_edges(current_edge)
    if not outgoing:
        return None
    weights = []
    for edge in outgoing:
        tau = module.pheromone.get(edge, 1.0) ** module.ALPHA
        eta = module.heuristic(edge) ** module.BETA
        weights.append(max(tau * eta, 1e-9))
    selected = random.choices(outgoing, weights)[0]
    route = list(traci.vehicle.getRoute(veh_id))
    if current_edge not in route:
        return None
    index = route.index(current_edge)
    return route[: index + 1] + [selected]


def run_algorithm(
    algo_name: str,
    duration: int,
    route_mode: str,
    decision_log_path: Path | None,
    sumo_cfg: str,
) -> dict:
    """Run one algorithm and return its route-validity report."""
    seed = seed_route_choice()
    reset_route_stats()

    # SUMO's --message-log does NOT contain warnings; they go to --error-log. Counting
    # "Invalid route replacement" in the message log silently reported zero every time.
    message_log = OUTPUT_DIR / f"sumo_messages_{algo_name}_{route_mode}.log"
    error_log = OUTPUT_DIR / f"sumo_errors_{algo_name}_{route_mode}.log"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    manager = get_emergency_manager()
    manager.load_canonical_scenario_file()
    manager.generate_emergencies()

    traci.start(
        [
            "sumo",
            "-c",
            sumo_cfg,
            "--no-step-log",
            "true",
            "--end",
            str(duration),
            "--message-log",
            str(message_log),
            "--error-log",
            str(error_log),
            "--tripinfo-output",
            str(OUTPUT_DIR / f"tripinfo_{algo_name}_{route_mode}.xml"),
            "--summary-output",
            str(OUTPUT_DIR / f"summary_{algo_name}_{route_mode}.xml"),
        ]
    )

    # Building the candidate pool needs no SUMO connection, but do it now so its cost is
    # not attributed to the first rerouting vehicle.
    pool_size = len(passenger_edge_pool())
    if algo_name == "e3":
        e3.initialize()

    decisions: list[str] = []
    applied_decisions = 0
    violation_message = None
    started: dict[str, float] = {}
    finished: dict[str, float] = {}
    distances: dict[str, float] = {}
    destination_of: dict[str, str] = {}
    arrived_at_declared_destination = 0
    arrived_elsewhere = 0

    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            manager.update(sim_time)
            set_blocked_edges(manager.blocked_edges)

            vehicle_ids = traci.vehicle.getIDList()
            for veh_id in vehicle_ids:
                if veh_id not in started:
                    started[veh_id] = sim_time
                    try:
                        destination_of[veh_id] = list(traci.vehicle.getRoute(veh_id))[-1]
                    except (traci.TraCIException, IndexError):
                        destination_of[veh_id] = ""
                try:
                    distances[veh_id] = traci.vehicle.getDistance(veh_id)
                except traci.TraCIException:
                    pass

            for veh_id in traci.simulation.getArrivedIDList():
                finished[veh_id] = sim_time

            if algo_name == "baseline":
                continue

            for veh_id in vehicle_ids:
                try:
                    if not should_reroute(veh_id):
                        continue
                    current_edge = traci.vehicle.getRoadID(veh_id)
                    if not current_edge or current_edge.startswith(":"):
                        continue
                    old_route = list(traci.vehicle.getRoute(veh_id))
                    if len(old_route) < 2:
                        continue

                    if route_mode == "truncated":
                        new_route = truncated_route(veh_id, current_edge, algo_name)
                        applied = False
                        if new_route and new_route != old_route:
                            try:
                                traci.vehicle.setRoute(veh_id, new_route)
                                applied = True
                            except traci.TraCIException:
                                applied = False
                    else:
                        new_route = select_route(algo_name, veh_id, current_edge)
                        applied = set_route_checked(veh_id, new_route, old_route)

                    if applied:
                        applied_decisions += 1
                    if new_route:
                        digest = hashlib.sha256(
                            ",".join(new_route).encode("utf-8")
                        ).hexdigest()[:16]
                        decisions.append(
                            f"{sim_time:.0f}\t{veh_id}\t{int(applied)}\t"
                            f"{len(new_route)}\t{new_route[0]}\t{new_route[-1]}\t{digest}"
                        )
                except RouteValidityError as error:
                    violation_message = str(error)
                    raise
                except traci.TraCIException:
                    continue
    finally:
        try:
            traci.close()
        except Exception:
            pass

    # A completed trip that ends somewhere other than its declared destination is the
    # signature of the truncated-route bug.
    completed_distances = []
    completed_durations = []
    for veh_id, end_time in finished.items():
        completed_durations.append(end_time - started.get(veh_id, end_time))
        completed_distances.append(distances.get(veh_id, 0.0))

    warnings = 0
    if error_log.exists():
        text = error_log.read_text(encoding="utf-8", errors="replace")
        warnings = text.count("Invalid route replacement")
        arrived_elsewhere = text.count("did not reach its destination")

    stats = get_route_stats()
    report = {
        "algorithm": algo_name,
        "route_mode": route_mode,
        "duration_seconds": duration,
        "route_choice_seed": seed,
        "pythonhashseed": os.environ.get("PYTHONHASHSEED", "unset"),
        "passenger_candidate_pool_size": pool_size,
        "route_stats": stats,
        "decisions_logged": len(decisions),
        "decisions_applied": applied_decisions,
        "sumo_invalid_route_replacement_warnings": warnings,
        "vehicles_seen": len(started),
        "vehicles_completed": len(finished),
        "mean_completed_travel_time_s": (
            round(sum(completed_durations) / len(completed_durations), 2)
            if completed_durations
            else None
        ),
        "mean_completed_distance_m": (
            round(sum(completed_distances) / len(completed_distances), 2)
            if completed_distances
            else None
        ),
        "destination_violation_message": violation_message,
        "decision_log_sha256": hashlib.sha256(
            "\n".join(decisions).encode("utf-8")
        ).hexdigest(),
    }
    # SUMO's "Invalid route replacement" warning is not by itself a failure. Measured: once
    # an incident closes a road, every vehicle that had already driven over it warns on its
    # next reroute, because SUMO revalidates the whole edge list including the edges the
    # vehicle has already left behind. Those replacements still succeed. What must be zero
    # is a route that changes the destination, a replacement SUMO quietly kept out, or a
    # route sent through a currently closed road.
    report["warning_note"] = (
        "SUMO warnings can refer to the vehicle's already-driven history after an incident "
        "closes a road it had used; they are reported, not treated as failures."
    )
    # `routes_crossing_closed_edge` is deliberately not a failure: it counts routes the
    # guard refused *before* they reached SUMO, which is the guard working. What must be
    # zero is a route that changes the destination, or a replacement SUMO quietly discarded.
    report["passed"] = (
        stats["destination_violations"] == 0
        and stats["setroute_silent_rejections"] == 0
        and violation_message is None
    )

    if decision_log_path:
        decision_log_path.parent.mkdir(parents=True, exist_ok=True)
        decision_log_path.write_text("\n".join(decisions), encoding="utf-8")

    return report


def run_in_subprocess(algo_name: str, duration: int, log_path: Path, hash_seed: str) -> dict:
    """Run one algorithm in a fresh process, so PYTHONHASHSEED actually applies."""
    environment = dict(os.environ)
    environment["PYTHONHASHSEED"] = hash_seed
    report_path = log_path.with_suffix(".json")
    if report_path.exists():
        report_path.unlink()
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--algo",
        algo_name,
        "--duration",
        str(duration),
        "--decision-log",
        str(log_path),
        "--report",
        str(report_path),
    ]
    completed = subprocess.run(
        command, cwd=str(REPO_ROOT), env=environment, capture_output=True, text=True
    )
    # A failing check must not abort the sweep: the point is to report every algorithm.
    # The run still writes its report, and a non-zero exit only means `passed` is false.
    if report_path.exists():
        return json.loads(report_path.read_text(encoding="utf-8"))
    raise RuntimeError(
        f"{algo_name} run produced no report (exit {completed.returncode}):\n"
        f"{completed.stdout[-2000:]}\n{completed.stderr[-2000:]}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--algo", choices=ALGORITHMS)
    parser.add_argument("--duration", type=int, default=DEFAULT_DURATION)
    parser.add_argument("--route-mode", choices=("destination_valid", "truncated"), default="destination_valid")
    parser.add_argument("--decision-log", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--sumo-cfg", default="simulation.sumocfg")
    parser.add_argument(
        "--determinism-algo",
        default="aco",
        help="Algorithm used for the two-process determinism check in the full sweep.",
    )
    parser.add_argument("--skip-determinism", action="store_true")
    arguments = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if arguments.algo:
        report = run_algorithm(
            arguments.algo,
            arguments.duration,
            arguments.route_mode,
            arguments.decision_log,
            arguments.sumo_cfg,
        )
        text = json.dumps(report, indent=2)
        if arguments.report:
            arguments.report.write_text(text, encoding="utf-8")
        print(text)
        return 0 if report["passed"] else 1

    # Full sweep: every algorithm in its own process, then the determinism pair.
    results = {}
    for algo_name in ALGORITHMS:
        print(f"[13] running {algo_name} for {arguments.duration}s ...", flush=True)
        log_path = OUTPUT_DIR / f"decisions_{algo_name}.tsv"
        results[algo_name] = run_in_subprocess(algo_name, arguments.duration, log_path, "0")
        stats = results[algo_name]["route_stats"]
        print(
            f"     violations={stats['destination_violations']} "
            f"applied={results[algo_name]['decisions_applied']} "
            f"route_build_failures={stats['route_build_failures']} "
            f"setroute_errors={stats['setroute_errors']} "
            f"silent={stats['setroute_silent_rejections']} "
            f"closed_road_routes={stats['routes_crossing_closed_edge']} "
            f"sumo_warnings={results[algo_name]['sumo_invalid_route_replacement_warnings']}",
            flush=True,
        )

    determinism = None
    if not arguments.skip_determinism:
        algo_name = arguments.determinism_algo
        print(f"[13] determinism check: {algo_name} twice with PYTHONHASHSEED=0 ...", flush=True)
        first = run_in_subprocess(
            algo_name, arguments.duration, OUTPUT_DIR / f"determinism_{algo_name}_a.tsv", "0"
        )
        second = run_in_subprocess(
            algo_name, arguments.duration, OUTPUT_DIR / f"determinism_{algo_name}_b.tsv", "0"
        )
        determinism = {
            "algorithm": algo_name,
            "run_a_sha256": first["decision_log_sha256"],
            "run_b_sha256": second["decision_log_sha256"],
            "decisions": first["decisions_logged"],
            "identical": first["decision_log_sha256"] == second["decision_log_sha256"],
        }
        print(f"     identical={determinism['identical']} decisions={determinism['decisions']}", flush=True)

    def total(key):
        return sum(report["route_stats"][key] for report in results.values())

    total_violations = total("destination_violations")
    summary = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": arguments.duration,
        "per_algorithm": results,
        "determinism": determinism,
        "routes_refused_for_crossing_a_closed_road": total("routes_crossing_closed_edge"),
        "checks": {
            "no_destination_violations": total_violations == 0,
            "no_silent_route_rejections": total("setroute_silent_rejections") == 0,
            "every_algorithm_passed": all(report["passed"] for report in results.values()),
            "route_decisions_reproducible": determinism["identical"] if determinism else None,
        },
    }
    summary["passed"] = all(
        value for value in summary["checks"].values() if value is not None
    )
    output_path = OUTPUT_DIR / "route_validity_report.json"
    output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[13] report written to {output_path}")
    print(f"[13] passed={summary['passed']}")
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
