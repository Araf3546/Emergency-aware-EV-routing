#!/usr/bin/env python3
"""Check that reported metrics agree with SUMO's own output for the same run.

This is Step 4's acceptance check: "metric values from the CSV agree with `tripinfo.xml`
for the same run, to the last decimal."

It re-reads `tripinfo.xml` and `summary.xml` independently of the code that produced the
CSV, recomputes every column and every aggregate, and compares. It also measures the size
of the waiting-time defect (audit M1) by accumulating waiting time the old way during a
live run and comparing it with what SUMO recorded for the same vehicles.

Usage:
    python scripts/15_verify_metrics.py --algo baseline           # check an existing run
    python scripts/15_verify_metrics.py --algo baseline --run     # run it first, then check
    python scripts/15_verify_metrics.py --waiting-time-demo 600   # measure the M1 overcount
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import traci  # noqa: E402

from metrics.trip_metrics import aggregate, load_summary, load_tripinfo  # noqa: E402
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES  # noqa: E402

OUTPUT_DIR = REPO_ROOT / "results" / "metric_validation"

# CSV column -> (tripinfo field, transform)
COLUMN_CHECKS = {
    "TravelTime": "scheduled_travel_time",
    "InNetworkDuration": "duration",
    "DepartDelay": "departDelay",
    "WaitingTime": "waitingTime",
    "TimeLoss": "timeLoss",
    "Distance": "routeLength",
}


def check_csv_against_tripinfo(run_dir: Path, algo_name: str) -> dict:
    """Compare every per-vehicle CSV cell with the value SUMO recorded."""
    tripinfo_path = run_dir / "tripinfo.xml"
    csv_path = run_dir / f"{algo_name}_results.csv"
    trips = load_tripinfo(tripinfo_path)

    mismatches = []
    rows = 0
    with open(csv_path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows += 1
            veh = row["VehicleID"]
            record = trips.get(veh)
            if record is None:
                mismatches.append({"vehicle": veh, "field": "*", "reason": "not in tripinfo"})
                continue
            for column, field in COLUMN_CHECKS.items():
                reported = float(row[column])
                expected = round(float(record.get(field, 0.0)), 2)
                if abs(reported - expected) > 1e-9:
                    mismatches.append(
                        {
                            "vehicle": veh,
                            "field": column,
                            "csv": reported,
                            "tripinfo": expected,
                        }
                    )
            if row["VehicleType"] != record.get("vehicle_type", ""):
                mismatches.append({"vehicle": veh, "field": "VehicleType"})
            expected_emergency = "Yes" if record.get("vehicle_type") in EMERGENCY_VEHICLE_TYPES else "No"
            if row["IsEmergency"] != expected_emergency:
                mismatches.append({"vehicle": veh, "field": "IsEmergency"})

    return {
        "csv_rows": rows,
        "tripinfo_entries": len(trips),
        "row_count_matches": rows == len(trips),
        "cell_mismatches": len(mismatches),
        "examples": mismatches[:10],
    }


def check_summary_aggregates(run_dir: Path, algo_name: str) -> dict:
    """Recompute the aggregates independently and compare with the stored metrics JSON."""
    trips = load_tripinfo(run_dir / "tripinfo.xml")
    summary_path = run_dir / "summary.xml"
    summary = load_summary(summary_path) if summary_path.exists() else None
    recomputed = aggregate(trips, summary, EMERGENCY_VEHICLE_TYPES)

    stored_path = run_dir / f"{algo_name}_metrics.json"
    stored = json.loads(stored_path.read_text(encoding="utf-8"))["trip_metrics"]

    differences = {}
    for key, value in recomputed.items():
        if key in stored and stored[key] != value:
            differences[key] = {"stored": stored[key], "recomputed": value}
    return {
        "aggregates_compared": len(recomputed),
        "aggregate_differences": differences,
        "matches": not differences,
    }


def measure_waiting_time_overcount(duration: int, sumo_cfg: str) -> dict:
    """Quantify audit M1 by accumulating waiting time the old way during a live run.

    The old code did `total += traci.vehicle.getWaitingTime(veh)` every step. That value is
    already a running counter that resets when the vehicle moves, so summing it counts the
    same wait once per step it persists.
    """
    run_dir = OUTPUT_DIR
    run_dir.mkdir(parents=True, exist_ok=True)
    tripinfo_path = run_dir / "waiting_demo_tripinfo.xml"

    traci.start([
        "sumo", "-c", sumo_cfg,
        "--no-step-log", "true",
        "--end", str(duration),
        "--tripinfo-output", str(tripinfo_path),
    ])
    per_step_sum: dict[str, float] = {}
    try:
        while traci.simulation.getTime() < duration:
            traci.simulationStep()
            for veh in traci.vehicle.getIDList():
                try:
                    per_step_sum[veh] = per_step_sum.get(veh, 0.0) + traci.vehicle.getWaitingTime(veh)
                except traci.TraCIException:
                    continue
    finally:
        try:
            traci.close()
        except Exception:
            pass

    trips = load_tripinfo(tripinfo_path)
    compared = [
        (per_step_sum[veh], float(record.get("waitingTime", 0.0)))
        for veh, record in trips.items()
        if veh in per_step_sum
    ]
    if not compared:
        return {"vehicles_compared": 0}

    old_total = sum(old for old, _ in compared)
    true_total = sum(true for _, true in compared)
    worst = max(compared, key=lambda pair: pair[0] - pair[1])
    return {
        "vehicles_compared": len(compared),
        "old_per_step_sum_mean_s": round(old_total / len(compared), 2),
        "sumo_accumulated_waiting_mean_s": round(true_total / len(compared), 2),
        "overcount_factor": round(old_total / true_total, 2) if true_total else None,
        "worst_single_vehicle": {
            "old_method_s": round(worst[0], 2),
            "sumo_s": round(worst[1], 2),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--algo", default="baseline")
    parser.add_argument("--run", action="store_true", help="run the algorithm first")
    parser.add_argument("--duration", type=int, default=600)
    parser.add_argument("--sumo-cfg", default="simulation.sumocfg")
    parser.add_argument("--waiting-time-demo", type=int, default=0,
                        help="measure the M1 overcount over this many seconds")
    arguments = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    report: dict = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }

    if arguments.waiting_time_demo:
        report["waiting_time_overcount"] = measure_waiting_time_overcount(
            arguments.waiting_time_demo, arguments.sumo_cfg
        )
        print(json.dumps(report["waiting_time_overcount"], indent=2))

    if arguments.run:
        from metrics.metrics_logger import run_metrics

        run_metrics(arguments.algo, duration=arguments.duration, sumo_cfg=arguments.sumo_cfg)

    run_dir = REPO_ROOT / "results" / arguments.algo
    if (run_dir / "tripinfo.xml").exists():
        report["per_vehicle"] = check_csv_against_tripinfo(run_dir, arguments.algo)
        report["aggregates"] = check_summary_aggregates(run_dir, arguments.algo)
        report["checks"] = {
            "every_csv_row_matches_tripinfo": report["per_vehicle"]["cell_mismatches"] == 0,
            "csv_covers_every_completed_trip": report["per_vehicle"]["row_count_matches"],
            "stored_aggregates_reproducible": report["aggregates"]["matches"],
        }
        report["passed"] = all(report["checks"].values())
        print(json.dumps({k: report[k] for k in ("per_vehicle", "aggregates", "checks", "passed")}, indent=2))
    else:
        report["passed"] = None
        print(f"[15] no run found at {run_dir}; use --run to produce one")

    output_path = OUTPUT_DIR / f"metric_validation_{arguments.algo}.json"
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"[15] report written to {output_path}")
    return 0 if report.get("passed") in (True, None) else 1


if __name__ == "__main__":
    raise SystemExit(main())
