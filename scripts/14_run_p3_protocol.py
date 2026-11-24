#!/usr/bin/env python3
"""Run the P3 protocol runtime over the contracted scenario and check the Step 3 criteria.

Audit Step 3 asks for evidence that the emergency layer is real: non-zero emergency
broadcasts, non-zero emergency zone snapshots, a recorded response time for all three
incidents, and a non-empty emergency-events log whose `end_time` matches the scenario file.

This is a verification entry point, not a replacement for `controller/controller_p3.py`.
It exists because that controller loops to 14,400 steps against a 3,600-second scenario
(audit R2), which at the measured cost of the coordinator would take about 6.4 hours.
Here the run is bounded by the contracted duration in `config/scenario.yaml`.

Usage:
    python scripts/14_run_p3_protocol.py                 # full 3,600 s contracted run
    python scripts/14_run_p3_protocol.py --duration 1300 # shorter check
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import traci  # noqa: E402
import yaml  # noqa: E402

from algorithms.route_utils import get_route_stats  # noqa: E402
from metrics.emergency_metrics import build_emergency_report  # noqa: E402
from metrics.trip_metrics import build_report, load_summary_timeseries  # noqa: E402
from p3_simulation_config import EMERGENCY_VEHICLE_TYPES  # noqa: E402
from emergency_manager import get_emergency_manager  # noqa: E402
from p3_runtime import P3RuntimeCoordinator  # noqa: E402

SCENARIO_CONFIG = REPO_ROOT / "config" / "scenario.yaml"

# Set per run from --run-name, because P3MetricsLogger.save_results writes to
# results/<run_name>/ and the two must not diverge.
OUTPUT_DIR = REPO_ROOT / "results" / "p3_hybrid"


def contracted_duration() -> int:
    with open(SCENARIO_CONFIG, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    return int(config["simulation"]["duration_seconds"])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=None)
    parser.add_argument("--sumo-cfg", default="simulation.sumocfg")
    parser.add_argument("--run-name", default="p3_hybrid")
    arguments = parser.parse_args()

    duration = arguments.duration or contracted_duration()

    global OUTPUT_DIR
    OUTPUT_DIR = REPO_ROOT / "results" / arguments.run_name
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    manager = get_emergency_manager()
    scenario_file = manager.load_canonical_scenario_file()
    manager.generate_emergencies()
    print(f"[14] scenario: {scenario_file}; {len(manager.events)} incidents; duration {duration}s")

    coordinator = P3RuntimeCoordinator()

    traci.start(
        [
            "sumo",
            "-c",
            arguments.sumo_cfg,
            "--no-step-log",
            "true",
            "--end",
            str(duration),
            "--error-log",
            str(OUTPUT_DIR / "sumo_errors.log"),
            "--tripinfo-output",
            str(OUTPUT_DIR / "tripinfo.xml"),
            "--summary-output",
            str(OUTPUT_DIR / "summary.xml"),
        ]
    )

    started = time.time()
    try:
        while traci.simulation.getTime() < duration:
            step_time = traci.simulation.getTime()
            traci.simulationStep()
            step_time = traci.simulation.getTime()
            manager.update(step_time)
            coordinator.update_step(step_time)
            if int(step_time) % 300 == 0:
                elapsed = time.time() - started
                print(
                    f"[14] t={step_time:.0f}s  vehicles={len(traci.vehicle.getIDList())}  "
                    f"responses={sum(1 for x in manager.get_response_summary().values() if x.get('measured'))}"
                    f"/{len(manager.events)}  elapsed={elapsed/60:.1f} min",
                    flush=True,
                )
    finally:
        try:
            traci.close()
        except Exception:
            pass

    coordinator.save_metrics(run_name=arguments.run_name)

    # ---- Step 3 acceptance criteria -------------------------------------------------
    metrics = coordinator.metrics
    responses = manager.get_response_summary()
    # EmergencyManager.get_response_summary() returns one entry per event whether or not it
    # was answered, and names the value `response_time_s`. Filter on `measured`.
    measured = {
        event_id: state
        for event_id, state in responses.items()
        if state.get("measured") and state.get("response_time_s") is not None
    }
    scenario_windows = {
        event_id: (event.start_time, event.end_time)
        for event_id, event in manager.events.items()
    }
    events_log = OUTPUT_DIR / f"{arguments.run_name}_emergency_events.jsonl"
    logged = []
    if events_log.exists():
        logged = [
            json.loads(line)
            for line in events_log.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    end_times_match = all(
        scenario_windows.get(entry["event_id"], (None, None))[1] == entry.get("end_time")
        for entry in logged
    )

    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": duration,
        "wall_clock_minutes": round((time.time() - started) / 60, 1),
        "incidents_in_scenario": len(manager.events),
        "emergency_broadcasts": metrics.emergency_messages,
        "total_broadcasts": metrics.vehicle_broadcasts,
        "emergency_zone_snapshots": metrics.emergency_zones_observed,
        "yield_requests": metrics.yield_requests,
        "reroutes_executed": metrics.reroute_executed,
        "reroute_decisions": metrics.reroute_decisions,
        "emergency_event_responses": metrics.emergency_event_responses,
        "events_with_response_time": len(measured),
        "responders": {
            event_id: state["first_arrival_vehicle"] for event_id, state in measured.items()
        },
        "arrived_after_incident_end": {
            event_id: state["arrived_after_incident_end"] for event_id, state in measured.items()
        },
        "response_times": {
            event_id: state["response_time_s"] for event_id, state in measured.items()
        },
        "emergency_events_logged": len(logged),
        "logged_end_times_match_scenario": end_times_match,
        "route_stats": get_route_stats(),
        # Step 4: contract metrics from SUMO's own output, and the five emergency metrics
        # of audit M11.
        "trip_metrics": build_report(
            OUTPUT_DIR / "tripinfo.xml",
            OUTPUT_DIR / "summary.xml",
            emergency_types=EMERGENCY_VEHICLE_TYPES,
            applied_reroutes=metrics.reroute_executed,
        ),
        "emergency_metrics": build_emergency_report(
            events_log,
            OUTPUT_DIR / "tripinfo.xml",
            OUTPUT_DIR / "summary.xml",
            blockage_samples=coordinator.blockage_samples,
        ),
        "checks": {
            "emergency_broadcasts_non_zero": metrics.emergency_messages > 0,
            "emergency_zone_snapshots_non_zero": metrics.emergency_zones_observed > 0,
            "response_time_for_every_incident": len(measured) == len(manager.events),
            "emergency_events_log_non_empty": len(logged) > 0,
            "logged_end_times_match_scenario": end_times_match,
        },
    }
    report["passed"] = all(report["checks"].values())

    # Audit M3, M4: the congestion time series is SUMO's own per-step summary, which costs
    # nothing, instead of a 21,385-edge scan per step.
    try:
        series = load_summary_timeseries(OUTPUT_DIR / "summary.xml")
        import csv as _csv

        with open(OUTPUT_DIR / f"{arguments.run_name}_network_timeseries.csv", "w", newline="") as handle:
            writer = _csv.writer(handle)
            writer.writerow(
                ["sim_time", "running", "halting", "mean_speed_ms", "mean_waiting_time_s", "teleports"]
            )
            for row in series:
                writer.writerow(
                    [
                        row.get("time", ""),
                        row.get("running", ""),
                        row.get("halting", ""),
                        row.get("meanSpeed", ""),
                        row.get("meanWaitingTime", ""),
                        row.get("teleports", ""),
                    ]
                )
    except Exception as error:
        print(f"[14] could not write network time series: {error}")

    output_path = OUTPUT_DIR / "step3_emergency_layer_report.json"
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["checks"], indent=2))
    print(f"[14] response times: {report['response_times']}")
    print(f"[14] report written to {output_path}")
    print(f"[14] passed={report['passed']}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
