#!/usr/bin/env python3
"""Measure emergency response time for every seeded incident, over several seeds.

Response time is defined by experiment_contract.md sections 7 and 8: from incident
activation until the incident's own dispatched responder has its road ID equal to the
incident's `center_edge`. There is no requirement that the incident window still be
open - a responder that reaches the incident after `end_time` still produces a valid,
reportable response time. Detection lives in EmergencyManager.record_responder_arrivals
so this script, p3_runtime, the tuner and the two metrics runners all share it.

This is the no-rerouting baseline condition: vehicles follow their route files and the
only live intervention is the incident closure itself. Run it after any change to the
demand, the incident scenario or the dispatch file.

    python scripts/12_measure_emergency_response.py
"""

from __future__ import annotations

import argparse
import json
import shutil
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import traci  # noqa: E402

from emergency_manager import EmergencyManager  # noqa: E402

# Options in the base config whose values are output file paths. They are redirected
# into the per-seed run directory so a measurement run never overwrites the artifacts
# of the recorded baseline validation.
OUTPUT_OPTIONS = {
    "tripinfo-output": "tripinfo.xml",
    "summary-output": "summary.xml",
    "emission-output": None,       # None means "drop it": far too large to keep per seed
    "queue-output": None,
    "message-log": "sumo_messages.log",
}
INPUT_OPTIONS = ("net-file", "route-files", "additional-files")


def derive_config(base_config: Path, run_directory: Path, target: Path) -> Path:
    """Copy the base sumocfg, absolutising inputs and redirecting outputs.

    Input paths in a sumocfg resolve relative to the config file, so they are made
    absolute before the derived config is written somewhere else.
    """
    tree = ET.parse(base_config)
    root = tree.getroot()
    base_directory = base_config.resolve().parent

    for option_name in INPUT_OPTIONS:
        for element in root.iter(option_name):
            values = [value.strip() for value in element.attrib["value"].split(",") if value.strip()]
            element.set("value", ",".join(str((base_directory / value).resolve()) for value in values))

    parents = {child: parent for parent in root.iter() for child in parent}
    for option_name, filename in OUTPUT_OPTIONS.items():
        for element in list(root.iter(option_name)):
            if filename is None:
                parents[element].remove(element)
            else:
                element.set("value", str((run_directory / filename).resolve()))

    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target


def run_one_seed(config: Path, scenario: Path, seed: int, end_time: float,
                 run_directory: Path) -> dict:
    """Run the scenario once and return the per-event response report."""
    manager = EmergencyManager()
    manager.load_planned_events(scenario)
    manager.generate_emergencies()

    dispatch_ids = {
        event.dispatch_vehicle_id: event_id
        for event_id, event in manager.events.items()
        if event.dispatch_vehicle_id
    }
    dispatch_entered: dict[str, float] = {}
    dispatch_left: dict[str, float] = {}

    traci.start([
        "sumo", "-c", str(config), "--seed", str(seed),
        "--end", str(int(end_time)), "--no-step-log", "true", "--no-warnings", "true",
    ])
    try:
        while traci.simulation.getTime() < end_time:
            traci.simulationStep()
            now = float(traci.simulation.getTime())
            manager.update(now)

            running_ids = traci.vehicle.getIDList()
            arrived_ids = traci.simulation.getArrivedIDList()
            manager.record_responder_arrivals(now, vehicle_ids=running_ids, arrived_ids=arrived_ids)

            for vehicle_id in running_ids:
                if vehicle_id in dispatch_ids:
                    dispatch_entered.setdefault(vehicle_id, now)
            for vehicle_id in arrived_ids:
                if vehicle_id in dispatch_ids:
                    dispatch_left.setdefault(vehicle_id, now)
    finally:
        try:
            traci.close()
        except Exception:
            pass

    summary = manager.get_response_summary()
    for event_state in summary.values():
        vehicle_id = event_state["dispatch_vehicle_id"]
        event_state["responder_entered_network_at"] = dispatch_entered.get(vehicle_id)
        event_state["responder_completed_route_at"] = dispatch_left.get(vehicle_id)
    return {
        "seed": seed,
        "run_directory": str(run_directory),
        "incident_errors": list(manager.incident_errors),
        "events": summary,
    }


def aggregate(runs: list[dict]) -> dict:
    """Mean, sample standard deviation and range of response time, per event."""
    aggregated = {}
    event_ids = sorted({event_id for run in runs for event_id in run["events"]}, key=str)
    for event_id in event_ids:
        states = [run["events"].get(event_id) for run in runs]
        values = [state["response_time_s"] for state in states
                  if state and state["response_time_s"] is not None]
        reference = next(state for state in states if state)
        aggregated[str(event_id)] = {
            "event_id": event_id,
            "start_time": reference["start_time"],
            "end_time": reference["end_time"],
            "center_edge": reference["center_edge"],
            "dispatch_vehicle_id": reference["dispatch_vehicle_id"],
            "seeds_measured": len(values),
            "seeds_run": len(states),
            "response_times_s": values,
            "mean_s": round(statistics.mean(values), 1) if values else None,
            "stdev_s": round(statistics.stdev(values), 1) if len(values) > 1 else None,
            "min_s": round(min(values), 1) if values else None,
            "max_s": round(max(values), 1) if values else None,
            "arrived_after_incident_end_in_seeds": [
                run["seed"] for run, state in zip(runs, states)
                if state and state["arrived_after_incident_end"]
            ],
        }
    return aggregated


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "simulation.sumocfg")
    parser.add_argument("--scenario", type=Path,
                        default=PROJECT_ROOT / "scenarios" / "emergency_scenarios.yaml")
    parser.add_argument("--seeds", type=int, nargs="+", default=[20260722, 20260723, 20260724])
    parser.add_argument("--end", type=float, default=3600.0)
    parser.add_argument("--output-dir", type=Path,
                        default=PROJECT_ROOT / "results" / "emergency_response")
    parser.add_argument("--keep-run-artifacts", action="store_true",
                        help="Keep each seed's tripinfo/summary instead of deleting them.")
    arguments = parser.parse_args()

    for path in (arguments.config, arguments.scenario):
        if not path.is_file():
            parser.error(f"Required file not found: {path}")

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    runs = []
    for seed in arguments.seeds:
        run_directory = arguments.output_dir / f"seed_{seed}"
        run_directory.mkdir(parents=True, exist_ok=True)
        config = derive_config(arguments.config, run_directory, run_directory / "run.sumocfg")
        print(f"--- seed {seed} ---")
        runs.append(run_one_seed(config, arguments.scenario, seed, arguments.end, run_directory))
        if not arguments.keep_run_artifacts:
            shutil.rmtree(run_directory, ignore_errors=True)

    aggregated = aggregate(runs)
    all_measured = all(
        entry["seeds_measured"] == entry["seeds_run"] for entry in aggregated.values()
    )

    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "definition": (
            "Response time = incident start_time to the moment the incident's own "
            "dispatch vehicle's road ID equals the incident's center_edge. Contract "
            "sections 7 and 8. Arrival after the incident end_time is still a valid "
            "measurement and is flagged, never discarded."
        ),
        "config": str(arguments.config),
        "scenario": str(arguments.scenario),
        "seeds": arguments.seeds,
        "simulation_end_s": arguments.end,
        "all_events_measured_in_all_seeds": all_measured,
        "per_event": aggregated,
        "per_seed": runs,
    }
    report_path = arguments.output_dir / "emergency_response_times.json"
    report_path.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")

    print()
    print(f"{'event':>6} {'window':>12} {'measured':>9} {'mean':>8} {'stdev':>7} "
          f"{'min':>7} {'max':>7}  after_end_in_seeds")
    for entry in aggregated.values():
        window = f"{entry['start_time']:.0f}-{entry['end_time']:.0f}"
        print(f"{str(entry['event_id']):>6} {window:>12} "
              f"{entry['seeds_measured']}/{entry['seeds_run']:<7} "
              f"{str(entry['mean_s']):>8} {str(entry['stdev_s']):>7} "
              f"{str(entry['min_s']):>7} {str(entry['max_s']):>7}  "
              f"{entry['arrived_after_incident_end_in_seeds']}")
    print()
    print(f"Wrote {report_path}")
    if not all_measured:
        print("FAILED: at least one incident produced no response time.")
    return 0 if all_measured else 2


if __name__ == "__main__":
    raise SystemExit(main())
