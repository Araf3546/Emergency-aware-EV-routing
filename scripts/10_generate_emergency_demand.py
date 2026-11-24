#!/usr/bin/env python3
"""Generate the seeded ambient emergency-vehicle demand for the Porto baseline.

Per config/experiment_contract.md section 11 (decision 13.1) the thesis models two
distinct emergency populations:

  1. ambient emergency-class vehicles (6% of the 4,000-vehicle demand = 240), which
     drive ordinary trips and broadcast emergency priority through the protocol; and
  2. the 3 scripted incident-dispatch vehicles defined in
     scenarios/emergency_scenarios.yaml, which are already routed and validated in
     network/validation/emergency_dispatch_validation.rou.xml.

This script generates population (1). It does not touch population (2).

vType ids are deliberately prefixed with "emergency_" so they cannot collide with the
"ambulance"/"fire_truck"/"police" vTypes declared by the dispatch route file when SUMO
loads both route files together.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import random
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET


WINDOW_START_SECONDS = 14 * 3600
WINDOW_END_SECONDS = 16 * 3600

# vClass="emergency" is what exempts these vehicles from an incident closure
# (see emergency_manager.CLOSURE_DISALLOWED_CLASSES).
EMERGENCY_VTYPES = [
    {"id": "emergency_ambulance", "guiShape": "emergency", "color": "1,1,1"},
    {"id": "emergency_fire_truck", "guiShape": "firebrigade", "color": "1,0,0"},
    {"id": "emergency_police", "guiShape": "police", "color": "0,0,1"},
    {"id": "emergency_relief_vehicle", "guiShape": "truck", "color": "1,0.5,0"},
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def departure_times(path: Path, tag: str) -> list[float]:
    return [float(element.attrib["depart"]) for element in ET.parse(path).getroot().findall(tag)]


def is_non_decreasing(values: list[float]) -> bool:
    return all(values[index] <= values[index + 1] for index in range(len(values) - 1))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--od-pairs", type=Path, default=Path("demand/od_pairs.csv"))
    parser.add_argument("--network", type=Path, default=Path("network/porto.net.xml"))
    parser.add_argument("--trips-output", type=Path, default=Path("demand/porto.emergency.trips.xml"))
    parser.add_argument("--output", type=Path, default=Path("demand/porto.emergency.rou.xml"))
    parser.add_argument("--report", type=Path, default=Path("demand/emergency_demand_generation.json"))
    parser.add_argument("--error-log", type=Path, default=Path("demand/emergency_demand_generation_errors.log"))
    parser.add_argument("--duarouter", type=Path, default=Path(r"C:\Program Files (x86)\Eclipse\Sumo\bin\duarouter.exe"))
    parser.add_argument("--seed", type=int, default=20260723)
    parser.add_argument("--route-seed", type=int, default=20260724)
    # 6% of the 4,000-vehicle demand fixed by experiment_contract.md section 11,
    # decision 13.8. Was 480 when the target demand was 8,000.
    parser.add_argument("--emergency-vehicles", type=int, default=240)
    parser.add_argument("--simulation-duration", type=int, default=3600)
    parser.add_argument("--max-speed", type=float, default=16.67, help="m/s; 60 km/h vs 50 km/h for normal traffic")
    parser.add_argument("--speed-factor", type=float, default=1.2)
    arguments = parser.parse_args()

    for path in (arguments.od_pairs, arguments.network, arguments.duarouter):
        if not path.is_file():
            parser.error(f"Required file not found: {path}")
    if arguments.emergency_vehicles < 1:
        parser.error("Emergency vehicle count must be positive")

    with arguments.od_pairs.open(newline="", encoding="utf-8") as handle:
        candidates = [
            row for row in csv.DictReader(handle)
            if WINDOW_START_SECONDS <= int(row["departure_second_of_day_local"]) < WINDOW_END_SECONDS
        ]
    if not candidates:
        raise RuntimeError("No mapped OD pairs exist in the 14:00-16:00 calibration window")

    rng = random.Random(arguments.seed)
    selected = rng.choices(candidates, k=arguments.emergency_vehicles)

    scheduled = []
    for index, pair in enumerate(selected):
        observed_seconds = int(pair["departure_second_of_day_local"])
        fraction = (observed_seconds - WINDOW_START_SECONDS) / (WINDOW_END_SECONDS - WINDOW_START_SECONDS)
        vtype = EMERGENCY_VTYPES[index % len(EMERGENCY_VTYPES)]["id"]
        scheduled.append((fraction * arguments.simulation_duration, pair, vtype))
    scheduled.sort(key=lambda item: item[0])

    root = ET.Element("routes")
    for vtype in EMERGENCY_VTYPES:
        ET.SubElement(
            root, "vType", id=vtype["id"], vClass="emergency", guiShape=vtype["guiShape"],
            color=vtype["color"], accel="2.9", decel="5.0", sigma="0.5", length="6.5",
            minGap="2.5", maxSpeed=f"{arguments.max_speed}", speedFactor=f"{arguments.speed_factor}",
        )
    for index, (departure, pair, vtype) in enumerate(scheduled):
        ET.SubElement(
            root, "trip", id=f"emergency_{index:05d}", depart=f"{departure:.3f}", type=vtype,
            **{"from": pair["origin_edge"], "to": pair["destination_edge"]},
        )
    arguments.trips_output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(arguments.trips_output, encoding="utf-8", xml_declaration=True)

    command = [
        str(arguments.duarouter), "--net-file", str(arguments.network),
        "--route-files", str(arguments.trips_output), "--output-file", str(arguments.output),
        "--seed", str(arguments.route_seed), "--remove-loops", "true",
        "--unsorted-input", "true", "--error-log", str(arguments.error_log),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)

    routed_ids = set()
    if arguments.output.is_file():
        routed_ids = {element.attrib["id"] for element in ET.parse(arguments.output).getroot().findall("vehicle")}
    trip_ids = {element.attrib["id"] for element in ET.parse(arguments.trips_output).getroot().findall("trip")}

    combined_output = f"{completed.stdout}\n{completed.stderr}"
    sorting_warning_detected = "sorted by departure time" in combined_output.lower()
    output_departures = departure_times(arguments.output, "vehicle") if arguments.output.is_file() else []
    departures_non_decreasing = is_non_decreasing(output_departures)

    type_counts: dict[str, int] = {}
    if arguments.output.is_file():
        for element in ET.parse(arguments.output).getroot().findall("vehicle"):
            type_counts[element.attrib.get("type", "")] = type_counts.get(element.attrib.get("type", ""), 0) + 1

    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "Ambient emergency-class demand (contract section 11, decision 13.1 population 1).",
        "inputs": {
            "od_pairs": str(arguments.od_pairs), "od_pairs_sha256": sha256(arguments.od_pairs),
            "network": str(arguments.network), "network_sha256": sha256(arguments.network),
            "demand_seed": arguments.seed, "route_seed": arguments.route_seed,
            "emergency_vehicles": arguments.emergency_vehicles,
            "simulation_duration_seconds": arguments.simulation_duration,
            "max_speed_mps": arguments.max_speed, "speed_factor": arguments.speed_factor,
        },
        "artifacts": {
            "trips_file": str(arguments.trips_output), "trips_file_sha256": sha256(arguments.trips_output),
            "route_file": str(arguments.output) if arguments.output.is_file() else None,
            "route_file_sha256": sha256(arguments.output) if arguments.output.is_file() else None,
            "error_log": str(arguments.error_log),
        },
        "command": command,
        "duarouter_exit_code": completed.returncode,
        "trip_count": len(trip_ids),
        "routed_vehicle_count": len(routed_ids),
        "all_trips_routed": len(routed_ids) == len(trip_ids),
        "rejected_trip_ids": sorted(trip_ids - routed_ids),
        "vehicles_per_type": type_counts,
        "sorting_warning_detected": sorting_warning_detected,
        "departures_non_decreasing": departures_non_decreasing,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    arguments.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {arguments.output}: {len(routed_ids)}/{len(trip_ids)} emergency trips routed")
    for type_id, count in sorted(type_counts.items()):
        print(f"  {type_id}: {count}")
    if sorting_warning_detected:
        print("FAILED: duarouter reported unsorted departures despite --unsorted-input.")
    if not departures_non_decreasing:
        print("FAILED: emergency route file is not sorted by departure time.")

    success = (
        completed.returncode == 0
        and len(routed_ids) == len(trip_ids)
        and not sorting_warning_detected
        and departures_non_decreasing
    )
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
