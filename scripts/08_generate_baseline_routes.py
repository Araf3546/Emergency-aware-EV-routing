#!/usr/bin/env python3
"""Generate destination-valid routes for the seeded normal-traffic baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def vehicle_count(path: Path) -> int:
    return len(ET.parse(path).getroot().findall("vehicle"))


def departure_times(path: Path) -> list[float]:
    return [float(element.attrib["depart"]) for element in ET.parse(path).getroot().findall("vehicle")]


def is_non_decreasing(values: list[float]) -> bool:
    return all(values[index] <= values[index + 1] for index in range(len(values) - 1))


def first_out_of_order_index(values: list[float]) -> int | None:
    for index in range(len(values) - 1):
        if values[index] > values[index + 1]:
            return index
    return None


def load_trip_metadata(path: Path) -> dict[str, dict[str, str]]:
    root = ET.parse(path).getroot()
    metadata: dict[str, dict[str, str]] = {}
    for trip in root.findall("trip"):
        trip_id = trip.attrib["id"]
        metadata[trip_id] = {
            "from": trip.attrib.get("from", ""),
            "to": trip.attrib.get("to", ""),
            "depart": trip.attrib.get("depart", ""),
        }
    return metadata


def extract_rejection_reasons(log_text: str, failed_ids: set[str]) -> dict[str, str]:
    if not log_text or not failed_ids:
        return {}
    keyword_pattern = re.compile(r"error|warning|could not|cannot|no connection|failed|not build", re.IGNORECASE)
    reasons: dict[str, str] = {}
    lines = [line.strip() for line in log_text.splitlines() if line.strip()]
    for trip_id in failed_ids:
        for line in lines:
            if trip_id in line and keyword_pattern.search(line):
                reasons[trip_id] = line
                break
    return reasons


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=Path("network/porto.net.xml"))
    parser.add_argument("--trips", type=Path, default=Path("demand/porto.trips.xml"))
    parser.add_argument("--output", type=Path, default=Path("demand/porto.rou.xml"))
    parser.add_argument("--report", type=Path, default=Path("demand/baseline_route_generation.json"))
    parser.add_argument("--error-log", type=Path, default=Path("demand/baseline_route_generation_errors.log"))
    parser.add_argument("--duarouter", type=Path, default=Path(r"C:\Program Files (x86)\Eclipse\Sumo\bin\duarouter.exe"))
    parser.add_argument("--seed", type=int, default=20260722)
    arguments = parser.parse_args()
    for path in (arguments.network, arguments.trips, arguments.duarouter):
        if not path.is_file():
            parser.error(f"Required file not found: {path}")

    command = [
        str(arguments.duarouter), "--net-file", str(arguments.network), "--route-files", str(arguments.trips),
        "--output-file", str(arguments.output), "--seed", str(arguments.seed), "--remove-loops", "true",
        "--unsorted-input", "true", "--error-log", str(arguments.error_log),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    route_count = vehicle_count(arguments.output) if arguments.output.is_file() else 0
    combined_process_output = f"{completed.stdout}\n{completed.stderr}"
    sorting_warning_detected = "sorted by departure time" in combined_process_output.lower()
    output_departures = departure_times(arguments.output) if arguments.output.is_file() else []
    departures_non_decreasing = is_non_decreasing(output_departures)
    first_out_of_order = None if departures_non_decreasing else first_out_of_order_index(output_departures)
    trip_metadata = load_trip_metadata(arguments.trips)
    trip_count = len(trip_metadata)
    successful_ids = {element.attrib["id"] for element in ET.parse(arguments.output).getroot().findall("vehicle")} if arguments.output.is_file() else set()
    failed_ids = sorted(set(trip_metadata) - successful_ids)
    rejection_reasons = extract_rejection_reasons(completed.stdout + "\n" + completed.stderr, set(failed_ids))
    rejected_trips = [
        {
            "trip_id": trip_id,
            "origin_edge": trip_metadata[trip_id]["from"],
            "destination_edge": trip_metadata[trip_id]["to"],
            "depart": trip_metadata[trip_id]["depart"],
            "reason": rejection_reasons.get(trip_id, "Route was not present in the routed output file."),
            "reason_preserved": trip_id in rejection_reasons,
        }
        for trip_id in failed_ids
    ]
    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "network": str(arguments.network), "network_sha256": sha256(arguments.network),
            "trips": str(arguments.trips), "trips_sha256": sha256(arguments.trips), "seed": arguments.seed,
        },
        "artifacts": {
            "output": str(arguments.output) if arguments.output.is_file() else None,
            "output_sha256": sha256(arguments.output) if arguments.output.is_file() else None,
            "error_log": str(arguments.error_log),
        },
        "command": command,
        "duarouter_exit_code": completed.returncode,
        "trip_count": trip_count,
        "routed_vehicle_count": route_count,
        "all_trips_routed": route_count == trip_count,
        "rejected_trips": rejected_trips,
        "preserved_rejected_trip_reasons": all(item["reason_preserved"] for item in rejected_trips),
        "sorting_warning_detected": sorting_warning_detected,
        "departures_non_decreasing": departures_non_decreasing,
        "first_out_of_order_departure_index": first_out_of_order,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    arguments.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {arguments.output}: {route_count}/{trip_count} trips routed")
    # Contract v0.7, decision 13.13: this output carries no speed factors, and without them
    # compared arms drive different vehicle populations. tests/test_frozen_speed_factors.py
    # fails until the next step has been run.
    print("NEXT: run scripts/08b_freeze_speed_factors.py to freeze per-vehicle speed factors "
          "(contract decision 13.13).")
    if sorting_warning_detected:
        print("FAILED: duarouter emitted a 'sorted by departure time' warning despite --unsorted-input.")
    if not departures_non_decreasing:
        print(
            f"FAILED: output route file is not sorted by departure time "
            f"(first violation at vehicle index {first_out_of_order})."
        )
    success = (
        completed.returncode == 0
        and route_count == trip_count
        and not sorting_warning_detected
        and departures_non_decreasing
    )
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
