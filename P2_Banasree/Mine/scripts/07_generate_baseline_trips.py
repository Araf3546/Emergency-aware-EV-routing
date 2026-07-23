#!/usr/bin/env python3
"""Generate the seeded normal-traffic Porto baseline trip file.

The final emergency vehicles are deliberately excluded here and will be added
only by the later seeded emergency-scenario generator.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import random
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET


WINDOW_START_SECONDS = 14 * 3600
WINDOW_END_SECONDS = 16 * 3600


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--od-pairs", type=Path, default=Path("demand/od_pairs.csv"))
    parser.add_argument("--output", type=Path, default=Path("demand/porto.trips.xml"))
    parser.add_argument("--report", type=Path, default=Path("demand/baseline_trip_generation.json"))
    parser.add_argument("--seed", type=int, default=20260721)
    parser.add_argument("--normal-vehicles", type=int, default=7520)
    parser.add_argument("--simulation-duration", type=int, default=3600)
    arguments = parser.parse_args()
    if not arguments.od_pairs.is_file():
        parser.error(f"Mapped OD file not found: {arguments.od_pairs}")
    if arguments.normal_vehicles < 1 or arguments.simulation_duration < 1:
        parser.error("Vehicle count and simulation duration must be positive")

    with arguments.od_pairs.open(newline="", encoding="utf-8") as handle:
        candidates = [
            row for row in csv.DictReader(handle)
            if WINDOW_START_SECONDS <= int(row["departure_second_of_day_local"]) < WINDOW_END_SECONDS
        ]
    if not candidates:
        raise RuntimeError("No mapped OD pairs exist in the 14:00-16:00 calibration window")

    selected = random.Random(arguments.seed).choices(candidates, k=arguments.normal_vehicles)
    scheduled = []
    for pair in selected:
        observed_seconds = int(pair["departure_second_of_day_local"])
        departure = (observed_seconds - WINDOW_START_SECONDS) / (WINDOW_END_SECONDS - WINDOW_START_SECONDS)
        scheduled.append((departure * arguments.simulation_duration, pair))
    scheduled.sort(key=lambda item: item[0])
    root = ET.Element("routes")
    ET.SubElement(
        root, "vType", id="normal_passenger", vClass="passenger", accel="2.6", decel="4.5",
        sigma="0.5", length="5.0", minGap="2.5", maxSpeed="13.89",
    )
    for index, (departure, pair) in enumerate(scheduled):
        ET.SubElement(
            root, "trip", id=f"normal_{index:05d}", depart=f"{departure:.3f}", type="normal_passenger",
            **{"from": pair["origin_edge"], "to": pair["destination_edge"]},
        )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(arguments.output, encoding="utf-8", xml_declaration=True)
    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "Normal-traffic baseline demand; emergency vehicles excluded pending validated emergency scenarios.",
        "inputs": {
            "od_pairs": str(arguments.od_pairs), "od_pairs_sha256": sha256(arguments.od_pairs),
            "seed": arguments.seed, "normal_vehicles": arguments.normal_vehicles,
            "simulation_duration_seconds": arguments.simulation_duration,
            "source_window_local": "14:00-16:00", "source_window_seconds": 7200,
            "departure_time_mapping": "linear mapping from source window to simulation duration",
            "sampling": "seeded sampling with replacement from mapped OD pairs in the source window",
        },
        "output": {"trip_file": str(arguments.output), "trip_file_sha256": sha256(arguments.output)},
        "available_mapped_pairs_in_window": len(candidates),
    }
    arguments.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {arguments.output} with {arguments.normal_vehicles} normal trips")


if __name__ == "__main__":
    main()
