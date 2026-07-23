#!/usr/bin/env python3
"""Validate that a seeded sample of mapped Porto OD pairs can be routed.

This is a Gate B validation utility.  Its temporary-style validation trips are
kept under network/validation/ and must not be treated as the final scenario
demand file.
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


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_pairs(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_validation_trips(pairs: list[dict[str, str]], output: Path) -> None:
    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="validation_passenger", vClass="passenger")
    for index, pair in enumerate(pairs):
        ET.SubElement(
            root,
            "trip",
            id=f"routeability_{index:05d}",
            depart="0",
            type="validation_passenger",
            **{"from": pair["origin_edge"], "to": pair["destination_edge"]},
        )
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def routed_ids(route_file: Path) -> set[str]:
    root = ET.parse(route_file).getroot()
    return {element.attrib["id"] for element in root.findall("vehicle") if "id" in element.attrib}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=Path("network/porto.net.xml"))
    parser.add_argument("--od-pairs", type=Path, default=Path("demand/od_pairs.csv"))
    parser.add_argument("--duarouter", type=Path, default=Path(r"C:\Program Files (x86)\Eclipse\Sumo\bin\duarouter.exe"))
    parser.add_argument("--seed", type=int, default=20260721)
    parser.add_argument("--sample-size", type=int, default=8000)
    parser.add_argument("--minimum-routeable-rate", type=float, default=0.99)
    parser.add_argument("--work-directory", type=Path, default=Path("network/validation"))
    parser.add_argument("--report", type=Path, default=Path("network/routeability_validation.json"))
    arguments = parser.parse_args()

    if not arguments.network.is_file() or not arguments.od_pairs.is_file() or not arguments.duarouter.is_file():
        parser.error("Network, OD-pair, and duarouter paths must all exist")
    if not 0 < arguments.minimum_routeable_rate <= 1:
        parser.error("--minimum-routeable-rate must be greater than 0 and at most 1")

    all_pairs = load_pairs(arguments.od_pairs)
    if len(all_pairs) < arguments.sample_size:
        parser.error(f"Only {len(all_pairs)} mapped pairs available; requested {arguments.sample_size}")
    sampled_pairs = random.Random(arguments.seed).sample(all_pairs, arguments.sample_size)
    arguments.work_directory.mkdir(parents=True, exist_ok=True)
    trip_file = arguments.work_directory / "routeability_sample.trips.xml"
    route_file = arguments.work_directory / "routeability_sample.rou.xml"
    log_file = arguments.work_directory / "duarouter.log"
    write_validation_trips(sampled_pairs, trip_file)

    command = [
        str(arguments.duarouter), "--net-file", str(arguments.network), "--route-files", str(trip_file),
        "--output-file", str(route_file), "--ignore-errors", "true", "--no-warnings", "true",
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    log_file.write_text(
        "COMMAND:\n" + " ".join(f'"{part}"' if " " in part else part for part in command)
        + "\n\nSTDOUT:\n" + completed.stdout + "\n\nSTDERR:\n" + completed.stderr,
        encoding="utf-8",
    )
    successful_ids = routed_ids(route_file) if route_file.is_file() else set()
    requested_ids = {f"routeability_{index:05d}" for index in range(arguments.sample_size)}
    failed_ids = sorted(requested_ids - successful_ids)
    routeable_rate = len(successful_ids) / arguments.sample_size
    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "Gate B seeded OD routeability validation; not final trip demand.",
        "inputs": {
            "network": str(arguments.network),
            "network_sha256": sha256(arguments.network),
            "od_pairs": str(arguments.od_pairs),
            "od_pairs_sha256": sha256(arguments.od_pairs),
            "seed": arguments.seed,
            "sample_size": arguments.sample_size,
            "minimum_routeable_rate": arguments.minimum_routeable_rate,
        },
        "artifacts": {
            "validation_trip_file": str(trip_file),
            "validation_route_file": str(route_file),
            "duarouter_log": str(log_file),
            "command": command,
        },
        "results": {
            "duarouter_exit_code": completed.returncode,
            "routeable_pairs": len(successful_ids),
            "unrouteable_pairs": len(failed_ids),
            "routeable_rate": round(routeable_rate, 8),
            "unrouteable_id_examples": failed_ids[:20],
        },
        "passed": completed.returncode == 0 and routeable_rate >= arguments.minimum_routeable_rate,
    }
    arguments.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {arguments.report}")
    print(f"Routeability: {routeable_rate:.2%} ({len(successful_ids)}/{arguments.sample_size})")
    print(f"Gate B routeability: {'PASS' if report['passed'] else 'FAIL'}")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
