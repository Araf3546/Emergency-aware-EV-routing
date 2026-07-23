#!/usr/bin/env python3
"""Generate destination-valid routes for the seeded normal-traffic baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=Path("network/porto.net.xml"))
    parser.add_argument("--trips", type=Path, default=Path("demand/porto.trips.xml"))
    parser.add_argument("--output", type=Path, default=Path("demand/porto.rou.xml"))
    parser.add_argument("--report", type=Path, default=Path("demand/baseline_route_generation.json"))
    parser.add_argument("--duarouter", type=Path, default=Path(r"C:\Program Files (x86)\Eclipse\Sumo\bin\duarouter.exe"))
    parser.add_argument("--seed", type=int, default=20260722)
    arguments = parser.parse_args()
    for path in (arguments.network, arguments.trips, arguments.duarouter):
        if not path.is_file():
            parser.error(f"Required file not found: {path}")

    command = [
        str(arguments.duarouter), "--net-file", str(arguments.network), "--route-files", str(arguments.trips),
        "--output-file", str(arguments.output), "--seed", str(arguments.seed), "--remove-loops", "true",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    route_count = vehicle_count(arguments.output) if arguments.output.is_file() else 0
    trip_count = len(ET.parse(arguments.trips).getroot().findall("trip"))
    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "network": str(arguments.network), "network_sha256": sha256(arguments.network),
            "trips": str(arguments.trips), "trips_sha256": sha256(arguments.trips), "seed": arguments.seed,
        },
        "command": command,
        "duarouter_exit_code": completed.returncode,
        "trip_count": trip_count,
        "routed_vehicle_count": route_count,
        "all_trips_routed": route_count == trip_count,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "output": str(arguments.output) if arguments.output.is_file() else None,
        "output_sha256": sha256(arguments.output) if arguments.output.is_file() else None,
    }
    arguments.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {arguments.output}: {route_count}/{trip_count} trips routed")
    return 0 if completed.returncode == 0 and route_count == trip_count else 2


if __name__ == "__main__":
    raise SystemExit(main())
