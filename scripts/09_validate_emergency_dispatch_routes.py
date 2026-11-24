#!/usr/bin/env python3
"""Validate the seeded emergency dispatch routes against the planned incident set."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

try:
    import yaml
except ImportError:  # Optional when the scenario is supplied as JSON.
    yaml = None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_document(path: Path):
    text = path.read_text(encoding="utf-8")
    suffix = path.suffix.lower()
    if suffix == ".json":
        return json.loads(text)
    if suffix in {".yml", ".yaml"}:
        if yaml is None:
            raise RuntimeError("PyYAML is required to read YAML emergency scenarios")
        return yaml.safe_load(text)
    raise ValueError(f"Unsupported scenario format: {path.suffix}")


def load_scenario_events(path: Path) -> list[dict[str, object]]:
    document = load_document(path)
    if isinstance(document, dict):
        events = document.get("events", [])
    else:
        events = document
    if not isinstance(events, list):
        raise ValueError("Scenario file must contain an events list")
    return events


def load_trip_map(path: Path) -> dict[str, dict[str, str]]:
    root = ET.parse(path).getroot()
    trip_map: dict[str, dict[str, str]] = {}
    for trip in root.findall("trip"):
        trip_id = trip.attrib["id"]
        trip_map[trip_id] = {
            "from": trip.attrib.get("from", ""),
            "to": trip.attrib.get("to", ""),
            "depart": trip.attrib.get("depart", ""),
        }
    return trip_map


def load_route_vehicle_ids(path: Path) -> set[str]:
    if not path.is_file():
        return set()
    root = ET.parse(path).getroot()
    return {vehicle.attrib["id"] for vehicle in root.findall("vehicle") if "id" in vehicle.attrib}


def write_dispatch_trips(events: list[dict[str, object]], output: Path) -> None:
    root = ET.Element("routes")
    vehicle_type_styles = {
        "ambulance": {"vClass": "emergency", "color": "white"},
        "fire_truck": {"vClass": "emergency", "color": "red"},
        "police": {"vClass": "emergency", "color": "blue"},
        "relief_vehicle": {"vClass": "emergency", "color": "yellow"},
    }
    for vehicle_type in sorted({str(event["dispatch"]["vehicle_type"]) for event in events if isinstance(event.get("dispatch"), dict)}):
        style = vehicle_type_styles.get(vehicle_type, {"vClass": "emergency"})
        ET.SubElement(root, "vType", id=vehicle_type, **style)
    for event in events:
        dispatch = event.get("dispatch", {})
        if not isinstance(dispatch, dict):
            raise ValueError("Each event must include a dispatch mapping")
        ET.SubElement(
            root,
            "trip",
            id=str(dispatch["vehicle_id"]),
            depart=str(dispatch["departure_time"]),
            type=str(dispatch["vehicle_type"]),
            **{"from": str(dispatch["origin_edge"]), "to": str(dispatch["destination_edge"])},
        )
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, default=Path("scenarios/emergency_scenarios.yaml"))
    parser.add_argument("--network", type=Path, default=Path("network/porto.net.xml"))
    parser.add_argument("--routeability-trip-file", type=Path, default=Path("network/validation/routeability_sample.trips.xml"))
    parser.add_argument("--routeability-route-file", type=Path, default=Path("network/validation/routeability_sample.rou.xml"))
    parser.add_argument("--duarouter", type=Path, default=Path(r"C:\Program Files (x86)\Eclipse\Sumo\bin\duarouter.exe"))
    parser.add_argument("--work-directory", type=Path, default=Path("network/validation/emergency_dispatch_validation"))
    parser.add_argument("--dispatch-trip-file", type=Path, default=Path("network/validation/emergency_dispatch_validation.trips.xml"))
    parser.add_argument("--dispatch-route-file", type=Path, default=Path("network/validation/emergency_dispatch_validation.rou.xml"))
    parser.add_argument("--error-log", type=Path, default=Path("network/validation/emergency_dispatch_validation.log"))
    parser.add_argument("--report", type=Path, default=Path("network/emergency_dispatch_validation.json"))
    arguments = parser.parse_args()

    for required in (arguments.scenario, arguments.network, arguments.routeability_trip_file, arguments.routeability_route_file, arguments.duarouter):
        if not required.is_file():
            parser.error(f"Required file not found: {required}")

    events = load_scenario_events(arguments.scenario)
    if not events:
        parser.error("Scenario file does not contain any events")

    arguments.work_directory.mkdir(parents=True, exist_ok=True)
    write_dispatch_trips(events, arguments.dispatch_trip_file)

    command = [
        str(arguments.duarouter),
        "--net-file",
        str(arguments.network),
        "--route-files",
        str(arguments.dispatch_trip_file),
        "--output-file",
        str(arguments.dispatch_route_file),
        "--error-log",
        str(arguments.error_log),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)

    routeability_trip_map = load_trip_map(arguments.routeability_trip_file)
    routeability_route_ids = load_route_vehicle_ids(arguments.routeability_route_file)
    dispatch_route_ids = load_route_vehicle_ids(arguments.dispatch_route_file)

    dispatch_reports = []
    for event in events:
        event_id = event.get("event_id", event.get("id"))
        dispatch = event.get("dispatch", {})
        if not isinstance(dispatch, dict):
            parser.error(f"Event {event_id} is missing a dispatch mapping")

        reference = dispatch.get("validated_route_reference", {})
        if not isinstance(reference, dict):
            reference = {}
        reference_trip_id = str(reference.get("trip_id", ""))
        reference_trip = routeability_trip_map.get(reference_trip_id)
        route_reference_present = bool(reference_trip_id) and reference_trip_id in routeability_route_ids and reference_trip is not None
        reference_edges_match = bool(
            reference_trip
            and reference_trip.get("from") == str(dispatch.get("origin_edge", ""))
            and reference_trip.get("to") == str(dispatch.get("destination_edge", ""))
        )
        dispatch_id = str(dispatch["vehicle_id"])
        dispatch_routed = dispatch_id in dispatch_route_ids
        route_status = "routeable" if dispatch_routed else "unrouteable"
        reason = ""
        if not dispatch_routed:
            reason = "dispatch trip was not present in the routed output file"
        elif not route_reference_present:
            reason = "validated route reference was not found in the routeability artifacts"
        elif not reference_edges_match:
            reason = "validated route reference edges do not match the planned dispatch"

        dispatch_reports.append(
            {
                "event_id": event_id,
                "dispatch_id": dispatch_id,
                "vehicle_type": dispatch.get("vehicle_type"),
                "origin_edge": dispatch.get("origin_edge"),
                "destination_edge": dispatch.get("destination_edge"),
                "departure_time": dispatch.get("departure_time"),
                "route_status": route_status,
                "dispatch_routed": dispatch_routed,
                "validated_route_reference": reference,
                "validated_route_reference_present": route_reference_present,
                "validated_route_reference_edges_match": reference_edges_match,
                "reason": reason,
            }
        )

    passed = completed.returncode == 0 and all(item["dispatch_routed"] for item in dispatch_reports) and all(
        item["validated_route_reference_present"] and item["validated_route_reference_edges_match"]
        for item in dispatch_reports
    )

    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "Scenario-specific emergency dispatch validation for the seeded Porto thesis scenario.",
        "inputs": {
            "scenario": str(arguments.scenario),
            "scenario_sha256": sha256(arguments.scenario),
            "network": str(arguments.network),
            "network_sha256": sha256(arguments.network),
            "routeability_trip_file": str(arguments.routeability_trip_file),
            "routeability_trip_file_sha256": sha256(arguments.routeability_trip_file),
            "routeability_route_file": str(arguments.routeability_route_file),
            "routeability_route_file_sha256": sha256(arguments.routeability_route_file),
        },
        "artifacts": {
            "dispatch_trip_file": str(arguments.dispatch_trip_file),
            "dispatch_route_file": str(arguments.dispatch_route_file),
            "error_log": str(arguments.error_log),
            "command": command,
        },
        "results": {
            "duarouter_exit_code": completed.returncode,
            "dispatch_count": len(dispatch_reports),
            "routeable_dispatches": sum(1 for item in dispatch_reports if item["dispatch_routed"]),
            "unrouteable_dispatches": sum(1 for item in dispatch_reports if not item["dispatch_routed"]),
            "reference_matches": sum(1 for item in dispatch_reports if item["validated_route_reference_present"] and item["validated_route_reference_edges_match"]),
        },
        "dispatches": dispatch_reports,
        "passed": passed,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    arguments.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {arguments.report}")
    print(f"Dispatch validation: {'PASS' if passed else 'FAIL'} ({report['results']['routeable_dispatches']}/{report['results']['dispatch_count']})")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())