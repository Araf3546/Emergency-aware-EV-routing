#!/usr/bin/env python3
"""Validate a SUMO road network before calibrated demand is generated.

The script reports passenger-vehicle access, directed connectivity, and a
reusable source/sink candidate pool.  It does not modify the network.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


PASSENGER_CLASS = "passenger"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def class_is_allowed(attributes: dict[str, str], vehicle_class: str) -> bool:
    """Apply SUMO allow/disallow access semantics for one vehicle class."""
    allowed = attributes.get("allow")
    if allowed is not None:
        return vehicle_class in allowed.split()

    disallowed = attributes.get("disallow")
    if disallowed is not None:
        return vehicle_class not in disallowed.split()

    return True


def strongly_connected_components(
    vertices: set[str], adjacency: dict[str, set[str]]
) -> list[list[str]]:
    """Return directed SCCs using an iterative Kosaraju traversal."""
    reverse_adjacency: dict[str, set[str]] = defaultdict(set)
    for source, destinations in adjacency.items():
        for destination in destinations:
            reverse_adjacency[destination].add(source)

    visited: set[str] = set()
    finish_order: list[str] = []
    for start in vertices:
        if start in visited:
            continue
        visited.add(start)
        stack: list[tuple[str, Iterable[str]]] = [(start, iter(adjacency[start]))]
        while stack:
            vertex, neighbours = stack[-1]
            try:
                neighbour = next(neighbours)
            except StopIteration:
                finish_order.append(vertex)
                stack.pop()
                continue
            if neighbour not in visited:
                visited.add(neighbour)
                stack.append((neighbour, iter(adjacency[neighbour])))

    components: list[list[str]] = []
    assigned: set[str] = set()
    for start in reversed(finish_order):
        if start in assigned:
            continue
        component: list[str] = []
        stack = [start]
        assigned.add(start)
        while stack:
            vertex = stack.pop()
            component.append(vertex)
            for neighbour in reverse_adjacency[vertex]:
                if neighbour not in assigned:
                    assigned.add(neighbour)
                    stack.append(neighbour)
        components.append(component)
    return components


def parse_network(network_path: Path) -> tuple[dict[str, dict[str, str]], list[tuple[str, str]]]:
    """Read non-internal edges and directed connections from a SUMO net XML."""
    root = ET.parse(network_path).getroot()
    edge_types = {element.attrib["id"]: element.attrib for element in root.findall("type")}
    edges: dict[str, dict[str, str]] = {}

    for edge in root.findall("edge"):
        attributes = edge.attrib
        edge_id = attributes.get("id", "")
        if not edge_id or attributes.get("function") == "internal" or edge_id.startswith(":"):
            continue

        access_attributes = dict(edge_types.get(attributes.get("type", ""), {}))
        access_attributes.update(
            {key: value for key, value in attributes.items() if key in {"allow", "disallow"}}
        )
        lanes = edge.findall("lane")
        # Lane-level access is authoritative when present; otherwise use the edge type.
        passenger_accessible = (
            any(class_is_allowed(lane.attrib, PASSENGER_CLASS) for lane in lanes)
            if any("allow" in lane.attrib or "disallow" in lane.attrib for lane in lanes)
            else class_is_allowed(access_attributes, PASSENGER_CLASS)
        )
        edges[edge_id] = {
            "from": attributes.get("from", ""),
            "to": attributes.get("to", ""),
            "type": attributes.get("type", ""),
            "passenger_accessible": str(passenger_accessible).lower(),
        }

    connections = [
        (connection.attrib["from"], connection.attrib["to"])
        for connection in root.findall("connection")
        if "from" in connection.attrib and "to" in connection.attrib
    ]
    return edges, connections


def build_report(
    network_path: Path, min_scc_fraction: float, min_candidate_edges: int
) -> dict[str, object]:
    edges, connections = parse_network(network_path)
    passenger_edges = {
        edge_id
        for edge_id, details in edges.items()
        if details["passenger_accessible"] == "true"
    }
    adjacency: dict[str, set[str]] = {edge_id: set() for edge_id in passenger_edges}
    incoming: dict[str, set[str]] = {edge_id: set() for edge_id in passenger_edges}
    usable_connections = 0
    for source, destination in connections:
        if source in passenger_edges and destination in passenger_edges:
            adjacency[source].add(destination)
            incoming[destination].add(source)
            usable_connections += 1

    components = strongly_connected_components(passenger_edges, adjacency) if passenger_edges else []
    largest_component = max(components, key=len, default=[])
    largest_component_set = set(largest_component)
    scc_fraction = len(largest_component) / len(passenger_edges) if passenger_edges else 0.0

    candidates = sorted(
        edge_id
        for edge_id in largest_component
        if adjacency[edge_id] and incoming[edge_id]
    )
    dead_end_edges = sorted(edge_id for edge_id in passenger_edges if not adjacency[edge_id])
    entry_only_edges = sorted(edge_id for edge_id in passenger_edges if not incoming[edge_id])
    type_counts = Counter(details["type"] or "untyped" for details in edges.values())

    checks = {
        "has_passenger_edges": bool(passenger_edges),
        "largest_scc_meets_minimum": scc_fraction >= min_scc_fraction,
        "candidate_pool_meets_minimum": len(candidates) >= min_candidate_edges,
    }
    return {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "network": {
            "path": str(network_path),
            "sha256": sha256(network_path),
        },
        "parameters": {
            "vehicle_class": PASSENGER_CLASS,
            "min_largest_scc_fraction": min_scc_fraction,
            "min_candidate_edges": min_candidate_edges,
        },
        "counts": {
            "non_internal_edges": len(edges),
            "passenger_accessible_edges": len(passenger_edges),
            "passenger_accessible_connections": usable_connections,
            "strongly_connected_components": len(components),
            "largest_strongly_connected_component_edges": len(largest_component),
            "source_sink_candidate_edges": len(candidates),
            "dead_end_passenger_edges": len(dead_end_edges),
            "entry_only_passenger_edges": len(entry_only_edges),
        },
        "fractions": {
            "largest_scc_of_passenger_edges": round(scc_fraction, 8),
        },
        "edge_type_counts": dict(sorted(type_counts.items())),
        "examples": {
            "source_sink_candidate_edges": candidates[:20],
            "dead_end_passenger_edges": dead_end_edges[:20],
            "entry_only_passenger_edges": entry_only_edges[:20],
        },
        "checks": checks,
        "passed": all(checks.values()),
        "interpretation": (
            "This topology check does not prove calibrated OD reachability. "
            "Run the later sampled-OD routeability validation before demand generation."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--network",
        type=Path,
        default=Path("network/porto.net.xml"),
        help="Input SUMO .net.xml file (default: network/porto.net.xml).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("network/network_validation.json"),
        help="Validation report path (default: network/network_validation.json).",
    )
    parser.add_argument(
        "--min-largest-scc-fraction",
        type=float,
        default=0.90,
        help="Minimum passenger-edge share in the largest SCC (default: 0.90).",
    )
    parser.add_argument("--min-candidate-edges", type=int, default=100)
    arguments = parser.parse_args()

    if not arguments.network.is_file():
        parser.error(f"Network file not found: {arguments.network}")
    if not 0 < arguments.min_largest_scc_fraction <= 1:
        parser.error("--min-largest-scc-fraction must be greater than 0 and at most 1")
    if arguments.min_candidate_edges < 1:
        parser.error("--min-candidate-edges must be at least 1")

    report = build_report(
        arguments.network,
        arguments.min_largest_scc_fraction,
        arguments.min_candidate_edges,
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {arguments.output}")
    print(f"Topology validation: {'PASS' if report['passed'] else 'FAIL'}")
    print(json.dumps(report["counts"], indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
