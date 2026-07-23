#!/usr/bin/env python3
"""Map cleaned Porto trip endpoints to the validated passenger-edge pool."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import pyarrow.parquet as pq
from scipy.spatial import cKDTree
import sumolib


def load_validator(script_path: Path):
    specification = importlib.util.spec_from_file_location("network_validator", script_path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"Cannot load validator module: {script_path}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def passenger_candidate_edges(network_path: Path, validator_path: Path) -> set[str]:
    validator = load_validator(validator_path)
    edges, connections = validator.parse_network(network_path)
    passenger = {
        edge_id for edge_id, details in edges.items() if details["passenger_accessible"] == "true"
    }
    adjacency = {edge_id: set() for edge_id in passenger}
    incoming = {edge_id: set() for edge_id in passenger}
    for source, destination in connections:
        if source in passenger and destination in passenger:
            adjacency[source].add(destination)
            incoming[destination].add(source)
    largest = max(validator.strongly_connected_components(passenger, adjacency), key=len)
    return {edge_id for edge_id in largest if adjacency[edge_id] and incoming[edge_id]}


def densified_edge_samples(network, edge_ids: set[str], spacing_m: float) -> tuple[cKDTree, list[str], dict[str, list[tuple[float, float]]]]:
    points: list[tuple[float, float]] = []
    point_edge_ids: list[str] = []
    shapes: dict[str, list[tuple[float, float]]] = {}
    for edge_id in sorted(edge_ids):
        shape = [(float(x), float(y)) for x, y in network.getEdge(edge_id).getShape()]
        if len(shape) < 2:
            continue
        shapes[edge_id] = shape
        for first, second in zip(shape, shape[1:]):
            distance = math.dist(first, second)
            steps = max(1, math.ceil(distance / spacing_m))
            for step in range(steps):
                ratio = step / steps
                points.append((first[0] + (second[0] - first[0]) * ratio, first[1] + (second[1] - first[1]) * ratio))
                point_edge_ids.append(edge_id)
        points.append(shape[-1])
        point_edge_ids.append(edge_id)
    return cKDTree(points), point_edge_ids, shapes


def point_segment_distance(point: tuple[float, float], first: tuple[float, float], second: tuple[float, float]) -> float:
    dx, dy = second[0] - first[0], second[1] - first[1]
    if dx == 0 and dy == 0:
        return math.dist(point, first)
    ratio = max(0.0, min(1.0, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / (dx * dx + dy * dy)))
    nearest = (first[0] + ratio * dx, first[1] + ratio * dy)
    return math.dist(point, nearest)


def nearest_edge(point: tuple[float, float], tree: cKDTree, point_edge_ids: list[str], shapes: dict[str, list[tuple[float, float]]]) -> tuple[str, float]:
    _, indices = tree.query(point, k=min(12, len(point_edge_ids)))
    candidates = {point_edge_ids[int(index)] for index in (indices if hasattr(indices, "__iter__") else [indices])}
    distances = {
        edge_id: min(point_segment_distance(point, first, second) for first, second in zip(shape, shape[1:]))
        for edge_id, shape in ((edge_id, shapes[edge_id]) for edge_id in candidates)
    }
    edge_id = min(distances, key=distances.get)
    return edge_id, distances[edge_id]


def map_od(
    network_path: Path, cleaned_trips: Path, output: Path, report_output: Path,
    validator_path: Path, max_snap_distance_m: float, spacing_m: float,
) -> None:
    candidate_ids = passenger_candidate_edges(network_path, validator_path)
    network = sumolib.net.readNet(str(network_path), withInternal=False)
    tree, point_edge_ids, shapes = densified_edge_samples(network, candidate_ids, spacing_m)
    counts = {"input_clean_trips": 0, "mapped_od_pairs": 0, "rejected_snap_distance": 0, "rejected_same_edge": 0}
    output.parent.mkdir(parents=True, exist_ok=True)

    table = pq.read_table(cleaned_trips)
    with output.open("w", newline="", encoding="utf-8") as handle:
        fields = [
            "trip_id", "timestamp_utc", "departure_second_of_day_local", "origin_edge", "destination_edge",
            "origin_snap_distance_m", "destination_snap_distance_m", "origin_lon", "origin_lat",
            "destination_lon", "destination_lat", "observed_duration_seconds", "observed_gps_distance_m",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for trip in table.to_pylist():
            counts["input_clean_trips"] += 1
            origin_xy = network.convertLonLat2XY(trip["origin_lon"], trip["origin_lat"])
            destination_xy = network.convertLonLat2XY(trip["destination_lon"], trip["destination_lat"])
            origin_edge, origin_distance = nearest_edge(origin_xy, tree, point_edge_ids, shapes)
            destination_edge, destination_distance = nearest_edge(destination_xy, tree, point_edge_ids, shapes)
            if origin_distance > max_snap_distance_m or destination_distance > max_snap_distance_m:
                counts["rejected_snap_distance"] += 1
                continue
            if origin_edge == destination_edge:
                counts["rejected_same_edge"] += 1
                continue
            writer.writerow({
                key: trip[key] for key in (
                    "trip_id", "timestamp_utc", "departure_second_of_day_local", "origin_lon", "origin_lat",
                    "destination_lon", "destination_lat", "observed_duration_seconds", "observed_gps_distance_m",
                )
            } | {
                "origin_edge": origin_edge,
                "destination_edge": destination_edge,
                "origin_snap_distance_m": round(origin_distance, 3),
                "destination_snap_distance_m": round(destination_distance, 3),
            })
            counts["mapped_od_pairs"] += 1

    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "network": str(network_path),
        "cleaned_trips": str(cleaned_trips),
        "candidate_edge_pool_size": len(candidate_ids),
        "max_snap_distance_m": max_snap_distance_m,
        "edge_sample_spacing_m": spacing_m,
        "counts": counts,
        "next_gate": "Routeability must be checked before OD pairs are used to generate baseline trips.",
    }
    report_output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", type=Path, default=Path("network/porto.net.xml"))
    parser.add_argument("--cleaned-trips", type=Path, default=Path("data/processed/porto_clean_trips.parquet"))
    parser.add_argument("--output", type=Path, default=Path("demand/od_pairs.csv"))
    parser.add_argument("--report", type=Path, default=Path("demand/od_mapping_report.json"))
    parser.add_argument("--validator", type=Path, default=Path("scripts/03_validate_network.py"))
    parser.add_argument("--max-snap-distance-m", type=float, default=100.0)
    parser.add_argument("--edge-sample-spacing-m", type=float, default=15.0)
    arguments = parser.parse_args()
    if arguments.max_snap_distance_m <= 0 or arguments.edge_sample_spacing_m <= 0:
        parser.error("Snap distance and edge-sample spacing must be positive")
    map_od(
        arguments.network, arguments.cleaned_trips, arguments.output, arguments.report,
        arguments.validator, arguments.max_snap_distance_m, arguments.edge_sample_spacing_m,
    )


if __name__ == "__main__":
    main()
