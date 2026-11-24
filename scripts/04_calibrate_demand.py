#!/usr/bin/env python3
"""Clean Porto taxi trajectories and create reproducible demand statistics.

Only completed trajectories whose origin and destination lie in the approved
study-area polygon are retained. The immutable raw archive is never modified.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import zipfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
from zoneinfo import ZoneInfo

import pyarrow as pa
import pyarrow.parquet as pq


PORTO_TIMEZONE = ZoneInfo("Europe/Lisbon")
PARQUET_SCHEMA = pa.schema(
    [
        ("trip_id", pa.string()),
        ("timestamp_utc", pa.int64()),
        ("departure_second_of_day_local", pa.int32()),
        ("origin_lon", pa.float64()),
        ("origin_lat", pa.float64()),
        ("destination_lon", pa.float64()),
        ("destination_lat", pa.float64()),
        ("polyline_points", pa.int32()),
        ("observed_duration_seconds", pa.int32()),
        ("observed_gps_distance_m", pa.float64()),
    ]
)


def load_polygon(path: Path) -> list[list[float]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    features = document.get("features", [])
    if len(features) != 1 or features[0].get("geometry", {}).get("type") != "Polygon":
        raise ValueError(f"Expected exactly one Polygon feature in {path}")
    return features[0]["geometry"]["coordinates"][0]


def point_in_polygon(lon: float, lat: float, polygon: list[list[float]]) -> bool:
    """Return whether a WGS84 point lies in a polygon, including its boundary."""
    inside = False
    for index, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[(index + 1) % len(polygon)]
        if (y1 > lat) != (y2 > lat):
            crossing_lon = (x2 - x1) * (lat - y1) / (y2 - y1) + x1
            if lon <= crossing_lon:
                inside = not inside
    return inside


def haversine_metres(first: list[float], second: list[float]) -> float:
    lon1, lat1 = map(math.radians, first)
    lon2, lat2 = map(math.radians, second)
    sin_lat = math.sin((lat2 - lat1) / 2)
    sin_lon = math.sin((lon2 - lon1) / 2)
    a_value = sin_lat**2 + math.cos(lat1) * math.cos(lat2) * sin_lon**2
    return 6_371_000 * 2 * math.atan2(math.sqrt(a_value), math.sqrt(1 - a_value))


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    location = (len(ordered) - 1) * percent / 100
    lower = math.floor(location)
    upper = math.ceil(location)
    if lower == upper:
        return round(float(ordered[lower]), 3)
    return round(float(ordered[lower] + (ordered[upper] - ordered[lower]) * (location - lower)), 3)


def describe(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "min": None, "max": None}
    return {
        "count": len(values),
        "mean": round(mean(values), 3),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "min": round(min(values), 3),
        "max": round(max(values), 3),
    }


def clean(
    raw_archive: Path, study_area: Path, parquet_output: Path, cleaning_report_output: Path,
    statistics_output: Path, batch_size: int,
) -> None:
    polygon = load_polygon(study_area)
    counters: Counter[str] = Counter()
    durations: list[float] = []
    distances: list[float] = []
    point_counts: list[float] = []
    departure_hours: Counter[int] = Counter()
    records: list[dict[str, object]] = []
    parquet_output.parent.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(raw_archive) as archive:
        csv_files = [name for name in archive.namelist() if name.lower().endswith(".csv")]
        if len(csv_files) != 1:
            raise ValueError(f"Expected exactly one CSV in {raw_archive}, found {csv_files}")
        with archive.open(csv_files[0]) as raw_file, pq.ParquetWriter(
            parquet_output, PARQUET_SCHEMA, compression="zstd"
        ) as writer:
            reader = csv.DictReader((line.decode("utf-8") for line in raw_file))
            for row in reader:
                counters["input_rows"] += 1
                if row.get("MISSING_DATA", "").strip().lower() != "false":
                    counters["rejected_missing_data"] += 1
                    continue
                try:
                    polyline = json.loads(row["POLYLINE"])
                except (KeyError, TypeError, json.JSONDecodeError):
                    counters["rejected_invalid_polyline"] += 1
                    continue
                if not isinstance(polyline, list) or len(polyline) < 2:
                    counters["rejected_short_polyline"] += 1
                    continue
                try:
                    origin = [float(polyline[0][0]), float(polyline[0][1])]
                    destination = [float(polyline[-1][0]), float(polyline[-1][1])]
                    timestamp = int(row["TIMESTAMP"])
                    if not all(math.isfinite(value) for value in origin + destination):
                        raise ValueError
                except (IndexError, TypeError, ValueError, KeyError):
                    counters["rejected_invalid_coordinates"] += 1
                    continue
                if not point_in_polygon(*origin, polygon) or not point_in_polygon(*destination, polygon):
                    counters["rejected_outside_study_area"] += 1
                    continue

                distance_m = sum(
                    haversine_metres(first, second) for first, second in zip(polyline, polyline[1:])
                )
                duration_s = (len(polyline) - 1) * 15
                local_departure = datetime.fromtimestamp(timestamp, timezone.utc).astimezone(PORTO_TIMEZONE)
                record = {
                    "trip_id": str(row["TRIP_ID"]),
                    "timestamp_utc": timestamp,
                    "departure_second_of_day_local": (
                        local_departure.hour * 3600 + local_departure.minute * 60 + local_departure.second
                    ),
                    "origin_lon": origin[0],
                    "origin_lat": origin[1],
                    "destination_lon": destination[0],
                    "destination_lat": destination[1],
                    "polyline_points": len(polyline),
                    "observed_duration_seconds": duration_s,
                    "observed_gps_distance_m": distance_m,
                }
                records.append(record)
                counters["retained_rows"] += 1
                durations.append(duration_s)
                distances.append(distance_m)
                point_counts.append(len(polyline))
                departure_hours[local_departure.hour] += 1
                if len(records) >= batch_size:
                    writer.write_table(pa.Table.from_pylist(records, schema=PARQUET_SCHEMA))
                    records.clear()
            if records:
                writer.write_table(pa.Table.from_pylist(records, schema=PARQUET_SCHEMA))

    report = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "raw_archive": str(raw_archive),
        "study_area": str(study_area),
        "selection_rule": "MISSING_DATA is false; at least two GPS points; both origin and destination are inside the approved study-area polygon.",
        "counts": dict(counters),
    }
    statistics = {
        "report_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "cleaned_trip_count": counters["retained_rows"],
        "departure_time_basis": "Europe/Lisbon local time derived from the UTC Unix TIMESTAMP field.",
        "departure_hour_counts": {str(hour): departure_hours[hour] for hour in range(24)},
        "observed_duration_seconds": describe(durations),
        "observed_gps_distance_m": describe(distances),
        "polyline_point_count": describe(point_counts),
    }
    cleaning_report_output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    statistics_output.parent.mkdir(parents=True, exist_ok=True)
    statistics_output.write_text(json.dumps(statistics, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-archive", type=Path, default=Path("Data/raw/porto/train.csv.zip"))
    parser.add_argument("--study-area", type=Path, default=Path("config/porto_study_area.geojson"))
    parser.add_argument("--parquet-output", type=Path, default=Path("data/processed/porto_clean_trips.parquet"))
    parser.add_argument("--cleaning-report", type=Path, default=Path("data/processed/porto_cleaning_report.json"))
    parser.add_argument("--statistics-output", type=Path, default=Path("demand/demand_statistics.json"))
    parser.add_argument("--batch-size", type=int, default=10_000)
    arguments = parser.parse_args()
    if arguments.batch_size < 1:
        parser.error("--batch-size must be at least 1")
    clean(
        arguments.raw_archive, arguments.study_area, arguments.parquet_output,
        arguments.cleaning_report, arguments.statistics_output, arguments.batch_size,
    )


if __name__ == "__main__":
    main()
