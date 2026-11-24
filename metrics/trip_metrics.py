"""Contract metrics derived from SUMO's own trip output.

Audit findings M1, M3, M4, M6, M7 and M8 all have the same root cause: the metrics were
rebuilt by polling TraCI every step instead of reading what SUMO already records. That
produced a waiting time that grew quadratically with stop duration, a congestion figure
that was really a property of the road graph, a travel time that excluded insertion delay,
and a teleport count that was a maximum rather than a total.

`tripinfo.xml` and `summary.xml` are authoritative, cost nothing at run time, and match the
contract's definitions directly:

* contract section 8, travel time - "vehicle arrival time minus departure time". `duration`
  is measured from actual insertion, so scheduled travel time is `duration + departDelay`.
* contract section 8, waiting time - "SUMO accumulated waiting time per completed vehicle",
  which is the `waitingTime` attribute.
* contract section 8 with decision 13.4, congestion - "mean vehicle time loss relative to
  free-flow movement" in seconds per vehicle, which is the `timeLoss` attribute. Not a
  speed proxy.
* contract section 5 - rejected trips, route failures, teleports and incomplete trips must
  all be reported. `summary.xml` carries the running totals.

This module only reads files, so it can be tested without starting a simulation.
"""

from __future__ import annotations

import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Optional
from xml.etree import ElementTree as ET

# Vehicle types that count as emergency vehicles are resolved by the caller, so this module
# stays free of simulation imports.

TRIP_FIELDS = (
    "depart",
    "departDelay",
    "arrival",
    "duration",
    "routeLength",
    "waitingTime",
    "waitingCount",
    "stopTime",
    "timeLoss",
    "rerouteNo",
)


def load_tripinfo(path: Path | str, finished_only: bool = False) -> Dict[str, Dict[str, object]]:
    """Read `tripinfo.xml` into {vehicle_id: {field: value}}.

    Only vehicles that finished their trip appear in this file, which is why it is the
    correct basis for "completed trip" statistics (audit M8: the old baseline module
    recorded unfinished vehicles as having arrived at the end of the simulation).

    That holds only without `--tripinfo-output.write-unfinished`. A file written with it (every
    tuner and comparison run) also holds unfinished vehicles, with `arrival="-1"`. Pass
    `finished_only=True` to drop them (audit B5, 2026-09-17); the default reads every record,
    exactly as before, so no existing caller changes.
    """
    trips: Dict[str, Dict[str, object]] = {}
    root = ET.parse(str(path)).getroot()
    for element in root.findall("tripinfo"):
        attributes = element.attrib
        if finished_only and float(attributes.get("arrival", "0")) < 0:
            continue
        record: Dict[str, object] = {
            "vehicle_type": attributes.get("vType", ""),
            "vaporized": attributes.get("vaporized", ""),
        }
        for field in TRIP_FIELDS:
            value = attributes.get(field)
            if value is None:
                continue
            record[field] = int(value) if field in {"waitingCount", "rerouteNo"} else float(value)
        record["scheduled_travel_time"] = float(record.get("duration", 0.0)) + float(
            record.get("departDelay", 0.0)
        )
        trips[attributes["id"]] = record
    return trips


def load_summary(path: Path | str) -> Dict[str, float]:
    """Read the final step of `summary.xml`.

    `teleports` there is cumulative, which is what contract section 8 asks for. Audit M6
    describes the old approach, `max()` over per-step teleport lists, which reports the
    largest number of simultaneous teleports instead of the total.
    """
    root = ET.parse(str(path)).getroot()
    steps = root.findall("step")
    if not steps:
        return {}
    last = steps[-1].attrib
    result: Dict[str, float] = {}
    for key in ("time", "loaded", "inserted", "running", "waiting", "ended", "arrived", "teleports", "collisions"):
        if key in last:
            result[key] = float(last[key])
    return result


def load_summary_timeseries(path: Path | str) -> List[Dict[str, float]]:
    """Per-step network state from `summary.xml`.

    SUMO already records running, halting, mean speed and mean waiting time every step, at
    no cost. Audit M4 describes the alternative the code used to take: averaging
    `getLastStepMeanSpeed` over every edge in the network each step, which measured the road
    graph rather than the traffic and dominated the runtime.
    """
    root = ET.parse(str(path)).getroot()
    series: List[Dict[str, float]] = []
    for step in root.findall("step"):
        attributes = step.attrib
        row: Dict[str, float] = {}
        for key in ("time", "running", "halting", "meanSpeed", "meanWaitingTime", "meanTravelTime", "teleports"):
            if key in attributes:
                row[key] = float(attributes[key])
        if row:
            series.append(row)
    return series


def _mean(values: List[float]) -> Optional[float]:
    return round(statistics.mean(values), 4) if values else None


def _median(values: List[float]) -> Optional[float]:
    return round(statistics.median(values), 4) if values else None


def aggregate(
    trips: Dict[str, Dict[str, object]],
    summary: Optional[Dict[str, float]] = None,
    emergency_types: Iterable[str] = (),
) -> Dict[str, object]:
    """Summarise completed trips using the contract's metric definitions."""
    emergency = set(emergency_types)

    def series(field: str, subset: Optional[Dict[str, Dict[str, object]]] = None) -> List[float]:
        source = trips if subset is None else subset
        return [float(record[field]) for record in source.values() if field in record]

    emergency_trips = {
        veh: record
        for veh, record in trips.items()
        if record.get("vehicle_type") in emergency
    }
    normal_trips = {
        veh: record for veh, record in trips.items() if veh not in emergency_trips
    }

    report: Dict[str, object] = {
        "completed_trips": len(trips),
        "completed_emergency_trips": len(emergency_trips),
        "completed_normal_trips": len(normal_trips),
        # Contract section 8: arrival minus scheduled departure, so insertion delay counts.
        "mean_travel_time_s": _mean(series("scheduled_travel_time")),
        "median_travel_time_s": _median(series("scheduled_travel_time")),
        "mean_in_network_duration_s": _mean(series("duration")),
        "mean_depart_delay_s": _mean(series("departDelay")),
        # Contract section 8: SUMO accumulated waiting time, read once, not summed per step.
        "mean_waiting_time_s": _mean(series("waitingTime")),
        "median_waiting_time_s": _median(series("waitingTime")),
        # Contract section 8 with decision 13.4: congestion is mean time loss in seconds.
        "mean_time_loss_s": _mean(series("timeLoss")),
        "median_time_loss_s": _median(series("timeLoss")),
        "mean_route_length_m": _mean(series("routeLength")),
        "successful_reroutes_total": int(sum(series("rerouteNo"))),
        "vehicles_rerouted": sum(1 for record in trips.values() if record.get("rerouteNo", 0)),
        "mean_emergency_travel_time_s": _mean(series("scheduled_travel_time", emergency_trips)),
        "mean_emergency_time_loss_s": _mean(series("timeLoss", emergency_trips)),
        "mean_normal_time_loss_s": _mean(series("timeLoss", normal_trips)),
    }

    if summary:
        loaded = summary.get("loaded", 0.0)
        inserted = summary.get("inserted", 0.0)
        ended = summary.get("ended", summary.get("arrived", 0.0))
        report.update(
            {
                "loaded_vehicles": int(loaded),
                "inserted_vehicles": int(inserted),
                "ended_vehicles": int(ended),
                "still_running_at_end": int(summary.get("running", 0.0)),
                # Contract section 5 requires these reported, never silently ignored.
                "never_inserted_vehicles": int(max(loaded - inserted, 0.0)),
                "incomplete_trips": int(max(inserted - ended, 0.0)),
                "teleports_total": int(summary.get("teleports", 0.0)),
                "collisions_total": int(summary.get("collisions", 0.0)),
                "insertion_rate": round(inserted / loaded, 6) if loaded else None,
                "completion_rate": round(ended / loaded, 6) if loaded else None,
            }
        )
    return report


def reconcile_reroutes(
    trips: Dict[str, Dict[str, object]], applied_reroutes: int
) -> Dict[str, object]:
    """Compare our own applied-reroute counter with SUMO's `rerouteNo`.

    They agree only over vehicles that finished, because a vehicle still running at the end
    of the simulation never writes a tripinfo entry. Reported rather than asserted.
    """
    sumo_total = int(sum(int(record.get("rerouteNo", 0)) for record in trips.values()))
    return {
        "reroutes_applied_by_runner": applied_reroutes,
        "sumo_reroute_no_over_completed_trips": sumo_total,
        "note": (
            "SUMO's rerouteNo covers completed trips only; vehicles still running at the "
            "end of the run write no tripinfo entry, so the runner's total is normally "
            "the larger of the two."
        ),
    }


def build_report(
    tripinfo_path: Path | str,
    summary_path: Optional[Path | str] = None,
    emergency_types: Iterable[str] = (),
    applied_reroutes: Optional[int] = None,
    finished_only: bool = False,
) -> Dict[str, object]:
    """Full metric report for one run, straight from SUMO's own output.

    `finished_only` is passed to `load_tripinfo`; use it for a file written with unfinished vehicles.
    """
    trips = load_tripinfo(tripinfo_path, finished_only=finished_only)
    summary = load_summary(summary_path) if summary_path and Path(summary_path).exists() else None
    report = aggregate(trips, summary, emergency_types)
    report["source"] = {
        "tripinfo": str(tripinfo_path),
        "summary": str(summary_path) if summary_path else None,
    }
    if applied_reroutes is not None:
        report["reroute_reconciliation"] = reconcile_reroutes(trips, applied_reroutes)
    return report
