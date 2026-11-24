"""The emergency metrics the design document requires but the code never produced.

Audit M11 lists five metrics that `design_phase.md` sections 5.2 and 5.4 require and
`p3_checklist.md` section 7 leaves unchecked. They are the measurements the thesis argument
rests on, so they must exist before the comparison campaign, not after it.

1. **Emergency vehicle delay** relative to free-flow movement. SUMO's `timeLoss` for the
   responder is exactly that: the time it lost against travelling its own route at the
   allowed speed.
2. **Response time under high congestion.** Each incident's window is classified against the
   network state SUMO records in `summary.xml`, so the classification uses the same data as
   every other metric rather than a separate proxy.
3. **Response time under multiple simultaneous emergencies.** Counts how many incidents were
   active at any point between an incident starting and its responder arriving.
4. **Per-event end time** in the event log.
5. **Conflict / blockage rate near emergency zones**: how much of the traffic standing on
   the incident's own edges was halted while the incident was active.

Everything here is computed from files a run already writes, except the blockage samples,
which the runtime collects over a handful of edges per step.
"""

from __future__ import annotations

import json
import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

from metrics.trip_metrics import load_summary_timeseries, load_tripinfo


def load_event_log(path: Path | str) -> List[Dict[str, object]]:
    """Read a `*_emergency_events.jsonl` file, accepting either responder field name.

    The two runners once wrote different keys for the responding vehicle. They now agree,
    but the older spelling is still accepted so previously recorded runs remain readable.
    """
    text = Path(path).read_text(encoding="utf-8")
    events = [json.loads(line) for line in text.splitlines() if line.strip()]
    for event in events:
        if not event.get("vehicle_id") and event.get("first_arrival_vehicle"):
            event["vehicle_id"] = event["first_arrival_vehicle"]
    return events


# ---------------------------------------------------------------- 1. responder delay

def responder_delay(
    trips: Dict[str, Dict[str, object]],
    responder_ids: Iterable[str],
) -> Dict[str, object]:
    """Delay of each dispatched responder against free-flow movement, in seconds.

    `timeLoss` is SUMO's own measure of time lost relative to travelling the vehicle's route
    at the maximum allowed speed, which is the design document's "ideal reference path".
    A responder still driving at the end of the run has no tripinfo entry and is reported as
    unmeasured rather than as zero.
    """
    per_vehicle = {}
    for vehicle_id in responder_ids:
        record = trips.get(vehicle_id)
        if record is None:
            per_vehicle[vehicle_id] = None
            continue
        per_vehicle[vehicle_id] = {
            "time_loss_s": round(float(record.get("timeLoss", 0.0)), 2),
            "travel_time_s": round(float(record["scheduled_travel_time"]), 2),
            "free_flow_travel_time_s": round(
                float(record["scheduled_travel_time"]) - float(record.get("timeLoss", 0.0)), 2
            ),
            "route_length_m": round(float(record.get("routeLength", 0.0)), 2),
        }
    measured = [v["time_loss_s"] for v in per_vehicle.values() if v]
    return {
        "per_responder": per_vehicle,
        "responders_measured": len(measured),
        "mean_delay_s": round(statistics.mean(measured), 2) if measured else None,
    }


# ------------------------------------------------- 2. congestion context of each incident

def network_congestion_windows(
    summary_series: Sequence[Dict[str, float]],
    events: Sequence[Dict[str, object]],
    high_congestion_quantile: float = 0.75,
) -> Dict[str, object]:
    """Classify each incident's response window as high or normal congestion.

    The threshold is the given quantile of `meanWaitingTime` across the whole run, so it is
    relative to the run itself and needs no magic constant. The window measured is from the
    incident starting to its responder arriving, which is the interval the response time
    covers.
    """
    waiting = [row.get("meanWaitingTime", 0.0) for row in summary_series]
    if not waiting:
        return {"threshold_mean_waiting_time_s": None, "per_event": {}}

    ordered = sorted(waiting)
    index = min(int(len(ordered) * high_congestion_quantile), len(ordered) - 1)
    threshold = ordered[index]

    by_time = {row["time"]: row for row in summary_series if "time" in row}
    per_event: Dict[str, object] = {}
    for event in events:
        start = float(event["start_time"])
        end = float(event.get("first_arrival_time", event["end_time"]))
        window = [
            row for time, row in by_time.items() if start <= time <= end
        ]
        if not window:
            per_event[str(event["event_id"])] = None
            continue
        mean_waiting = statistics.mean(row.get("meanWaitingTime", 0.0) for row in window)
        per_event[str(event["event_id"])] = {
            "response_time_s": event.get("response_time"),
            "mean_waiting_time_in_window_s": round(mean_waiting, 3),
            "mean_speed_in_window_ms": round(
                statistics.mean(row.get("meanSpeed", 0.0) for row in window), 3
            ),
            "high_congestion": mean_waiting >= threshold,
        }
    return {
        "threshold_mean_waiting_time_s": round(threshold, 3),
        "high_congestion_quantile": high_congestion_quantile,
        "per_event": per_event,
    }


# ------------------------------------------- 3. simultaneous emergencies during a response

def simultaneous_emergencies(events: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """How many incidents were active at any point during each incident's response."""
    per_event: Dict[str, object] = {}
    for event in events:
        start = float(event["start_time"])
        arrival = float(event.get("first_arrival_time", event["end_time"]))
        overlapping = [
            other["event_id"]
            for other in events
            if other["event_id"] != event["event_id"]
            and float(other["start_time"]) <= arrival
            and float(other["end_time"]) >= start
        ]
        per_event[str(event["event_id"])] = {
            "response_time_s": event.get("response_time"),
            "concurrent_incidents": len(overlapping) + 1,
            "overlapping_event_ids": overlapping,
        }
    counts = [entry["concurrent_incidents"] for entry in per_event.values()]
    return {
        "per_event": per_event,
        "max_concurrent_incidents": max(counts) if counts else 0,
        "any_overlap": any(count > 1 for count in counts),
    }


# ----------------------------------------------------- 5. blockage near the emergency zone

def blockage_rate(samples: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Share of vehicles on an incident's own edges that were halted while it was active.

    Samples come from the runtime, one per step per active incident:
    `{"event_id": ..., "sim_time": ..., "vehicles": n, "halting": h}`.
    """
    per_event: Dict[str, Dict[str, float]] = {}
    for sample in samples:
        key = str(sample["event_id"])
        bucket = per_event.setdefault(key, {"vehicle_steps": 0.0, "halting_steps": 0.0, "samples": 0.0})
        bucket["vehicle_steps"] += float(sample.get("vehicles", 0))
        bucket["halting_steps"] += float(sample.get("halting", 0))
        bucket["samples"] += 1

    result: Dict[str, object] = {}
    for key, bucket in per_event.items():
        vehicle_steps = bucket["vehicle_steps"]
        result[key] = {
            "samples": int(bucket["samples"]),
            "vehicle_steps_on_incident_edges": int(vehicle_steps),
            "halting_vehicle_steps": int(bucket["halting_steps"]),
            "blockage_rate": round(bucket["halting_steps"] / vehicle_steps, 4) if vehicle_steps else None,
        }
    return result


# ----------------------------------------------------------------------- full report

def build_emergency_report(
    event_log_path: Path | str,
    tripinfo_path: Path | str,
    summary_path: Optional[Path | str] = None,
    blockage_samples: Sequence[Dict[str, object]] = (),
) -> Dict[str, object]:
    """The five metrics of audit M11, for one run."""
    events = load_event_log(event_log_path)
    trips = load_tripinfo(tripinfo_path)
    series = load_summary_timeseries(summary_path) if summary_path and Path(summary_path).exists() else []

    responder_ids = [event["vehicle_id"] for event in events if event.get("vehicle_id")]

    return {
        "incidents": len(events),
        # 4. per-event end time, which the event log carries since B5.
        "events": [
            {
                "event_id": event["event_id"],
                "start_time": event["start_time"],
                "end_time": event["end_time"],
                "response_time_s": event.get("response_time"),
                "responder": event.get("vehicle_id"),
                "arrived_after_incident_end": event.get("arrived_after_incident_end"),
            }
            for event in events
        ],
        "responder_delay": responder_delay(trips, responder_ids),
        "congestion_context": network_congestion_windows(series, events),
        "simultaneous_emergencies": simultaneous_emergencies(events),
        "blockage_near_emergency_zones": blockage_rate(blockage_samples),
    }
