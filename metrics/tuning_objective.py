"""The E3 weight-tuning objective, decided 2026-09-13 (decisions D1-D8, contract decision 13.17).

Reads SUMO's own output files only, so every rule here can be tested without a simulation.

    J = 0.35 * TT/TT_b  +  0.30 * RT  +  0.2625 * TL/TL_b  +  0.07 * U/U_b  +  0.0175 * P/P_b

Lower is better. `_b` is the no-rerouting baseline at the same seed and duration, so the baseline
itself scores exactly 1.0 (D2).

* TT, travel (D1): the mean, over every vehicle scheduled to depart by T, of (arrival, or T if it
  has not arrived) minus its scheduled departure. A vehicle still driving at T, or never inserted,
  counts its time up to T, so finishing always lowers a vehicle's counted time and leaving a slow
  trip unfinished can never pay.
* RT, emergency response (D3, D4): the mean over incidents started before T of
  (response / that incident's baseline response). A valid arrival counts as measured, including
  one after the incident window (13.10). A responder that teleported (M12) or had not reached the
  centre edge by T counts as T - activation, which is always at least any valid arrival.
* TL, time loss (D1, decision 13.4): the mean of SUMO `timeLoss` over every vehicle that entered
  the network, finished or not. Insertion waiting is not time loss in SUMO; it is in TT only.
* U, incomplete share (D5): 1 - arrived / vehicles scheduled by T. Kept as a deliberate
  completion guard although TT already counts unfinished time.
* P, teleports (D6): SUMO's teleport events (summary.xml total) / vehicles scheduled by T.
* "Arrived" in TT and U (2026-09-14): a normal arrival, or one SUMO terminal event, "teleports
  beyond arrival edge". SUMO removes a teleporting vehicle that reaches the end of its route
  before it can be reinserted. It counts that vehicle in summary `arrived` and writes its tripinfo
  record with the removal time and vaporized="teleport". Such a trip arrived at that time. Its
  teleport stays in P, because summary `teleports` counted it when it began. Only that event
  counts, and only when SUMO's log reports it for the same vehicle, time and edge as the record.
  Every other removal is unfinished: vaporized="end" (still driving at T), any other reason, or a
  record without the matching report. An ordinary teleport does not end a trip: the vehicle is
  reinserted, then arrives normally or is still driving at T.
* Waiting time is NOT in J (D7): SUMO's `timeLoss` already contains it. It is still computed here
  with the contract's definition (accumulated `waitingTime` per completed vehicle) for reporting.

Weights (D8). Emergency response gets 30% as a deliberate thesis-priority decision: improving
emergency response is the central goal of this thesis. The other 70% keeps the original
traffic-category proportions travel : delay : completion : gridlock = 40 : 30 : 8 : 2, where
delay is the former waiting 20% plus congestion 10%, now carried entirely by time loss.
"""

from __future__ import annotations

import re
import statistics
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Tuple
from xml.etree import ElementTree as ET

# D8. The order of the traffic terms is travel, time loss, incomplete, teleports = 40:30:8:2 x 0.70.
EMERGENCY_SHARE = 0.30
OBJECTIVE_WEIGHTS: Dict[str, float] = {
    "travel": 0.35,
    "response": 0.30,
    "time_loss": 0.2625,
    "incomplete": 0.07,
    "teleports": 0.0175,
}
if abs(sum(OBJECTIVE_WEIGHTS.values()) - 1.0) > 1e-12:  # pragma: no cover - guards an edit
    raise ValueError("objective weights must sum to 1")


def scheduled_population(route_files: Iterable[Path | str], sim_time: float) -> Dict[str, float]:
    """Every vehicle the run is scheduled to depart by `sim_time`: id -> scheduled departure.

    Taken from the route files the run actually loads, so it does not depend on how far ahead
    SUMO happens to read them.
    """
    population: Dict[str, float] = {}
    for route_file in route_files:
        for element in ET.parse(str(route_file)).getroot().iter("vehicle"):
            depart = float(element.get("depart"))
            if depart <= sim_time:
                population[element.get("id")] = depart
    return population


# SUMO 1.25.0, MSVehicleTransfer: the only warning written where a trip ends with reason
# TELEPORT_ARRIVED, which is the reason tripinfo writes as vaporized="teleport".
TELEPORT_BEYOND_ARRIVAL = re.compile(
    r"Warning: Vehicle '(?P<vehicle>[^']+)' teleports beyond arrival edge '(?P<edge>[^']+)', "
    r"time=(?P<time>[0-9.]+)\.")


def teleports_beyond_arrival(sumo_log: Path | str | None) -> Dict[str, Tuple[str, float]]:
    """SUMO's own reports of "teleports beyond arrival edge" in a run's --error-log:
    vehicle -> (edge, time)."""
    reported: Dict[str, Tuple[str, float]] = {}
    if sumo_log is None:
        return reported
    for line in Path(sumo_log).read_text(encoding="utf-8", errors="replace").splitlines():
        match = TELEPORT_BEYOND_ARRIVAL.fullmatch(line.strip())
        if match:
            reported[match.group("vehicle")] = (match.group("edge"), float(match.group("time")))
    return reported


def ended_beyond_arrival(record: Mapping, reported: Mapping[str, Tuple[str, float]]) -> bool:
    """True only when a tripinfo record and SUMO's report describe that event together. The record
    says vaporized="teleport" with a real arrival time, and the log reports the same vehicle
    teleporting beyond its arrival edge at that time, on the edge of the record's arrival lane."""
    event = reported.get(record.get("id"))
    if event is None or record.get("vaporized") != "teleport":
        return False
    edge, time = event
    arrival = float(record["arrival"])
    return arrival >= 0 and arrival == time and record.get("arrivalLane", "").rsplit("_", 1)[0] == edge


def _responses(responses: Mapping, sim_time: float) -> Dict[str, dict]:
    """D3: one value per incident that started before T."""
    out: Dict[str, dict] = {}
    for event_id, state in responses.items():
        start = float(state["start_time"])
        if start >= sim_time:
            continue                          # never activated in this run
        valid = state.get("response_time_s")
        if valid is not None:
            status = "valid_post_window" if state.get("arrived_after_incident_end") else "valid"
            value = float(valid)
        else:
            status = "teleported" if state.get("responder_teleported") else "not_reached"
            value = float(sim_time) - start   # censored: T - activation
        out[str(event_id)] = {"start_time": start, "value_s": value, "status": status}
    return out


def run_metrics(
    tripinfo_path: Path | str,
    summary_path: Path | str,
    population: Mapping[str, float],
    sim_time: float,
    responses: Mapping,
    sumo_log: Path | str | None = None,
) -> Dict[str, object]:
    """Every quantity J needs, plus the contract's reported definitions next to them.

    `tripinfo_path` must have been written with `--tripinfo-output.write-unfinished`. `sumo_log`
    is the run's SUMO --error-log. Without it no trip is recognised as ended by a teleport beyond
    its arrival edge: such a trip counts as unfinished, finished falls short of summary `arrived`,
    and the tuner rejects the run.
    """
    records = {e.get("id"): e.attrib for e in ET.parse(str(tripinfo_path)).getroot().iter("tripinfo")}
    steps = ET.parse(str(summary_path)).getroot().findall("step")
    last = steps[-1].attrib if steps else {}
    n = len(population)
    if n == 0:
        raise ValueError("no vehicle is scheduled to depart before T")
    reported = teleports_beyond_arrival(sumo_log)

    def finished(record) -> bool:
        if record.get("vaporized"):
            return ended_beyond_arrival(record, reported)   # any other removal is unfinished
        return float(record["arrival"]) >= 0

    travel = []
    for vehicle_id, scheduled in population.items():
        record = records.get(vehicle_id)
        end = float(record["arrival"]) if record is not None and finished(record) else float(sim_time)
        travel.append(end - scheduled)
    in_network = [r for v, r in records.items() if v in population]
    done = [r for r in in_network if finished(r)]
    return {
        "sim_time": float(sim_time),
        "vehicles_scheduled": n,
        "finished": len(done),
        "unfinished_in_network": len(in_network) - len(done),
        "never_inserted": n - len(in_network),
        "records_outside_population": sum(1 for v in records if v not in population),
        "summary_arrived": int(float(last.get("arrived", -1))),
        # Included in `finished`; their teleports are still in `teleports`.
        "finished_by_teleport_beyond_arrival": sorted(
            v for v, r in records.items() if v in population and ended_beyond_arrival(r, reported)),
        # --- the five quantities in J
        "travel_s": statistics.mean(travel),
        "time_loss_s": statistics.mean(float(r["timeLoss"]) for r in in_network) if in_network else 0.0,
        "incomplete_share": 1.0 - len(done) / n,
        "teleports": int(float(last.get("teleports", 0))),
        "teleports_per_vehicle": float(last.get("teleports", 0)) / n,
        "responses": _responses(responses, sim_time),
        # --- reported with the contract's completed-trip definitions; NOT in J
        "reported_waiting_s_completed": statistics.mean(float(r["waitingTime"]) for r in done) if done else None,
        "reported_travel_s_completed": statistics.mean(
            float(r["duration"]) + float(r.get("departDelay", 0.0)) for r in done) if done else None,
        "reported_time_loss_s_completed": statistics.mean(float(r["timeLoss"]) for r in done) if done else None,
    }


def objective(metrics: Mapping, reference: Mapping) -> Dict[str, object]:
    """J and its parts for one run, relative to the reference (baseline) run's metrics."""
    incidents = sorted(reference["responses"])
    if sorted(metrics["responses"]) != incidents:
        raise ValueError(f"incident sets differ: {sorted(metrics['responses'])} vs {incidents}")

    def ratio(value: float, base: float, name: str) -> float:
        if base <= 0:
            raise ValueError(f"reference {name} is {base}; its relative term is undefined")
        return value / base

    relative = {
        "travel": ratio(metrics["travel_s"], reference["travel_s"], "travel"),
        "response": (statistics.mean(
            ratio(metrics["responses"][e]["value_s"], reference["responses"][e]["value_s"], f"response {e}")
            for e in incidents) if incidents else 1.0),
        "time_loss": ratio(metrics["time_loss_s"], reference["time_loss_s"], "time loss"),
        "incomplete": ratio(metrics["incomplete_share"], reference["incomplete_share"], "incomplete share"),
        "teleports": ratio(metrics["teleports_per_vehicle"], reference["teleports_per_vehicle"], "teleports"),
    }
    terms = {name: OBJECTIVE_WEIGHTS[name] * value for name, value in relative.items()}
    return {"J": sum(terms.values()), "relative": relative, "terms": terms,
            "weights": dict(OBJECTIVE_WEIGHTS)}


def check_reference(reference: Mapping, seed: int, sim_time: float,
                    frozen_demand_sha256: str, scenario_sha256: str) -> None:
    """Refuse a reference built for another seed, duration, demand or incident set."""
    problems = []
    if int(reference.get("seed", -1)) != int(seed):
        problems.append(f"seed {reference.get('seed')} != {seed}")
    if float(reference.get("sim_time", -1)) != float(sim_time):
        problems.append(f"sim_time {reference.get('sim_time')} != {sim_time}")
    if reference.get("frozen_demand_sha256") != frozen_demand_sha256:
        problems.append("frozen demand file changed")
    if reference.get("scenario_sha256") != scenario_sha256:
        problems.append("incident scenario file changed")
    if problems:
        raise ValueError("tuning reference does not match this run: " + "; ".join(problems))
