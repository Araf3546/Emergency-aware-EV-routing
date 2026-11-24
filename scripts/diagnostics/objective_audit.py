#!/usr/bin/env python3
"""Tuning-objective design audit (2026-09-13). Reads saved runs only - no simulation.

    python scripts/diagnostics/objective_audit.py --out results/objective_audit_2026-09-13

1. Rebuilds the CURRENT tuner objective from a run's SUMO files and checks the rebuild against
   the one run whose objective the tuner itself recorded (the 1,800 s identity run).
2. Splits that objective into its six terms: share of the total, and - using the two smoke
   trials - share of the difference between two weight sets (what actually ranks trials).
3. Evaluates the current objective and a contract-aligned candidate on the saved full-hour runs
   at the tuning seed (baseline, E3 layer OFF, E3 layer ON).
4. Quantifies the survivorship loophole: when does the objective prefer a slow trip to stay
   unfinished?
"""

from __future__ import annotations

import argparse
import json
import os
import statistics as st
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)

W_CURRENT = {"travel": 0.40, "waiting": 0.20, "response": 0.20, "congestion": 0.10,
             "incomplete": 0.08, "teleports": 0.02}
IDENTITY = Path("results/tuner_smoke_2026-09-12/identity_1800")
SMOKE = Path("results/tuner_smoke_2026-09-12/cli_2x700")
FULL = {
    "baseline (no rerouting)": Path("results/trigger_comparison_2026-09-12/full_off_s21/baseline"),
    "E3, emergency layer OFF": Path("results/trigger_comparison_2026-09-12/full_off_s21/current"),
    "E3, emergency layer ON": Path("results/trigger_comparison_2026-09-12/full_on_s21/current"),
}
SEED = 20260721


def tripinfo(path: Path) -> dict:
    return {e.get("id"): e.attrib for e in ET.parse(path).getroot().iter("tripinfo")}


def summary_steps(path: Path) -> list:
    return [s.attrib for s in ET.parse(path).getroot().iter("step")]


def frozen_departures() -> dict:
    from evaluation_seeds import frozen_demand_paths
    return {e.get("id"): float(e.get("depart"))
            for e in ET.parse(frozen_demand_paths(SEED)[0]).getroot().iter("vehicle")}


def all_scheduled(duration: int) -> dict:
    """Every vehicle SUMO would load by `duration`: ordinary, ambient emergency, responders."""
    out = {}
    for f in ("demand/porto.emergency.rou.xml", "demand/porto.dispatch.rou.xml"):
        for e in ET.parse(f).getroot().iter("vehicle"):
            if float(e.get("depart")) <= duration:
                out[e.get("id")] = float(e.get("depart"))
    return out


def rebuild_current(run: Path, duration: int, responses: dict, waiting=0.0) -> dict:
    """The current objective's six inputs, rebuilt from SUMO files.

    Inputs the files cannot give exactly: waiting (the tuner reads getWaitingTime at the last step
    a vehicle is seen - about 0; passed in) and the count of emergency vehicles the tuner saw.
    """
    trips = tripinfo(run / "tripinfo.xml")
    steps = summary_steps(run / "summary.xml")
    ordinary = {k: v for k, v in frozen_departures().items() if v <= duration}
    travel = []
    for vid, t in trips.items():
        if vid in ordinary:     # timed from scheduled departure, as the tuner does
            travel.append(float(t["arrival"]) - ordinary[vid])
        else:                   # timed from first sighting = actual insertion
            travel.append(float(t["arrival"]) - float(t["depart"]))
    speeds = [float(s["meanSpeed"]) for s in steps
              if float(s["time"]) % 30 == 0 and float(s.get("running", 0)) > 0 and float(s["meanSpeed"]) >= 0]
    congestion = st.mean(max(0.0, min(1.0, 1.0 - v / 15.0)) for v in speeds)
    cumulative = [float(s["teleports"]) for s in steps]
    per_step = [b - a for a, b in zip([0.0] + cumulative[:-1], cumulative)]
    completed_ordinary = sum(1 for k in trips if k in ordinary)
    incomplete = len(ordinary) - completed_ordinary
    emergency_seen = len(all_scheduled(duration))   # approximation: every non-ordinary vehicle inserted
    n = len(ordinary) + emergency_seen
    valid = [v for v in responses.values() if v is not None]
    inputs = {
        "travel": st.mean(travel), "waiting": waiting,
        "response": st.mean(valid) if valid else float(duration),
        "congestion": congestion, "incomplete": incomplete, "teleports": max(per_step),
        "vehicle_count": n, "completed": len(trips), "teleports_total": cumulative[-1],
    }
    terms = {
        "travel": W_CURRENT["travel"] * inputs["travel"] / duration,
        "waiting": W_CURRENT["waiting"] * inputs["waiting"] / duration,
        "response": W_CURRENT["response"] * inputs["response"] / duration,
        "congestion": W_CURRENT["congestion"] * inputs["congestion"],
        "incomplete": W_CURRENT["incomplete"] * incomplete / n,
        "teleports": W_CURRENT["teleports"] * inputs["teleports"] / n,
    }
    return {"inputs": inputs, "terms": terms, "objective": sum(terms.values())}


def contract_metrics(run: Path, duration: int, responses_full: dict) -> dict:
    """The contract's own definitions (metrics/trip_metrics.py), plus censored responses."""
    from metrics.trip_metrics import aggregate, load_summary, load_tripinfo
    trips = load_tripinfo(run / "tripinfo.xml")
    summ = load_summary(run / "summary.xml")
    agg = aggregate(trips, summ)
    per_event = {}
    for event_id, state in responses_full.items():
        valid = state.get("response_time_s")
        per_event[event_id] = {
            "valid_s": valid,
            "censored_s": valid if valid is not None else duration - float(state["start_time"]),
            "post_window": bool(state.get("arrived_after_incident_end")),
            "status": ("valid" if valid is not None else
                       "teleported" if state.get("responder_teleported") else "not reached"),
        }
    return {
        "travel_s": agg["mean_travel_time_s"], "waiting_s": agg["mean_waiting_time_s"],
        "time_loss_s": agg["mean_time_loss_s"], "completed": agg["completed_trips"],
        "loaded": agg["loaded_vehicles"], "incomplete_share": 1.0 - agg["completion_rate"],
        "teleports_total": agg["teleports_total"], "teleports_per_loaded": agg["teleports_total"] / agg["loaded_vehicles"],
        "responses": per_event,
    }


def candidate(m: dict, ref: dict) -> dict:
    """Contract-aligned candidate: every term relative to the no-rerouting baseline (= 1.0)."""
    events = sorted(ref["responses"])
    r = st.mean(m["responses"][e]["censored_s"] / ref["responses"][e]["censored_s"] for e in events)
    rel = {
        "travel": m["travel_s"] / ref["travel_s"], "waiting": m["waiting_s"] / ref["waiting_s"],
        "response": r, "congestion": m["time_loss_s"] / ref["time_loss_s"],
        "incomplete": m["incomplete_share"] / ref["incomplete_share"],
        "teleports": m["teleports_per_loaded"] / ref["teleports_per_loaded"],
    }
    terms = {k: W_CURRENT[k] * v for k, v in rel.items()}
    return {"relative": rel, "terms": terms, "objective": sum(terms.values())}


def shares(terms: dict) -> dict:
    total = sum(terms.values())
    return {k: round(100.0 * v / total, 2) for k, v in terms.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    out = Path(parser.parse_args().out)
    out.mkdir(parents=True, exist_ok=True)
    report = {}

    # 1. rebuild check on the identity run, where the tuner recorded the true value
    ident_metrics = json.loads((IDENTITY / "metrics.json").read_text(encoding="utf-8"))
    ident_trial = json.loads((IDENTITY / "trial.json").read_text(encoding="utf-8"))
    rebuilt = rebuild_current(IDENTITY, 1800, ident_trial["responses"], ident_metrics["average_waiting_time"])
    exact_terms = {
        "travel": 0.40 * ident_metrics["average_travel_time"] / 1800,
        "waiting": 0.20 * ident_metrics["average_waiting_time"] / 1800,
        "response": 0.20 * ident_metrics["average_emergency_response_time"] / 1800,
        "congestion": 0.10 * ident_metrics["congestion"],
    }
    rest = ident_metrics["objective"] - sum(exact_terms.values())
    n_exact = (0.08 * ident_metrics["incomplete_vehicles"] + 0.02 * ident_metrics["teleports"]) / rest
    exact_terms["incomplete"] = 0.08 * ident_metrics["incomplete_vehicles"] / n_exact
    exact_terms["teleports"] = 0.02 * ident_metrics["teleports"] / n_exact
    report["identity_1800"] = {
        "objective_recorded": ident_metrics["objective"],
        "objective_rebuilt_from_files": rebuilt["objective"],
        "rebuild_error_pct": round(100 * (rebuilt["objective"] - ident_metrics["objective"]) / ident_metrics["objective"], 3),
        "inputs_recorded": ident_metrics, "inputs_rebuilt": rebuilt["inputs"],
        "vehicle_count_implied_by_recorded_objective": n_exact,
        "terms_exact": exact_terms, "share_of_objective_pct": shares(exact_terms),
        "sumo_truth": {k: v for k, v in contract_metrics(IDENTITY, 1800, {}).items() if k != "responses"},
    }

    # 2. what separates two weight sets: the two 700 s smoke trials
    import csv
    rows = list(csv.DictReader((SMOKE / "e3_trials.csv").open(encoding="utf-8")))
    t = [{k: float(r[k]) for k in r if r[k] not in ("", None)} for r in rows]
    n = [t[i]["completed_vehicles"] + t[i]["incomplete_vehicles"] for i in (0, 1)]
    diff = {
        "travel": 0.40 * (t[0]["average_travel_time"] - t[1]["average_travel_time"]) / 700,
        "waiting": 0.20 * (t[0]["average_waiting_time"] - t[1]["average_waiting_time"]) / 700,
        "response": 0.20 * (t[0]["average_emergency_response_time"] - t[1]["average_emergency_response_time"]) / 700,
        "congestion": 0.10 * (t[0]["congestion"] - t[1]["congestion"]),
    }
    diff["incomplete+teleports"] = (t[0]["value"] - t[1]["value"]) - sum(diff.values())
    report["smoke_trials_700"] = {
        "objective_difference": t[0]["value"] - t[1]["value"],
        "difference_by_term": diff, "share_of_difference_pct": shares(diff),
        "note": "two trials only, 700 s, no responder reached its incident (response constant at 700)",
    }

    # 3. the saved full-hour runs at the tuning seed
    full = {}
    for name, run in FULL.items():
        s = json.loads((run / "summary.json").read_text(encoding="utf-8"))
        responses = s["emergency"]["responses"]
        cur = rebuild_current(run, 3600, {k: v.get("response_time_s") for k, v in responses.items()})
        full[name] = {"current": cur, "current_share_pct": shares(cur["terms"]),
                      "contract": contract_metrics(run, 3600, responses)}
    ref = full["baseline (no rerouting)"]["contract"]
    for name in full:
        full[name]["candidate"] = candidate(full[name]["contract"], ref)
    report["full_hour_seed_20260721"] = full

    # 4. survivorship: extra seconds (x) above the mean at which a trip is "cheaper" unfinished
    b = full["E3, emergency layer ON"]["contract"]
    n_done, loaded = b["completed"], b["loaded"]
    cur_break_even = 0.08 * 3600 * (n_done - 1) / (0.40 * full["E3, emergency layer ON"]["current"]["inputs"]["vehicle_count"])
    cand_break_even = (0.08 / loaded / ref["incomplete_share"]) * (n_done - 1) * ref["travel_s"] / 0.40
    report["survivorship"] = {
        "current_objective_break_even_extra_travel_s": round(cur_break_even, 1),
        "candidate_completed_only_break_even_extra_travel_s": round(cand_break_even, 1),
        "meaning": "a trip whose travel time would exceed the mean by more than this lowers the "
                   "objective by NOT finishing (travel term only; waiting and time loss make it worse)",
    }
    (out / "objective_audit.json").write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("identity_1800", "smoke_trials_700", "survivorship")}, indent=1, default=str))
    for name, v in full.items():
        print(name, "current", round(v["current"]["objective"], 6), v["current_share_pct"],
              "| candidate", round(v["candidate"]["objective"], 4), {k: round(x, 4) for k, x in v["candidate"]["relative"].items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
