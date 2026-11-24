#!/usr/bin/env python3
"""Two supplementary response metrics over the completed held-out runs (2026-09-16).

**Post-hoc, descriptive, exploratory.** These metrics were NOT pre-registered. They add interpretation to runs that
are already finished; they change no result, select nothing, and the pre-registered held-out report
(`results/heldout_eval_2026-09-16/heldout_report.json`) remains the thesis result and is never touched.

Analysis only: this script runs no simulation. It reads saved `tripinfo.xml`, `trial.json` and `reference.json`
files and writes a JSON and a Markdown table into one new output folder, refusing to overwrite either.

**Metric 1 - responder delay decomposition.** Each response is split into the parts the saved trip record can
support:

    response = dispatch delay + free-flow driving + delay in traffic  (+/- the 1 s recording step)

where the dispatch delay is fixed by the scenario (60 s), free-flow driving is the responder's trip duration minus
its time lost, and the delay is SUMO's `timeLoss` for that responder. The responder's `routeLength` and `rerouteNo`
are reported beside them, because the interpretation depends on whether the route changed at all.

**Metric 2 - percentage change beside seconds.** For every incident and configuration:

    difference_s = baseline_response - treatment_response      (positive = treatment faster)
    ratio        = treatment_response / baseline_response      (as in the pre-registered report)
    percent      = difference_s / baseline_response * 100       (positive = improvement)

A percentage is computed only where the pre-registered censoring rule allows a ratio at all: both the baseline and
the treatment measurement must be valid or valid_post_window. Anything else is reported as undefined, never
substituted, and named in `excluded`.

    python scripts/diagnostics/response_delay_metrics.py --out results/supplementary_metrics_2026-09-16
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
HELDOUT = REPO_ROOT / "results/heldout_eval_2026-09-16"
PLAN = HELDOUT / "plan.json"
SCENARIOS = REPO_ROOT / "scenarios/emergency_scenarios.yaml"
INCIDENTS = ("404", "401", "405")
VALID_RESPONSE_STATUSES = ("valid", "valid_post_window")


def utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ---------------------------------------------------------------- the two metrics (pure, tested)

def delay_decomposition(duration_s: float, time_loss_s: float, dispatch_delay_s: float) -> Dict[str, float]:
    """Split one responder trip. Free-flow driving is what is left of the trip once delay is removed."""
    return {"dispatch_delay_s": float(dispatch_delay_s),
            "free_flow_s": float(duration_s) - float(time_loss_s),
            "delay_s": float(time_loss_s),
            "trip_duration_s": float(duration_s),
            "accounted_response_s": float(dispatch_delay_s) + float(duration_s)}


def percentage_change(baseline_s: Optional[float], treatment_s: Optional[float],
                      baseline_status: Optional[str] = "valid",
                      treatment_status: Optional[str] = "valid") -> Dict[str, Optional[float]]:
    """Seconds, ratio and percentage, or undefined with a reason. Positive seconds and percentage mean faster."""
    if baseline_s is None or baseline_status not in VALID_RESPONSE_STATUSES:
        return {"difference_s": None, "ratio": None, "percent": None,
                "reason": f"baseline censored or invalid (status {baseline_status})"}
    if treatment_s is None or treatment_status not in VALID_RESPONSE_STATUSES:
        return {"difference_s": None, "ratio": None, "percent": None,
                "reason": f"treatment censored or invalid (status {treatment_status})"}
    if float(baseline_s) == 0.0:
        return {"difference_s": None, "ratio": None, "percent": None, "reason": "baseline response is zero"}
    difference = float(baseline_s) - float(treatment_s)
    return {"difference_s": difference, "ratio": float(treatment_s) / float(baseline_s),
            "percent": difference / float(baseline_s) * 100.0, "reason": "defined"}


def summarise(values: List[Optional[float]]) -> Dict[str, Optional[float]]:
    defined = [v for v in values if v is not None]
    if not defined:
        return {"n": 0, "mean": None, "median": None, "min": None, "max": None}
    return {"n": len(defined), "mean": statistics.mean(defined), "median": statistics.median(defined),
            "min": min(defined), "max": max(defined)}


# ---------------------------------------------------------------- reading the saved runs

def scenario_dispatch_delays() -> Dict[str, float]:
    import yaml
    document = yaml.safe_load(SCENARIOS.read_text(encoding="utf-8"))
    delays = {}
    for event in document["events"]:
        incident = str(event["event_id"])
        delays[incident] = float(event["dispatch"]["departure_time"]) - float(event["start_time"])
    return delays


def responder_trips(tripinfo: Path) -> Dict[str, dict]:
    out = {}
    for element in ET.parse(tripinfo).getroot().iter("tripinfo"):
        identifier = element.get("id", "")
        if identifier.startswith("emergency_dispatch_"):
            out[identifier.split("_")[-1]] = {
                "duration_s": float(element.get("duration")), "time_loss_s": float(element.get("timeLoss")),
                "route_length_m": float(element.get("routeLength")), "reroutes": int(element.get("rerouteNo", "0")),
                "waiting_s": float(element.get("waitingTime")), "stops": int(element.get("waitingCount", "0")),
                "departed_s": float(element.get("depart")), "arrived_s": float(element.get("arrival"))}
    return out


def responses_of(metrics: dict) -> Dict[str, dict]:
    return {k: {"value_s": v["value_s"], "status": v["status"]} for k, v in metrics["responses"].items()}


def collect(seeds: List[int], configurations: List[str]) -> dict:
    dispatch = scenario_dispatch_delays()
    per_seed = {}
    for seed in seeds:
        reference = json.loads((REPO_ROOT / f"results/tuning_reference/seed_{seed}_3600s/reference.json")
                               .read_text(encoding="utf-8"))
        arms = {"baseline": {"trips": responder_trips(REPO_ROOT / f"results/tuning_reference/seed_{seed}_3600s/tripinfo.xml"),
                             "responses": responses_of(reference["metrics"])}}
        for label in configurations:
            run = HELDOUT / f"runs/{label}_seed_{seed}/trial"
            trial = json.loads((run / "trial.json").read_text(encoding="utf-8"))
            arms[label] = {"trips": responder_trips(run / "tripinfo.xml"),
                           "responses": responses_of(trial["metrics"])}
        incidents = {}
        for incident in INCIDENTS:
            entry = {"dispatch_delay_s": dispatch[incident], "arms": {}, "against_baseline": {}}
            for label, arm in arms.items():
                trip = arm["trips"].get(incident)
                response = arm["responses"].get(incident, {})
                block = {"response_s": response.get("value_s"), "response_status": response.get("status")}
                if trip:
                    block.update(delay_decomposition(trip["duration_s"], trip["time_loss_s"], dispatch[incident]))
                    block.update({"route_length_m": trip["route_length_m"], "responder_reroutes": trip["reroutes"],
                                  "waiting_s": trip["waiting_s"], "stops": trip["stops"]})
                entry["arms"][label] = block
            base = entry["arms"]["baseline"]
            for label in configurations:
                treatment = entry["arms"][label]
                change = percentage_change(base.get("response_s"), treatment.get("response_s"),
                                           base.get("response_status"), treatment.get("response_status"))
                if base.get("delay_s") is not None and treatment.get("delay_s") is not None and base["delay_s"] > 0:
                    change["delay_removed_s"] = base["delay_s"] - treatment["delay_s"]
                    change["delay_removed_percent"] = change["delay_removed_s"] / base["delay_s"] * 100.0
                    change["route_length_unchanged"] = (abs(base["route_length_m"] - treatment["route_length_m"])
                                                        < 5.0)
                entry["against_baseline"][label] = change
            incidents[incident] = entry
        per_seed[str(seed)] = incidents
    return per_seed


def aggregate(per_seed: dict, configurations: List[str]) -> dict:
    out = {}
    for label in configurations:
        by_incident = {}
        for incident in INCIDENTS:
            percents, delays, seconds = [], [], []
            for seed in per_seed:
                change = per_seed[seed][incident]["against_baseline"][label]
                percents.append(change.get("percent"))
                seconds.append(change.get("difference_s"))
                delays.append(change.get("delay_removed_percent"))
            by_incident[incident] = {"response_percent": summarise(percents),
                                     "response_seconds": summarise(seconds),
                                     "delay_removed_percent": summarise(delays)}
        every = [per_seed[s][i]["against_baseline"][label].get("percent") for s in per_seed for i in INCIDENTS]
        by_incident["all_measurements"] = {"response_percent": summarise(every)}
        out[label] = by_incident
    return out


# ---------------------------------------------------------------- output

def markdown(per_seed: dict, totals: dict, configurations: List[str]) -> str:
    final, sensitivity = configurations[0], configurations[1]
    lines = [
        "# Supplementary response metrics (post-hoc, descriptive)",
        "",
        "**Not pre-registered.** These two metrics add interpretation to the completed held-out runs. They change",
        "no result and select nothing; `results/heldout_eval_2026-09-16/heldout_report.json` remains the thesis",
        "result. No simulation was run to produce this file.",
        "",
        "Sign convention, as in the pre-registered report: **positive seconds and positive percentages mean the",
        "configuration was faster than that seed's own no-rerouting baseline.**",
        "",
        "## Metric 1 - responder delay decomposition",
        "",
        "`response = dispatch delay + free-flow driving + delay in traffic`, from the responder's own trip record.",
        "",
        "| seed | incident | route (m) base / F / S | responder reroutes | free-flow (s) base / F / S | delay (s) base / F / S |",
        "|---|---|---|---|---|---|",
    ]
    for seed in per_seed:
        for incident in INCIDENTS:
            arms = per_seed[seed][incident]["arms"]
            b, f, s = arms["baseline"], arms[final], arms[sensitivity]
            lines.append(
                f"| {seed} | {incident} | {b['route_length_m']:.0f} / {f['route_length_m']:.0f} / "
                f"{s['route_length_m']:.0f} | {b['responder_reroutes']} / {f['responder_reroutes']} / "
                f"{s['responder_reroutes']} | {b['free_flow_s']:.1f} / {f['free_flow_s']:.1f} / "
                f"{s['free_flow_s']:.1f} | {b['delay_s']:.1f} / {f['delay_s']:.1f} / {s['delay_s']:.1f} |")
    lines += ["", f"F = {final}, S = {sensitivity}. The dispatch delay is 60 s for all three incidents.", "",
              "## Metric 2 - percentage change beside seconds", "",
              "| seed | incident | baseline (s) | " + " | ".join(
                  f"{label} (s) | diff | % | ratio" for label in configurations) + " |",
              "|---|---|---|" + "---|---|---|---|" * len(configurations)]
    for seed in per_seed:
        for incident in INCIDENTS:
            entry = per_seed[seed][incident]
            row = [seed, incident, f"{entry['arms']['baseline']['response_s']:.0f}"]
            for label in configurations:
                arm, change = entry["arms"][label], entry["against_baseline"][label]
                if change["percent"] is None:
                    row += [f"{arm['response_s']:.0f}", "undefined", f"({change['reason']})", "undefined"]
                else:
                    row += [f"{arm['response_s']:.0f}", f"{change['difference_s']:+.0f} s",
                            f"{change['percent']:+.1f}%", f"{change['ratio']:.3f}"]
            lines.append("| " + " | ".join(row) + " |")
    lines += ["", "## Aggregates (descriptive only)", "",
              "| configuration | incident | mean % | median % | mean seconds | mean delay removed % |",
              "|---|---|---|---|---|---|"]
    for label in configurations:
        for incident in list(INCIDENTS) + ["all_measurements"]:
            block = totals[label][incident]
            percent, seconds = block["response_percent"], block.get("response_seconds")
            delay = block.get("delay_removed_percent")
            def fmt(value, suffix=""):
                return "-" if value is None else f"{value:+.1f}{suffix}"
            lines.append(f"| {label} | {incident} | {fmt(percent['mean'], '%')} | {fmt(percent['median'], '%')} | "
                         f"{fmt(seconds['mean'] if seconds else None, ' s')} | "
                         f"{fmt(delay['mean'] if delay else None, '%')} |")
    lines += ["", "Undefined ratios, if any, are excluded from every aggregate and named in the table above.", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", default="results/supplementary_metrics_2026-09-16")
    args = parser.parse_args()
    out = Path(args.out).resolve()
    json_path, markdown_path = out / "supplementary_metrics.json", out / "supplementary_metrics.md"
    for path in (json_path, markdown_path):
        if path.exists():
            raise SystemExit(f"{path} exists; analysis output is evidence and is never overwritten")
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    seeds = [int(s) for s in plan["seeds"]]
    configurations = [plan["final_configuration"], plan["sensitivity_configuration"]]
    per_seed = collect(seeds, configurations)
    totals = aggregate(per_seed, configurations)
    out.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(
        {"kind": "supplementary response metrics (post-hoc, descriptive, not pre-registered)",
         "written_utc": utc(), "source": "results/heldout_eval_2026-09-16 (completed runs); no simulation was run",
         "pre_registered_report_unchanged": "results/heldout_eval_2026-09-16/heldout_report.json",
         "sign_convention": "positive seconds and percentages mean faster than that seed's own baseline",
         "seeds": seeds, "configurations": configurations, "per_seed": per_seed, "aggregate": totals},
        indent=1), encoding="utf-8")
    markdown_path.write_text(markdown(per_seed, totals, configurations), encoding="utf-8")
    print(f"written: {json_path}\nwritten: {markdown_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
