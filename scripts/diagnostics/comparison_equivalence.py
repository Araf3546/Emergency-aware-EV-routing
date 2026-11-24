#!/usr/bin/env python3
"""Equivalence proofs for the V1 comparison infrastructure (2026-09-17). Diagnostic only.

Three proofs, each written into a new folder and never over an existing one:

* `e3-on`: the changed `tune_e3_weights.evaluate_weights(..., emergency_layer=True)` is run for T16 at a held-out
  seed and compared with that seed's frozen held-out run. It uses the fields and files the held-out integrity checks
  compare: the 17 `trial.json` fields, J, every tripinfo record (responders included), every summary step without
  SUMO's wall-clock `duration`, every SUMO warning and error line, and tracebacks. The two fields the change adds
  (`emergency_layer`, and the separate `blockage_samples.json`) are reported, not compared.

* `logger`: two `metrics/metrics_logger.py` output folders (the frozen loop and the refactored one) are compared
  file by file: tripinfo records, summary steps without `duration`, SUMO error lines, the metrics JSON with the
  run-environment fields removed, the per-vehicle CSV, the emergency-event log and the text summary.

    python scripts/diagnostics/comparison_equivalence.py e3-on --seed 20260722 --out <new folder>
    python scripts/diagnostics/comparison_equivalence.py logger --old <dir> --new <dir> --out <file.json>
    python scripts/diagnostics/comparison_equivalence.py runner --logger <dir> --method aco --out <new folder>

* `runner`: `tune_e3_weights.evaluate_comparator` (the comparison runner's measurement path) is run for the same
  method, seed and duration as a `metrics_logger` run and must drive the identical simulation: every summary step
  (without `duration`), every finished trip record, every SUMO warning and error line, the reroute counters, the
  route-validity counters and the responses. Only the output set differs (unfinished trips are added).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts"), str(REPO_ROOT / "scripts" / "diagnostics")]


def utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _write_new(path: Path, document: dict) -> None:
    if path.exists():
        raise SystemExit(f"{path} exists; evidence is never overwritten")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=1, default=str), encoding="utf-8")


def compare_trial_dirs(new_trial: Path, original_trial: Path) -> dict:
    """Field-by-field and file-by-file comparison of two tuner trial folders (pure apart from reading)."""
    import heldout_evaluation as H

    a = json.loads((new_trial / "trial.json").read_text(encoding="utf-8"))
    b = json.loads((original_trial / "trial.json").read_text(encoding="utf-8"))
    checks = {}
    for key in H.TRIAL_KEYS:
        checks[f"trial.json:{key}"] = a.get(key) == b.get(key)
    checks["J"] = (a.get("objective") or {}).get("J") == (b.get("objective") or {}).get("J")
    checks["tripinfo records"] = H._records(new_trial / "tripinfo.xml", "tripinfo") == \
        H._records(original_trial / "tripinfo.xml", "tripinfo")
    checks["summary steps (without wall-clock duration)"] = \
        H._records(new_trial / "summary.xml", "step", ignore=("duration",)) == \
        H._records(original_trial / "summary.xml", "step", ignore=("duration",))
    checks["SUMO warning and error lines"] = H._sumo_lines(new_trial / "errors.log") == \
        H._sumo_lines(original_trial / "errors.log")
    checks["no traceback in the new SUMO log"] = H.tracebacks(new_trial / "errors.log") == 0
    differing = sorted(k for k, ok in checks.items() if not ok)
    return {"checks": checks, "differing": differing, "identical": not differing,
            "added_fields": {"emergency_layer": a.get("emergency_layer"), "evaluator": a.get("evaluator"),
                             "blockage_samples_file": (new_trial / "blockage_samples.json").is_file()},
            "counts": {"tripinfo_records": len(H._records(new_trial / "tripinfo.xml", "tripinfo")),
                       "summary_steps": len(H._records(new_trial / "summary.xml", "step")),
                       "sumo_lines": len(H._sumo_lines(new_trial / "errors.log"))}}


def cmd_e3_on(args) -> int:
    os.chdir(REPO_ROOT)
    import heldout_evaluation as H
    import tune_e3_weights as tuner

    seed, out = int(args.seed), Path(args.out).resolve()
    if out.exists():
        raise SystemExit(f"{out} exists; evidence is never overwritten")
    original = REPO_ROOT / f"results/heldout_eval_2026-09-16/runs/trial_016_seed_{seed}/trial"
    weights = H.candidate_weights("trial_016")
    if weights != H.campaign_trial_weights("trial_016"):
        raise SystemExit("T16 weight file does not hold the campaign weights")
    reference = tuner.load_reference(H.reference_path(seed), seed, H.SIM_TIME)
    started, clock = utc(), time.time()
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), H.SIM_TIME, seed, trial_dir=out / "trial",
                               reference=reference, emergency_layer=True)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    result = compare_trial_dirs(out / "trial", original)
    result.update({"kind": "emergency_layer=True equivalence against the frozen held-out T16 run",
                   "seed": seed, "original": original.relative_to(REPO_ROOT).as_posix(), "rejected": rejected,
                   "pythonhashseed": os.environ.get("PYTHONHASHSEED", "unset"), "started_utc": started,
                   "finished_utc": utc(), "wall_seconds": round(time.time() - clock, 1)})
    _write_new(out / "equivalence.json", result)
    print(json.dumps({k: result[k] for k in ("identical", "differing", "added_fields", "counts", "rejected")}))
    return 0 if result["identical"] and rejected is None else 1


LOGGER_ENV_FIELDS = ("pythonhashseed",)


def compare_logger_dirs(old: Path, new: Path, algo: str) -> dict:
    import csv
    import heldout_evaluation as H

    def metrics(d):
        m = json.loads((d / f"{algo}_metrics.json").read_text(encoding="utf-8"))
        for key in LOGGER_ENV_FIELDS:
            m.get("route_stats", {}).pop(key, None)
        return m

    def rows(p):
        with open(p, newline="") as fh:
            return list(csv.reader(fh))

    checks = {
        "tripinfo records": H._records(old / "tripinfo.xml", "tripinfo") == H._records(new / "tripinfo.xml", "tripinfo"),
        "summary steps (without wall-clock duration)":
            H._records(old / "summary.xml", "step", ignore=("duration",)) ==
            H._records(new / "summary.xml", "step", ignore=("duration",)),
        "SUMO warning and error lines": H._sumo_lines(old / "sumo_errors.log") == H._sumo_lines(new / "sumo_errors.log"),
        "metrics JSON (route stats, counters, trip metrics)": metrics(old) == metrics(new),
        "per-vehicle CSV": rows(old / f"{algo}_results.csv") == rows(new / f"{algo}_results.csv"),
        "emergency event log": (old / f"{algo}_emergency_events.jsonl").read_text() ==
                               (new / f"{algo}_emergency_events.jsonl").read_text(),
        "text summary": (old / f"{algo}_summary.txt").read_text() == (new / f"{algo}_summary.txt").read_text(),
    }
    m = metrics(new)
    return {"algorithm": algo, "checks": checks, "identical": all(checks.values()),
            "differing": sorted(k for k, ok in checks.items() if not ok),
            "reroutes_applied": m.get("reroutes_applied"), "reroute_attempts": m.get("reroute_attempts"),
            "completed_trips": m.get("trip_metrics", {}).get("completed_trips")}


def cmd_logger(args) -> int:
    results = {}
    for algo in args.algos.split(","):
        results[algo] = compare_logger_dirs(Path(args.old) / "results" / algo / f"seed_{args.seed}",
                                            Path(args.new) / "results" / algo / f"seed_{args.seed}", algo)
    document = {"kind": "metrics_logger routing-loop refactor equivalence (frozen loop vs shared loop)",
                "seed": args.seed, "duration_s": args.duration, "old": str(args.old), "new": str(args.new),
                "utc": utc(), "identical": all(r["identical"] for r in results.values()), "algorithms": results}
    _write_new(Path(args.out), document)
    print(json.dumps({a: (r["identical"], r["differing"], r["reroutes_applied"]) for a, r in results.items()}))
    return 0 if document["identical"] else 1


def cmd_runner(args) -> int:
    os.chdir(REPO_ROOT)
    import heldout_evaluation as H
    import tune_e3_weights as tuner

    out = Path(args.out).resolve()
    if out.exists():
        raise SystemExit(f"{out} exists; evidence is never overwritten")
    logger = Path(args.logger) / "results" / args.method / f"seed_{args.seed}"
    clock = time.time()
    tuner.evaluate_comparator(args.method, Path("simulation.sumocfg"), args.duration, args.seed, trial_dir=out / "trial")
    trial = json.loads((out / "trial" / "trial.json").read_text(encoding="utf-8"))
    logged = json.loads((logger / f"{args.method}_metrics.json").read_text(encoding="utf-8"))
    finished_new = [r for r in H._records(out / "trial" / "tripinfo.xml", "tripinfo") if float(r[0]["arrival"]) >= 0]
    env = ("route_choice_seed", "pythonhashseed")
    events = [json.loads(line) for line in (logger / f"{args.method}_emergency_events.jsonl").read_text().splitlines()]
    checks = {
        "summary steps (without wall-clock duration)":
            H._records(out / "trial" / "summary.xml", "step", ignore=("duration",)) ==
            H._records(logger / "summary.xml", "step", ignore=("duration",)),
        # The tuner's SUMO command adds --emission-output, so every tripinfo record also lists an emissions device
        # and carries an <emissions> child (diagnosed 2026-09-17, runner_vs_logger_1200s/DIAGNOSIS.json). The trip
        # values themselves must be identical.
        "finished trip values (emissions-device output removed)":
            [{k: v for k, v in r[0].items() if k != "devices"} for r in finished_new] ==
            [{k: v for k, v in r[0].items() if k != "devices"}
             for r in H._records(logger / "tripinfo.xml", "tripinfo")],
        "SUMO warning and error lines": H._sumo_lines(out / "trial" / "errors.log") == H._sumo_lines(logger / "sumo_errors.log"),
        "reroute counters": {k: logged[k] for k in trial["routing"]} == trial["routing"],
        "route-validity counters": {k: v for k, v in trial["route_stats"].items() if k not in env} ==
            {k: v for k, v in logged["route_stats"].items() if isinstance(v, (int, float)) and k not in env},
        "responses": {str(e["event_id"]): e["response_time"] for e in events} ==
            {k: v["value_s"] for k, v in trial["metrics"]["responses"].items() if v["value_s"] is not None
             and v["status"] in ("valid", "valid_post_window")},
    }
    document = {"kind": "evaluate_comparator drives the same simulation as metrics_logger", "method": args.method,
                "seed": args.seed, "duration_s": args.duration, "logger_run": str(logger), "utc": utc(),
                "checks": checks, "identical": all(checks.values()),
                "differing": sorted(k for k, ok in checks.items() if not ok),
                "wall_seconds": round(time.time() - clock, 1),
                "pythonhashseed": os.environ.get("PYTHONHASHSEED", "unset")}
    _write_new(out / "equivalence.json", document)
    print(json.dumps({"method": args.method, "identical": document["identical"], "differing": document["differing"]}))
    return 0 if document["identical"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    e = sub.add_parser("e3-on")
    e.add_argument("--seed", type=int, required=True)
    e.add_argument("--out", required=True)
    g = sub.add_parser("logger")
    g.add_argument("--old", required=True)
    g.add_argument("--new", required=True)
    g.add_argument("--algos", default="dijkstra,astar,aco,aco_pso,e3")
    g.add_argument("--seed", type=int, default=20260722)
    g.add_argument("--duration", type=int, default=1200)
    g.add_argument("--out", required=True)
    n = sub.add_parser("runner")
    n.add_argument("--logger", required=True)
    n.add_argument("--method", required=True)
    n.add_argument("--seed", type=int, default=20260722)
    n.add_argument("--duration", type=int, default=1200)
    n.add_argument("--out", required=True)
    args = parser.parse_args()
    return {"e3-on": cmd_e3_on, "logger": cmd_logger, "runner": cmd_runner}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
