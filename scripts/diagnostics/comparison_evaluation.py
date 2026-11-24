#!/usr/bin/env python3
"""V1 comparison: the core six-method comparison and the emergency-aware extension (2026-09-17).

Decisions B1, B2 and B5 (`design choices.md`, "COMPARISON-STAGE DECISIONS B1, B2, B5"). Architecture follows
`heldout_evaluation.py`: plan -> run -> validate -> analyse, plus a separate figures step. Nothing is ever
overwritten; every stage refuses an output that already exists.

**Core six-method comparison** (five held-out seeds 20260722-26):
1. no-rerouting baseline - the seed's existing reference (`results/tuning_reference/`), reused;
2. Dijkstra, 3. A*, 4. ACO, 5. ACO-PSO - `tune_e3_weights.evaluate_comparator`, new runs;
6. E3 with T16 weights and the emergency layer OFF - `tune_e3_weights.evaluate_weights(..., emergency_layer=False)`,
   new runs.

**Separate emergency-aware extension** (not part of the six):
7. E3 with T16 weights and the emergency layer ON - the validated held-out runs
   (`results/heldout_eval_2026-09-16/runs/trial_016_seed_<s>/`), reused and never re-run.

Every row is measured with the same code (B5): tripinfo with unfinished vehicles, `metrics.tuning_objective.run_metrics`
(censored travel time, the same completion denominator and teleport definition, the same responder records) and the
frozen `objective` against the seed's own baseline. Completed-only averages are supplementary only.

Section C: C1 (route-validity counters) is in every row's `trial.json`; C2 (blockage samples) is written for the new
E3 rows (the frozen held-out runs did not save them); C4 (five-seed aggregation and figures) is `analyse` and
`figures`. The figures step reads only the finished report, so a plotting failure cannot touch raw evidence.

    python scripts/diagnostics/comparison_evaluation.py plan --out results/comparison_v1_2026-09-17/plan.json
    python scripts/diagnostics/comparison_evaluation.py campaign --plan results/comparison_v1_2026-09-17/plan.json --parallel 3
    python scripts/diagnostics/comparison_evaluation.py validate --plan results/comparison_v1_2026-09-17/plan.json
    python scripts/diagnostics/comparison_evaluation.py analyse --plan results/comparison_v1_2026-09-17/plan.json
    python scripts/diagnostics/comparison_evaluation.py figures --plan results/comparison_v1_2026-09-17/plan.json
    python scripts/diagnostics/comparison_evaluation.py status --plan results/comparison_v1_2026-09-17/plan.json

A timing check is just the campaign restricted to one seed: `campaign ... --seeds 20260722`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
OUT_ROOT = "results/comparison_v1_2026-09-17"
SEEDS = (20260722, 20260723, 20260724, 20260725, 20260726)
SIM_TIME = 3600
INCIDENTS = ("404", "401", "405")
COMPARATORS = ("dijkstra", "astar", "aco", "aco_pso")
E3_OFF = "e3_off_t16"
E3_ON = "e3_on_t16"
BASELINE = "baseline"
CORE = (BASELINE, "dijkstra", "astar", "aco", "aco_pso", E3_OFF)
EXTENSION = (E3_ON,)
NEW_METHODS = COMPARATORS + (E3_OFF,)
LABELS = {BASELINE: "No rerouting (baseline)", "dijkstra": "Dijkstra", "astar": "A*", "aco": "ACO",
          "aco_pso": "ACO-PSO", E3_OFF: "E3 (T16, emergency layer OFF)",
          E3_ON: "E3 (T16, emergency layer ON) - extension"}
FROZEN_TUNING_DIGEST = "c39f2a937667f63c"
VALID_RESPONSE_STATUSES = ("valid", "valid_post_window")
C1_COUNTERS = ("reroute_proposals", "reroutes_applied", "setroute_errors", "setroute_silent_rejections",
               "routes_crossing_closed_edge", "destination_violations", "route_build_failures",
               "no_candidate_edges")
HELDOUT_RUNS = "results/heldout_eval_2026-09-16/runs"
HELDOUT_REPORT = "results/heldout_eval_2026-09-16/heldout_report.json"
WEIGHTS_FILE = "results/final_weight_candidates_2026-09-16/trial_016_weights.json"


def utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel(path: Path) -> str:
    return Path(path).resolve().relative_to(REPO_ROOT).as_posix()


def write_new(path: Path, text: str) -> Path:
    path = Path(path)
    if path.exists():
        raise SystemExit(f"{path} exists; evidence is never overwritten")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def load(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _setup() -> None:
    os.chdir(REPO_ROOT)
    for p in (REPO_ROOT / "scripts" / "diagnostics", REPO_ROOT / "scripts", REPO_ROOT):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))


# ---------------------------------------------------------------- the plan (pure parts are tested)

def run_matrix(seeds: Sequence[int], out_root: str = OUT_ROOT) -> List[dict]:
    """The new runs: for every seed, the four comparators, then E3 with the layer off."""
    return [{"id": f"{m}_seed_{s}", "method": m, "seed": int(s), "out": f"{out_root}/runs/{m}_seed_{s}"}
            for s in seeds for m in NEW_METHODS]


def reused_rows(seeds: Sequence[int]) -> List[dict]:
    """The rows taken from finished, validated evidence: each seed's baseline and its held-out E3 layer-ON run."""
    rows = []
    for s in seeds:
        rows.append({"id": f"{BASELINE}_seed_{s}", "method": BASELINE, "seed": int(s),
                     "reference": f"results/tuning_reference/seed_{s}_{SIM_TIME}s/reference.json",
                     "trial_dir": f"results/tuning_reference/seed_{s}_{SIM_TIME}s"})
        rows.append({"id": f"{E3_ON}_seed_{s}", "method": E3_ON, "seed": int(s),
                     "run": f"{HELDOUT_RUNS}/trial_016_seed_{s}",
                     "trial_dir": f"{HELDOUT_RUNS}/trial_016_seed_{s}/trial"})
    return rows


def comparison_fingerprint(seed: int) -> Dict[str, str]:
    """The code-and-input fingerprint of a comparison run: every project module any row runs, loaded first, so the
    same set is fingerprinted in every process. Diagnostics and tests are excluded, as in the tuner."""
    _setup()
    import p3_runtime  # noqa: F401
    import algorithms.comparator_routing  # noqa: F401
    import metrics.trip_metrics  # noqa: F401
    import metrics.tuning_objective  # noqa: F401
    import p3_simulation_config  # noqa: F401
    import tune_e3_weights as tuner
    return tuner.evaluation_fingerprint(seed, Path("simulation.sumocfg"))


def code_digest(files: Dict[str, str]) -> str:
    """Digest of the fingerprint without any seed's own frozen demand file: one number for the code stage."""
    import heldout_evaluation as H
    kept = {k: v for k, v in files.items() if not H.is_seed_demand(k)}
    return hashlib.sha256(json.dumps(kept, sort_keys=True).encode("utf-8")).hexdigest()[:16]


def cmd_plan(args) -> int:
    _setup()
    import heldout_evaluation as H
    import tune_e3_weights as tuner
    from evaluation_seeds import evaluation_seeds

    out = Path(args.out).resolve()
    if out.exists():
        raise SystemExit(f"{out} exists; a plan is fixed once written")
    if tuple(int(s) for s in evaluation_seeds()) != SEEDS:
        raise SystemExit(f"the contracted evaluation seeds are {evaluation_seeds()}, not {SEEDS}")
    files = comparison_fingerprint(SEEDS[0])
    weights = H.candidate_weights("trial_016")
    if weights != H.campaign_trial_weights("trial_016"):
        raise SystemExit("the T16 weight file does not hold campaign trial 16's weights")
    heldout_report = load(REPO_ROOT / HELDOUT_REPORT)
    if heldout_report.get("valid") is not True:
        raise SystemExit("the held-out report is not valid; the layer-ON rows cannot be reused")
    reused = reused_rows(SEEDS)
    for row in reused:
        if row["method"] == BASELINE:
            ref = REPO_ROOT / row["reference"]
            tuner.load_reference(ref, row["seed"], SIM_TIME)          # seed, duration, demand, scenario, permissions
            row["reference_sha256"] = sha256(ref)
        else:
            run = REPO_ROOT / row["run"]
            meta = load(run / "run.json")
            if meta.get("rejected") or meta.get("configuration") != "trial_016" or meta.get("seed") != row["seed"]:
                raise SystemExit(f"{row['run']} is not a valid T16 held-out run")
            row["run_json_sha256"] = sha256(run / "run.json")
            row["trial_json_sha256"] = sha256(run / "trial" / "trial.json")
            row["code_fingerprint_digest"] = meta["code_fingerprint_digest"]
    document = {
        "kind": "V1 comparison: core six-method comparison plus a separate emergency-aware extension",
        "written_utc": utc(),
        "decisions": "B1, B2, B5 (design choices.md, 2026-09-17)",
        "seeds": list(SEEDS),
        "sim_time": SIM_TIME,
        "core_six_method_comparison": {m: LABELS[m] for m in CORE},
        "emergency_aware_extension": {m: LABELS[m] for m in EXTENSION},
        "naming_rule": "only the six core rows are 'the six-method comparison'; the layer-ON E3 row is an extension",
        "e3_weights_file": WEIGHTS_FILE, "e3_weights_file_sha256": sha256(REPO_ROOT / WEIGHTS_FILE),
        "e3_weights": weights,
        "measurement": ("identical for every row: tripinfo with unfinished vehicles; metrics.tuning_objective.run_metrics "
                        "(censored travel time, completion = finished / scheduled, teleports from summary, responder "
                        "records from the emergency manager); J = the frozen objective against the seed's own "
                        "no-rerouting reference; completed-only averages supplementary only"),
        "planning_horizon_note": ("B1: E3 chooses the next road on live local information and completes the route on "
                                  "free-flow times; Dijkstra and A* plan the whole route on live travel times"),
        "new_runs": run_matrix(SEEDS),
        "reused_rows": reused,
        "code_fingerprint": {"seed_for_file_map": SEEDS[0], "files": files, "code_digest": code_digest(files),
                             "frozen_tuning_chain_digest": FROZEN_TUNING_DIGEST,
                             "rule": ("every run's fingerprint must equal this file map in every entry except that "
                                      "seed's own frozen demand file")},
        "invalid_run_rule": ["any traceback, logged exception, rejection, changed fingerprint, wrong seed, wrong "
                             "duration, wrong weights, wrong layer setting or failed cross-check stops the campaign",
                             "no invalid run is replaced or supplemented",
                             "an interrupted run (no run.json) is set aside as *_interrupted_<k> and run again"],
        "report_specification": {
            "rows": list(CORE) + list(EXTENSION),
            "per_row_per_seed": ["J and its five weighted terms", "travel_s (censored)", "time_loss_s",
                                 "finished / scheduled", "incomplete_share", "teleports",
                                 "responses 404/401/405 with status", "normalized response per incident and mean",
                                 "C1 route-validity counters", "routing counters", "responder delay decomposition",
                                 "completed-only averages (supplementary)", "C2 blockage rate (E3 rows, where saved)"],
            "aggregate": "mean, SD and descriptive 95% Student-t interval over the five seeds",
            "paired": ["baseline - row, per incident response and time metrics (positive = row faster)",
                       "row - E3 layer OFF, for J, mean normalized response and traffic metrics "
                       "(negative = row lower)",
                       "E3 layer ON - E3 layer OFF: the emergency-layer ablation"],
            "not_reported": ["a winner or ranking", "baseline J as a comparison (it is 1.0 by construction)"],
        },
    }
    write_new(out, json.dumps(document, indent=1))
    print(f"plan: {rel(out)}; {len(document['new_runs'])} new runs; code digest {document['code_fingerprint']['code_digest']}")
    return 0


# ---------------------------------------------------------------- one run (a child process)

def cmd_run(args) -> int:
    _setup()
    import heldout_evaluation as H
    import tune_e3_weights as tuner

    plan = load(args.plan)
    spec = next((r for r in plan["new_runs"] if r["method"] == args.method and r["seed"] == args.seed), None)
    if spec is None:
        raise SystemExit(f"{args.method} at seed {args.seed} is not in the plan")
    out = REPO_ROOT / spec["out"]
    trial_dir = out / "trial"
    if (out / "run.json").exists() or (trial_dir.exists() and any(trial_dir.iterdir())):
        raise SystemExit(f"{out} already holds a run; evidence is never overwritten")
    files = comparison_fingerprint(args.seed)
    problems = H.code_differences(files, plan["code_fingerprint"]["files"])
    if problems:
        raise SystemExit("the code differs from the plan's fingerprint; refusing to run:\n  " + "\n  ".join(problems))
    reference_file = REPO_ROOT / f"results/tuning_reference/seed_{args.seed}_{SIM_TIME}s/reference.json"
    reference = tuner.load_reference(reference_file, args.seed, SIM_TIME)
    out.mkdir(parents=True, exist_ok=True)
    started, clock, rejected = utc(), time.time(), None
    meta = {"method": args.method, "seed": args.seed, "sim_time": SIM_TIME}
    try:
        if args.method == E3_OFF:
            weights = H.candidate_weights("trial_016")
            if weights != plan["e3_weights"]:
                raise SystemExit("the T16 weights differ from the plan")
            meta.update({"emergency_layer": False, "evaluator": tuner.evaluator_label(False),
                         "weights": weights, "weights_file": WEIGHTS_FILE,
                         "weights_file_sha256": sha256(REPO_ROOT / WEIGHTS_FILE)})
            tuner.evaluate_weights(weights, Path("simulation.sumocfg"), SIM_TIME, args.seed, trial_dir=trial_dir,
                                   reference=reference, emergency_layer=False)
        else:
            meta.update({"emergency_layer": None, "evaluator": f"{tuner.COMPARATOR_EVALUATOR}({args.method!r})"})
            tuner.evaluate_comparator(args.method, Path("simulation.sumocfg"), SIM_TIME, args.seed,
                                      trial_dir=trial_dir, reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    meta.update({"completed": True, "rejected": rejected,
                 "reference": rel(reference_file), "reference_sha256": sha256(reference_file),
                 "code_fingerprint_digest": tuner._digest(files), "code_digest": code_digest(files),
                 "code_matches_plan_except_seed_demand": True,
                 "pythonhashseed": os.environ.get("PYTHONHASHSEED", "unset"),
                 "started_utc": started, "finished_utc": utc(), "wall_seconds": round(time.time() - clock, 1)})
    write_new(out / "run.json", json.dumps(meta, indent=1))
    print(json.dumps({"method": args.method, "seed": args.seed, "rejected": rejected,
                      "wall_seconds": meta["wall_seconds"]}))
    return 0 if rejected is None else 1


# ---------------------------------------------------------------- validity (pure given the files)

def summary_steps(path: Path) -> int:
    import xml.etree.ElementTree as ET
    return sum(1 for _ in ET.parse(str(path)).getroot().iter("step"))


def tracebacks(*paths: Path) -> int:
    return sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback") for p in paths if p.is_file())


def new_run_problems(spec: dict, plan: dict, root: Path = REPO_ROOT) -> List[str]:
    """Why a finished new run is invalid (empty list = valid)."""
    import heldout_evaluation as H

    out = root / spec["out"]
    if not (out / "run.json").is_file():
        return ["no run.json (not run, or interrupted)"]
    meta, problems = load(out / "run.json"), []
    trial_file = out / "trial" / "trial.json"
    if not trial_file.is_file():
        return ["no trial.json"]
    trial = load(trial_file)
    if tracebacks(out / "run.out", out / "run.err", out / "trial" / "errors.log"):
        problems.append("traceback")
    if meta.get("rejected"):
        problems.append(f"rejected: {meta['rejected']}")
    if trial.get("exceptions_logged"):
        problems.append(f"{len(trial['exceptions_logged'])} exception(s) logged")
    if meta.get("seed") != spec["seed"] or trial.get("seed") != spec["seed"]:
        problems.append("wrong seed")
    if meta.get("sim_time") != SIM_TIME or float(trial.get("sim_time", -1)) != SIM_TIME:
        problems.append("wrong duration")
    if meta.get("method") != spec["method"]:
        problems.append("wrong method")
    if not meta.get("code_matches_plan_except_seed_demand") or meta.get("code_digest") != plan["code_fingerprint"]["code_digest"]:
        problems.append("code fingerprint differs from the plan")
    expected_ref = f"results/tuning_reference/seed_{spec['seed']}_{SIM_TIME}s/reference.json"
    if meta.get("reference") != expected_ref:
        problems.append("scored against another seed's baseline")
    elif meta.get("reference_sha256") != sha256(root / expected_ref):
        problems.append("the baseline reference changed after the run")
    if spec["method"] == E3_OFF:
        if trial.get("emergency_layer") is not False or meta.get("emergency_layer") is not False:
            problems.append("emergency layer is not OFF")
        if not (trial.get("weights_requested") == trial.get("weights_used") == meta.get("weights") == plan["e3_weights"]):
            problems.append("weights are not T16's")
        if meta.get("weights_file_sha256") != plan["e3_weights_file_sha256"]:
            problems.append("the T16 weight file changed")
        if not (out / "trial" / "blockage_samples.json").is_file():
            problems.append("C2 blockage samples missing")
    else:
        if trial.get("method") != spec["method"]:
            problems.append("trial.json names another method")
    if not trial.get("finished_equals_summary_arrived"):
        problems.append("finished != summary arrived")
    if not trial.get("no_tripinfo_record_outside_population"):
        problems.append("tripinfo record outside the population")
    pairing = trial.get("pairing") or {}
    if pairing.get("speed_factor_differences") != 0 or pairing.get("ordinary_vehicles_compared", 0) < 3000:
        problems.append(f"not the seed's frozen traffic realisation: {pairing}")
    if set((trial.get("ambient_road_classes") or {}).values()) != {"authority"}:
        problems.append("ambient emergency types not switched to authority")
    responses = (trial.get("metrics") or {}).get("responses") or {}
    if sorted(responses) != sorted(INCIDENTS):
        problems.append("emergency scenario incomplete: responses for 404/401/405 missing")
    if not (trial.get("objective") or {}).get("J"):
        problems.append("no J")
    if any(k not in (trial.get("route_stats") or {}) for k in C1_COUNTERS):
        problems.append("C1 route-validity counters missing")
    if (out / "trial" / "summary.xml").is_file() and summary_steps(out / "trial" / "summary.xml") != SIM_TIME:
        problems.append("summary does not cover 3600 steps")
    return problems


def reused_row_problems(row: dict, plan_row: dict, root: Path = REPO_ROOT) -> List[str]:
    """Why a reused row can no longer be used (its files changed or it was never valid)."""
    problems = []
    if row["method"] == BASELINE:
        ref = root / row["reference"]
        if not ref.is_file():
            return ["reference missing"]
        if sha256(ref) != plan_row["reference_sha256"]:
            problems.append("reference changed since the plan")
    else:
        run = root / row["run"]
        if not (run / "run.json").is_file():
            return ["held-out run missing"]
        if sha256(run / "run.json") != plan_row["run_json_sha256"] or \
                sha256(run / "trial" / "trial.json") != plan_row["trial_json_sha256"]:
            problems.append("held-out run changed since the plan")
        if load(run / "run.json").get("rejected"):
            problems.append("held-out run was rejected")
    return problems


def run_state(out: Path) -> str:
    if (out / "run.json").is_file():
        return "done"
    return "interrupted" if out.exists() and any(out.iterdir()) else "todo"


def validation(plan: dict, root: Path = REPO_ROOT) -> dict:
    new = {spec["id"]: new_run_problems(spec, plan, root) for spec in plan["new_runs"]}
    reused = {row["id"]: reused_row_problems(row, row, root) for row in plan["reused_rows"]}
    return {"utc": utc(), "valid": not any(new.values()) and not any(reused.values()),
            "new_runs": new, "reused_rows": reused}


def cmd_validate(args) -> int:
    _setup()
    plan_path = Path(args.plan)
    result = validation(load(plan_path))
    out = write_new(plan_path.parent / "validation" / f"validation_{stamp()}.json", json.dumps(result, indent=1))
    bad = {k: v for part in ("new_runs", "reused_rows") for k, v in result[part].items() if v}
    for key, problems in bad.items():
        print(f"  INVALID {key}: {problems}")
    print(f"validation: {'VALID' if result['valid'] else 'NOT VALID'} -> {rel(out)}")
    return 0 if result["valid"] else 1


def latest_validation_passed(plan_path: Path) -> bool:
    files = sorted((plan_path.parent / "validation").glob("validation_*.json"))
    return bool(files) and load(files[-1]).get("valid") is True


# ---------------------------------------------------------------- campaign

def cmd_campaign(args) -> int:
    _setup()
    plan_path = Path(args.plan)
    plan = load(plan_path)
    methods = set(args.methods.split(",")) if args.methods else set(NEW_METHODS)
    seeds = {int(s) for s in args.seeds.split(",")} if args.seeds else set(SEEDS)
    unknown = (methods - set(NEW_METHODS)) | (seeds - set(SEEDS))
    if unknown:
        raise SystemExit(f"not in the plan: {sorted(map(str, unknown))}")
    events = plan_path.parent / "campaign_events.jsonl"

    def log(**event):
        with events.open("a", encoding="utf-8") as f:
            f.write(json.dumps({"utc": utc(), **event}) + "\n")

    queue = []
    for spec in plan["new_runs"]:
        if spec["method"] not in methods or spec["seed"] not in seeds:
            continue
        out = REPO_ROOT / spec["out"]
        state = run_state(out)
        if state == "done":
            continue
        if state == "interrupted":
            k = 1
            while out.with_name(f"{out.name}_interrupted_{k}").exists():
                k += 1
            out.rename(out.with_name(f"{out.name}_interrupted_{k}"))
            log(event="interrupted_run_set_aside", run=spec["id"], kept_as=f"{out.name}_interrupted_{k}")
        queue.append(spec)
    log(event="campaign_started", todo=[s["id"] for s in queue], parallel=args.parallel)
    env = dict(os.environ, PYTHONHASHSEED="0")
    running, stopped = {}, None
    while (queue and stopped is None) or running:
        while queue and stopped is None and len(running) < args.parallel:
            spec = queue.pop(0)
            out = REPO_ROOT / spec["out"]
            out.mkdir(parents=True, exist_ok=True)
            fo = (out / "run.out").open("w", encoding="utf-8")
            fe = (out / "run.err").open("w", encoding="utf-8")
            command = [sys.executable, str(Path(__file__).resolve()), "run", "--plan", str(plan_path.resolve()),
                       "--method", spec["method"], "--seed", str(spec["seed"])]
            running[spec["id"]] = (subprocess.Popen(command, cwd=REPO_ROOT, env=env, stdout=fo, stderr=fe),
                                   spec, fo, fe, time.time())
            log(event="run_started", run=spec["id"])
            print(f"[comparison] {utc()} started {spec['id']}", flush=True)
        time.sleep(5)
        for run_id, (proc, spec, fo, fe, t0) in list(running.items()):
            if proc.poll() is None:
                continue
            fo.close(), fe.close()
            del running[run_id]
            problems = new_run_problems(spec, plan)
            log(event="run_finished", run=run_id, exit=proc.returncode, problems=problems,
                wall_seconds=round(time.time() - t0, 1))
            print(f"[comparison] {utc()} finished {run_id} in {time.time() - t0:.0f} s: "
                  f"{'valid' if not problems else problems}", flush=True)
            if problems:
                stopped = f"{run_id}: {problems}"
    log(event="campaign_stopped" if stopped else "campaign_idle", reason=stopped)
    print(f"[comparison] {'STOPPED - do not rerun or replace; report: ' + stopped if stopped else 'no selected runs left'}")
    return 1 if stopped else 0


def cmd_status(args) -> int:
    plan = load(args.plan)
    counts = {}
    for spec in plan["new_runs"]:
        state = run_state(REPO_ROOT / spec["out"])
        counts[state] = counts.get(state, 0) + 1
        wall = ""
        if state == "done":
            wall = f" ({load(REPO_ROOT / spec['out'] / 'run.json').get('wall_seconds')} s)"
        print(f"  {spec['id']:28s} {state}{wall}")
    print(f"new runs: {counts}; latest validation passed: {latest_validation_passed(Path(args.plan))}")
    return 0


# ---------------------------------------------------------------- analysis (C4)

def statistics_of(values: Sequence[Optional[float]]) -> dict:
    import heldout_evaluation as H
    return H.paired_statistics(list(values))


def row_evidence(row_id: str, method: str, seed: int, plan: dict, root: Path = REPO_ROOT,
                 trial_dir_override: Optional[Path] = None) -> dict:
    """One row at one seed, from its saved files only. `trial_dir_override` is for the dry pipeline check only."""
    import heldout_evaluation as H
    import response_delay_metrics as RDM
    from metrics.emergency_metrics import blockage_rate

    if method == BASELINE:
        ref = load(root / f"results/tuning_reference/seed_{seed}_{SIM_TIME}s/reference.json")
        m, trial_dir, objective, route_stats, routing, completed_only = ref["metrics"], \
            root / f"results/tuning_reference/seed_{seed}_{SIM_TIME}s", \
            {"J": 1.0, "terms": None, "relative": None}, None, None, None
        blockage = None
    else:
        if trial_dir_override is not None:
            trial_dir = Path(trial_dir_override)
        elif method == E3_ON:
            trial_dir = root / f"{HELDOUT_RUNS}/trial_016_seed_{seed}/trial"
        else:
            trial_dir = root / f"{OUT_ROOT}/runs/{method}_seed_{seed}/trial"
        trial = load(trial_dir / "trial.json")
        m, objective = trial["metrics"], trial["objective"]
        route_stats = {k: trial["route_stats"].get(k) for k in C1_COUNTERS}
        routing = trial.get("routing") or trial.get("coordinator")
        completed_only = trial.get("completed_only_supplementary")
        blockage_file = trial_dir / "blockage_samples.json"
        blockage = blockage_rate(load(blockage_file)) if blockage_file.is_file() else None
    trips = RDM.responder_trips(trial_dir / "tripinfo.xml")
    delays = RDM.scenario_dispatch_delays()
    responders = {i: ({**RDM.delay_decomposition(t["duration_s"], t["time_loss_s"], delays[i]),
                       "route_length_m": t["route_length_m"], "reroutes": t["reroutes"],
                       "waiting_s": t["waiting_s"], "stops": t["stops"]} if (t := trips.get(i)) else None)
                  for i in INCIDENTS}
    return {
        "row": method, "seed": seed, "J": objective.get("J"), "terms": objective.get("terms"),
        "relative": objective.get("relative"),
        "raw": {"travel_s": m["travel_s"], "time_loss_s": m["time_loss_s"], "finished": m["finished"],
                "vehicles_scheduled": m["vehicles_scheduled"],
                "completion_share": m["finished"] / m["vehicles_scheduled"],
                "incomplete_share": m["incomplete_share"], "teleports": m["teleports"],
                "responses": {k: {"value_s": v["value_s"], "status": v["status"]} for k, v in m["responses"].items()}},
        "completed_only_supplementary": {
            "travel_s": m.get("reported_travel_s_completed"), "time_loss_s": m.get("reported_time_loss_s_completed"),
            "waiting_s": m.get("reported_waiting_s_completed"),
            **({"report": completed_only} if completed_only else {})},
        "c1_route_validity": route_stats,
        "routing_counters": routing,
        "c2_blockage": blockage,
        "responders": responders,
    }


def aggregate_rows(per_seed: Dict[str, Dict[str, dict]], rows: Sequence[str]) -> dict:
    """Five-seed statistics per row, and the paired differences named in the plan (pure, tested)."""
    import heldout_evaluation as H
    seeds = sorted(per_seed)
    out: Dict[str, dict] = {}

    def series(row, getter):
        values = []
        for s in seeds:
            try:
                values.append(getter(per_seed[s][row]))
            except (KeyError, TypeError):
                values.append(None)
        return values

    def response(row, incident):
        def get(x):
            r = x["raw"]["responses"][incident]
            return r["value_s"] if r["status"] in VALID_RESPONSE_STATUSES else None
        return series(row, get)

    for row in rows:
        stats = {
            "J": statistics_of(series(row, lambda x: x["J"])),
            "travel_s": statistics_of(series(row, lambda x: x["raw"]["travel_s"])),
            "time_loss_s": statistics_of(series(row, lambda x: x["raw"]["time_loss_s"])),
            "finished": statistics_of(series(row, lambda x: x["raw"]["finished"])),
            "completion_share": statistics_of(series(row, lambda x: x["raw"]["completion_share"])),
            "incomplete_share": statistics_of(series(row, lambda x: x["raw"]["incomplete_share"])),
            "teleports": statistics_of(series(row, lambda x: x["raw"]["teleports"])),
            "mean_normalized_response": statistics_of(series(row, lambda x: x["normalized_response"]["mean"])),
            "responses": {i: statistics_of(response(row, i)) for i in INCIDENTS},
            "completed_only_travel_s": statistics_of(
                series(row, lambda x: x["completed_only_supplementary"]["travel_s"])),
        }
        base = {i: response(BASELINE, i) for i in INCIDENTS}
        mine = {i: response(row, i) for i in INCIDENTS}
        stats["baseline_minus_row"] = {
            **{f"response_{i}": statistics_of([None if b is None or r is None else b - r
                                               for b, r in zip(base[i], mine[i])]) for i in INCIDENTS},
            **{f"response_{i}_percent": statistics_of([None if b is None or r is None or b == 0 else (b - r) / b * 100
                                                       for b, r in zip(base[i], mine[i])]) for i in INCIDENTS},
            "travel_s": statistics_of([None if b is None or r is None else b - r for b, r in zip(
                series(BASELINE, lambda x: x["raw"]["travel_s"]), series(row, lambda x: x["raw"]["travel_s"]))]),
            "time_loss_s": statistics_of([None if b is None or r is None else b - r for b, r in zip(
                series(BASELINE, lambda x: x["raw"]["time_loss_s"]), series(row, lambda x: x["raw"]["time_loss_s"]))]),
            "finished_row_minus_baseline": statistics_of([None if b is None or r is None else r - b for b, r in zip(
                series(BASELINE, lambda x: x["raw"]["finished"]), series(row, lambda x: x["raw"]["finished"]))]),
        }
        if row != E3_OFF and E3_OFF in rows:
            def diff(getter):
                return statistics_of([None if a is None or b is None else a - b
                                      for a, b in zip(series(row, getter), series(E3_OFF, getter))])
            stats["row_minus_e3_off"] = {
                "J": diff(lambda x: x["J"]),
                "mean_normalized_response": diff(lambda x: x["normalized_response"]["mean"]),
                "travel_s": diff(lambda x: x["raw"]["travel_s"]),
                "time_loss_s": diff(lambda x: x["raw"]["time_loss_s"]),
                "incomplete_share": diff(lambda x: x["raw"]["incomplete_share"]),
                "teleports": diff(lambda x: x["raw"]["teleports"]),
                **{f"response_{i}": statistics_of([None if a is None or b is None else a - b
                                                   for a, b in zip(mine[i], response(E3_OFF, i))])
                   for i in INCIDENTS},
            }
        out[row] = stats
    return out


def cmd_analyse(args) -> int:
    _setup()
    import heldout_evaluation as H

    plan_path = Path(args.plan)
    plan = load(plan_path)
    target_json, target_md = plan_path.parent / "report.json", plan_path.parent / "report.md"
    for t in (target_json, target_md):
        if t.exists():
            raise SystemExit(f"{t} exists; the report is written once")
    result = validation(plan)
    if not result["valid"] or not latest_validation_passed(plan_path):
        raise SystemExit("refusing: run `validate` first; the latest validation must pass and the evidence must "
                         "still be valid")
    report = build_comparison_report(plan, rel(plan_path))
    write_new(target_json, json.dumps(report, indent=1, default=str))
    write_new(target_md, report_markdown(report))
    print(f"report: {rel(target_json)} and {rel(target_md)}")
    return 0


def build_comparison_report(plan: dict, plan_name: str, trial_dir_for=None) -> dict:
    """The full report from saved files. `trial_dir_for(method, seed)` is only for the dry pipeline check."""
    import heldout_evaluation as H

    rows = list(CORE) + list(EXTENSION)
    per_seed: Dict[str, Dict[str, dict]] = {}
    for s in SEEDS:
        block = {row: row_evidence(f"{row}_seed_{s}", row, s, plan,
                                   trial_dir_override=(trial_dir_for(row, s) if trial_dir_for and row in NEW_METHODS
                                                       else None))
                 for row in rows}
        base_resp = block[BASELINE]["raw"]["responses"]
        for row in rows:
            block[row]["normalized_response"] = H._normalized_block(block[row]["raw"]["responses"], base_resp)
        per_seed[str(s)] = block
    return {
        "kind": plan["kind"], "plan": plan_name, "written_utc": utc(), "valid": True,
        "seeds": plan["seeds"], "core_six_method_comparison": plan["core_six_method_comparison"],
        "emergency_aware_extension": plan["emergency_aware_extension"], "naming_rule": plan["naming_rule"],
        "measurement": plan["measurement"], "planning_horizon_note": plan["planning_horizon_note"],
        "code_digest": plan["code_fingerprint"]["code_digest"],
        "sign_conventions": {"baseline_minus_row": "positive = the row is faster / lower than no rerouting",
                             "finished_row_minus_baseline": "positive = the row completed more trips",
                             "row_minus_e3_off": "negative = the row is lower than E3 with the layer off"},
        "c2_note": ("blockage samples exist for the new E3 layer-OFF runs; the reused held-out layer-ON runs did not "
                    "save them, so their C2 entry is null"),
        "per_seed": per_seed,
        "aggregate": aggregate_rows(per_seed, rows),
    }


def _fmt(stat: dict, digits: int = 1) -> str:
    if not stat or stat.get("mean") is None:
        return "n/a"
    sd = stat.get("sd")
    return f"{stat['mean']:.{digits}f}" + (f" ± {sd:.{digits}f}" if sd is not None else "")


def report_markdown(report: dict) -> str:
    agg = report["aggregate"]
    core = list(report["core_six_method_comparison"])
    ext = list(report["emergency_aware_extension"])
    lines = ["# V1 comparison report", "",
             f"Written {report['written_utc']}. Seeds {', '.join(map(str, report['seeds']))}. "
             f"Comparison code digest `{report['code_digest']}`. Values are mean ± SD over the five seeds.", "",
             f"**Measurement.** {report['measurement']}.", "",
             f"**Planning horizon (B1).** {report['planning_horizon_note']}.", "",
             "## Core six-method comparison", "",
             "| method | J | travel time (s, censored) | completion | incomplete share | time loss (s) | teleports | mean normalized response |",
             "|---|---|---|---|---|---|---|---|"]

    def row_line(r):
        a = agg[r]
        comp = a["completion_share"]
        comp_text = "n/a" if comp["mean"] is None else f"{comp['mean'] * 100:.1f}% (finished {_fmt(a['finished'], 0)})"
        j = "1.0 by construction" if r == BASELINE else _fmt(a["J"], 4)
        return (f"| {LABELS[r]} | {j} | {_fmt(a['travel_s'])} | {comp_text} | {_fmt(a['incomplete_share'], 4)} | "
                f"{_fmt(a['time_loss_s'])} | {_fmt(a['teleports'])} | {_fmt(a['mean_normalized_response'], 3)} |")

    lines += [row_line(r) for r in core]
    lines += ["", "## Separate emergency-aware extension (not part of the six-method comparison)", "",
              "| row | J | travel time (s, censored) | completion | incomplete share | time loss (s) | teleports | mean normalized response |",
              "|---|---|---|---|---|---|---|---|"]
    lines += [row_line(r) for r in ext]
    lines += ["", "## Emergency response (s)", "",
              "| row | 404 | 401 | 405 | baseline − row 404 | 401 | 405 | % 404 | % 401 | % 405 |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in core + ext:
        a = agg[r]
        d = a["baseline_minus_row"]
        lines.append(f"| {LABELS[r]} | " + " | ".join(_fmt(a["responses"][i], 0) for i in INCIDENTS) + " | " +
                     " | ".join(_fmt(d[f"response_{i}"], 0) for i in INCIDENTS) + " | " +
                     " | ".join(_fmt(d[f"response_{i}_percent"], 1) for i in INCIDENTS) + " |")
    lines += ["", "## Paired against E3 with the emergency layer OFF (row − E3 OFF; negative = row lower)", "",
              "| row | ΔJ | Δ mean normalized response | Δ travel (s) | Δ time loss (s) | Δ incomplete share | Δ teleports |",
              "|---|---|---|---|---|---|---|"]
    for r in [x for x in core + ext if x not in (E3_OFF, BASELINE)]:
        d = agg[r]["row_minus_e3_off"]
        lines.append(f"| {LABELS[r]} | {_fmt(d['J'], 4)} | {_fmt(d['mean_normalized_response'], 3)} | "
                     f"{_fmt(d['travel_s'])} | {_fmt(d['time_loss_s'])} | {_fmt(d['incomplete_share'], 4)} | "
                     f"{_fmt(d['teleports'])} |")
    lines += ["", "The E3 layer-ON row minus the layer-OFF row is the emergency-layer ablation.", "",
              "## C1 — route validity (sum over the five seeds)", "",
              "| row | " + " | ".join(C1_COUNTERS) + " |", "|---|" + "---|" * len(C1_COUNTERS)]
    for r in core + ext:
        if r == BASELINE:
            continue
        totals = []
        for k in C1_COUNTERS:
            vals = [report["per_seed"][s][r]["c1_route_validity"].get(k) for s in report["per_seed"]]
            totals.append("n/a" if any(v is None for v in vals) else str(sum(vals)))
        lines.append(f"| {LABELS[r]} | " + " | ".join(totals) + " |")
    lines += ["", "## C2 — blockage on the incident edges (E3 rows)", "", report["c2_note"] + ".", "",
              "| row | seed | 404 | 401 | 405 |", "|---|---|---|---|---|"]
    for r in (E3_OFF, E3_ON):
        for s, block in report["per_seed"].items():
            b = block[r]["c2_blockage"]
            cells = ["not saved" if b is None else
                     str((b.get(i) or {}).get("blockage_rate")) for i in INCIDENTS]
            lines.append(f"| {LABELS[r]} | {s} | " + " | ".join(cells) + " |")
    lines += ["", "## Supplementary: completed-trip travel time only (s)", "",
              "| row | completed-only travel time |", "|---|---|"]
    lines += [f"| {LABELS[r]} | {_fmt(agg[r]['completed_only_travel_s'])} |" for r in core + ext]
    lines += ["", "Completed-only averages drop unfinished trips and are descriptive only; the censored values above "
              "are the primary measure.", "",
              "Five deterministic seeds, one run per row per seed. Intervals in `report.json` are descriptive, not "
              "significance tests. No winner or ranking is computed."]
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------- figures (reads only report.json)

def cmd_figures(args) -> int:
    import comparison_figures
    plan_path = Path(args.plan)
    report_path = plan_path.parent / "report.json"
    if not report_path.is_file():
        raise SystemExit("no report.json; run `analyse` first")
    written = comparison_figures.make_all(load(report_path), plan_path.parent / "figures")
    for p in written:
        print(f"  figure {rel(p)}")
    return 0


def main() -> int:
    _setup()
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("plan")
    p.add_argument("--out", required=True)
    r = sub.add_parser("run")
    r.add_argument("--plan", required=True)
    r.add_argument("--method", required=True, choices=NEW_METHODS)
    r.add_argument("--seed", type=int, required=True, choices=SEEDS)
    c = sub.add_parser("campaign")
    c.add_argument("--plan", required=True)
    c.add_argument("--parallel", type=int, default=3)
    c.add_argument("--methods", default=None, help="comma list; default all new methods")
    c.add_argument("--seeds", default=None, help="comma list; default all five seeds")
    for name in ("validate", "analyse", "figures", "status"):
        sub.add_parser(name).add_argument("--plan", required=True)
    args = parser.parse_args()
    return {"plan": cmd_plan, "run": cmd_run, "campaign": cmd_campaign, "validate": cmd_validate,
            "analyse": cmd_analyse, "figures": cmd_figures, "status": cmd_status}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
