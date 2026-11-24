#!/usr/bin/env python3
"""The post-tuning robustness check of contract 13.20: its nudge runner, plan, campaign and analysis (2026-09-16).

Diagnostic only. It never changes production code, routing, weights, the objective or the tuning campaign, and it
writes only into the folders it is given. Every run is the tuner's own trial function
(`tune_e3_weights.evaluate_weights`: tuning seed 20260721, 3,600 s, frozen demand, all three incidents, J against
the canonical baseline). One hook is added, at the start of each coordinator step:

- **No nudge** (`--no-nudge`): the hook only passes the call on, with no TraCI command, so the run must equal the
  original trial bit for bit (`check-reproduction`).
- **Nudge at t** (`--nudge-time t`): at the first step at or after t, the lowest-id ordinary vehicle (`normal_*`)
  moving faster than 5 m/s is slowed to half its speed over 2 s (`traci.vehicle.slowDown`), once. This is the
  contract's standardized perturbation rule. It is the rule of `yield_signal_diagnosis.py --nudge-time` (2026-09-15),
  applied at the same point of the step, before the coordinator's work.
  - The vehicle, time, speed, road, lane and lane position are recorded in `run.json`.
  - A nudge applied later than t, because no car qualified at t, is recorded and flagged invalid.

Every run first checks that the code/input fingerprint equals the frozen record
(`results/final_code_fingerprint_2026-09-15/campaign_settings.json`) and refuses to start otherwise.

    python scripts/diagnostics/robustness_nudge.py run --config trial_010 --no-nudge --out DIR
    python scripts/diagnostics/robustness_nudge.py check-reproduction DIR results/e3_tuning_campaign_2026-09-15/trials/trial_010
    python scripts/diagnostics/robustness_nudge.py plan --expect 10,16,11,17,12 --out results/robustness_check_2026-09-16/plan.json
    python scripts/diagnostics/robustness_nudge.py campaign --plan results/robustness_check_2026-09-16/plan.json --parallel 3
    python scripts/diagnostics/robustness_nudge.py analyse --plan results/robustness_check_2026-09-16/plan.json

The rules `plan` and `analyse` apply are those of contract 13.20, fixed before any robustness result existed:
- **finalists:** the 5 COMPLETE trials with the lowest tuning J (a duplicate weight set, to 1e-9, is replaced by the
  next trial), plus DEFAULT;
- **runs:** 4 per configuration, at 250 / 350 / 450 / 550 s;
- **score:** the mean J over its 4 nudge runs. The unperturbed J is reported only;
- **winner:** the lowest score, with DEFAULT a full candidate;
- **ties** (scores within 1e-6): more paired wins among the tied, then the lower worst-case J, then DEFAULT, then the
  lowest trial number;
- **validity:** any run with a traceback, a logged exception, a rejected trial, a fingerprint mismatch, or not
  exactly one nudge at its declared time stops the check. No winner is named, and no replacement or extra run is
  made.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CAMPAIGN = REPO_ROOT / "results/e3_tuning_campaign_2026-09-15"
FROZEN_RECORD = REPO_ROOT / "results/final_code_fingerprint_2026-09-15/campaign_settings.json"
REFERENCE = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
# DEFAULT has no tuning trial. Its unperturbed run on the frozen code's behaviour: the production run of 2026-09-15.
# It was made before the comment-only date fixes and the weight-guard constant, which change no runtime path.
DEFAULT_UNPERTURBED = REPO_ROOT / "results/unified_yield_production_2026-09-15/def_production/trial"
SEED, SIM_TIME = 20260721, 3600
NUDGE_TIMES = (250.0, 350.0, 450.0, 550.0)
FINALISTS = 5
MIN_SPEED_MS = 5.0          # only a car moving faster than this is nudged
SLOW_FACTOR = 0.5           # to half its speed
SLOW_DURATION_S = 2.0       # over 2 s
TIE_TOLERANCE = 1e-6
DUPLICATE_TOLERANCE = 1e-9
WEIGHT_NAMES = ("pheromone", "heuristic", "protocol", "bco", "velocity")


# ---------------------------------------------------------------- the perturbation rule (pure, tested)

def choose_nudge_vehicle(vehicle_ids: Iterable[str], speed_of: Callable[[str], float]) -> Optional[Tuple[str, float]]:
    """The lowest-id ordinary vehicle moving faster than MIN_SPEED_MS, or None. Ids are zero-padded, so sorting the
    strings sorts the numbers."""
    for vehicle in sorted(vehicle_ids):
        if vehicle.startswith("normal_"):
            speed = speed_of(vehicle)
            if speed > MIN_SPEED_MS:
                return vehicle, speed
    return None


class NudgeHook:
    """Wraps `P3RuntimeCoordinator.update_step`. With no nudge time it only passes the call on (no TraCI command)."""

    def __init__(self, nudge_time: Optional[float], traci_module=None):
        self.nudge_time = nudge_time
        self.traci = traci_module
        self.record: Optional[dict] = None
        self.steps_tried = 0

    def before_step(self, sim_time: float) -> None:
        if self.nudge_time is None or self.record is not None or sim_time < self.nudge_time:
            return
        t = self.traci
        self.steps_tried += 1
        chosen = choose_nudge_vehicle(t.vehicle.getIDList(), t.vehicle.getSpeed)
        if chosen is None:
            return                                      # try again next step; recorded as late if applied later
        vehicle, speed = chosen
        t.vehicle.slowDown(vehicle, speed * SLOW_FACTOR, SLOW_DURATION_S)
        self.record = {"vehicle": vehicle, "declared_time": self.nudge_time, "applied_at": sim_time,
                       "speed_ms": speed, "target_speed_ms": speed * SLOW_FACTOR, "duration_s": SLOW_DURATION_S,
                       "road": t.vehicle.getRoadID(vehicle), "lane": t.vehicle.getLaneID(vehicle),
                       "lane_position_m": t.vehicle.getLanePosition(vehicle), "steps_tried": self.steps_tried}

    def install(self, coordinator_class) -> None:
        real_update = coordinator_class.update_step
        hook = self

        def update_step(coordinator, sim_time):
            hook.before_step(sim_time)
            return real_update(coordinator, sim_time)

        coordinator_class.update_step = update_step


# ---------------------------------------------------------------- selection and ranking (pure, tested)

def select_finalists(trials: Sequence[dict], n: int = FINALISTS) -> List[int]:
    """`trials`: dicts with trial, state, value, weights. The n COMPLETE trials with the lowest J; a weight set
    equal (to DUPLICATE_TOLERANCE) to an already chosen one is skipped and the next trial taken."""
    complete = sorted((t for t in trials if t["state"] == "COMPLETE" and t["value"] is not None),
                      key=lambda t: (t["value"], t["trial"]))
    chosen: List[dict] = []
    for t in complete:
        if any(all(abs(t["weights"][k] - c["weights"][k]) <= DUPLICATE_TOLERANCE for k in WEIGHT_NAMES)
               for c in chosen):
            continue
        chosen.append(t)
        if len(chosen) == n:
            break
    return [t["trial"] for t in chosen]


def rank_configurations(nudge_j: Dict[str, Dict[float, float]], trial_number: Dict[str, Optional[int]]) -> List[dict]:
    """Contract 13.20. `nudge_j[label][t]` = J of that configuration's nudge run at t. DEFAULT has trial None.
    Returns every configuration in rank order, with its score and how its place was decided."""
    remaining = dict(nudge_j)
    order = []
    while remaining:
        scores = {label: sum(js.values()) / len(js) for label, js in remaining.items()}
        best = min(scores.values())
        tied = sorted(label for label, s in scores.items() if s - best <= TIE_TOLERANCE)
        decided_by = "lowest score"
        if len(tied) > 1:
            def wins(label):
                return sum(1 for other in tied if other != label for t in remaining[label]
                           if remaining[label][t] < remaining[other][t])
            keys = {label: (-wins(label), max(remaining[label].values()),
                            0 if trial_number[label] is None else 1,
                            trial_number[label] if trial_number[label] is not None else -1)
                    for label in tied}
            winner = min(tied, key=lambda label: keys[label])
            decided_by = "tie within 1e-6, broken by " + _which_key(keys, winner)
        else:
            winner = tied[0]
        order.append({"configuration": winner, "score": scores[winner], "decided_by": decided_by,
                      "tied_with": [x for x in tied if x != winner]})
        del remaining[winner]
    return order


def _which_key(keys: dict, winner: str) -> str:
    names = ("paired wins", "worst-case J", "DEFAULT preference", "lowest trial number")
    others = [k for label, k in keys.items() if label != winner]
    for i, name in enumerate(names):
        if all(keys[winner][i] < k[i] for k in others):
            return name
        others = [k for k in others if k[i] == keys[winner][i]]
    return "lowest trial number"


# ---------------------------------------------------------------- file helpers

def records(path: Path, tag: str, ignore: Sequence[str] = ()) -> list:
    """Every `tag` element's attributes (minus `ignore`) and children, in file order."""
    return [({k: v for k, v in e.attrib.items() if k not in ignore}, [(c.tag, dict(c.attrib)) for c in e])
            for e in ET.parse(path).getroot().iter(tag)]


def sumo_lines(path: Path) -> list:
    return [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.startswith(("Warning", "Error"))]


def tracebacks(*paths: Path) -> int:
    return sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback") for p in paths if p.is_file())


def utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def frozen_digest() -> str:
    return json.loads(FROZEN_RECORD.read_text(encoding="utf-8"))["code_fingerprint_digest"]


def configuration_weights(label: str) -> Dict[str, float]:
    if label == "DEFAULT":
        from p3_hybrid_router import DEFAULT_WEIGHTS
        return {name: float(DEFAULT_WEIGHTS[name]) for name in WEIGHT_NAMES}
    number = int(label.split("_")[1])
    trial = json.loads((CAMPAIGN / "trials" / f"trial_{number:03d}" / "trial.json").read_text(encoding="utf-8"))
    return {name: float(trial["weights_requested"][name]) for name in WEIGHT_NAMES}


def unperturbed_dir(label: str) -> Path:
    return DEFAULT_UNPERTURBED if label == "DEFAULT" else CAMPAIGN / "trials" / f"trial_{int(label.split('_')[1]):03d}"


# ---------------------------------------------------------------- run: one simulation

def run(args) -> int:
    sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
    import traci
    import tune_e3_weights as tuner
    import p3_runtime

    out = Path(args.out)
    trial_dir = out / "trial"
    if (out / "run.json").exists() or (trial_dir.exists() and any(trial_dir.iterdir())):
        raise SystemExit(f"{out} already holds a run; evidence is never overwritten")
    digest = tuner._digest(tuner.evaluation_fingerprint(SEED, Path("simulation.sumocfg")))
    if digest != frozen_digest():
        raise SystemExit(f"code fingerprint {digest} differs from the frozen record {frozen_digest()}; refusing to run")
    # A shorter run is a tool check only (unscored, never part of the 24): the reference is for 3,600 s.
    sim_time = int(args.sim_time)
    if sim_time != SIM_TIME and not args.unscored:
        raise SystemExit("a run shorter than 3,600 s is a tool check only; pass --unscored")
    out.mkdir(parents=True, exist_ok=True)
    weights = configuration_weights(args.config)
    hook = NudgeHook(None if args.no_nudge else float(args.nudge_time), traci)
    hook.install(p3_runtime.P3RuntimeCoordinator)
    started, clock = utc(), time.time()
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), sim_time, SEED, trial_dir=trial_dir,
                               reference=None if args.unscored else tuner.load_reference(REFERENCE, SEED, SIM_TIME))
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    record = {"completed": True, "configuration": args.config, "weights": weights,
              "nudge_time": hook.nudge_time, "nudge": hook.record, "rejected": rejected,
              "tool_check_only": bool(args.unscored),
              "code_fingerprint_digest": digest, "seed": SEED, "sim_time": sim_time,
              "pythonhashseed": os.environ.get("PYTHONHASHSEED", "unset"),
              "started_utc": started, "finished_utc": utc(), "wall_seconds": round(time.time() - clock, 1)}
    (out / "run.json").write_text(json.dumps(record, indent=1), encoding="utf-8")
    print(json.dumps({k: record[k] for k in ("configuration", "nudge_time", "nudge", "rejected")}))
    return 0 if rejected is None else 1


# ---------------------------------------------------------------- check-reproduction: a no-nudge run against the original

TRIAL_KEYS = ("evaluator", "road_permissions", "ambient_road_classes", "seed", "frozen_demand", "sim_time",
              "weights_requested", "weights_used", "exceptions_logged", "coordinator", "learning", "route_stats",
              "metrics", "objective", "finished_equals_summary_arrived", "no_tripinfo_record_outside_population",
              "pairing")


def check_reproduction(args) -> int:
    run_dir, original = Path(args.run), Path(args.original)
    new_trial = run_dir / "trial"
    failures = []

    def same(label, x, y):
        ok = x == y
        failures.extend([] if ok else [label])
        print(f"  {'same     ' if ok else 'DIFFERENT'} {label}" + ("" if ok else f": {str(x)[:120]} | {str(y)[:120]}"))

    meta = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    same("the run applied no nudge", None, meta["nudge"])
    same("the run was not rejected", None, meta["rejected"])
    a = json.loads((new_trial / "trial.json").read_text(encoding="utf-8"))
    b = json.loads((original / "trial.json").read_text(encoding="utf-8"))
    for key in TRIAL_KEYS:
        same(f"trial.json {key}", a.get(key), b.get(key))
    same("J", (a.get("objective") or {}).get("J"), (b.get("objective") or {}).get("J"))
    for tag, name, ignore in (("tripinfo", "tripinfo.xml", ()), ("step", "summary.xml", ("duration",))):
        x, y = records(new_trial / name, tag, ignore), records(original / name, tag, ignore)
        same(f"{name} records ({len(x)} / {len(y)})", x, y)
    wx, wy = sumo_lines(new_trial / "errors.log"), sumo_lines(original / "errors.log")
    same(f"SUMO warning and error lines ({len(wx)} / {len(wy)})", wx, wy)
    same("tracebacks in the run", 0, tracebacks(run_dir / "run.out", run_dir / "run.err", new_trial / "errors.log"))
    print("REPRODUCED" if not failures else "NOT REPRODUCED: " + ", ".join(failures))
    return 0 if not failures else 1


# ---------------------------------------------------------------- plan

def campaign_trials() -> List[dict]:
    rows = list(csv.DictReader((CAMPAIGN / "e3_trials.csv").open(encoding="utf-8")))
    return [{"trial": int(r["trial"]), "state": r["state"],
             "value": float(r["value"]) if r["value"] not in ("", None) else None,
             "weights": {k: float(r[k]) for k in WEIGHT_NAMES}} for r in rows]


def plan(args) -> int:
    out = Path(args.out).resolve()                      # main() has changed into the repository root
    if out.exists():
        raise SystemExit(f"{out} exists; a plan is fixed once written")
    trials = campaign_trials()
    progress = json.loads((CAMPAIGN / "progress.json").read_text(encoding="utf-8"))
    if not progress.get("campaign_complete"):
        raise SystemExit("the tuning campaign is not complete")
    finalists = select_finalists(trials)
    if args.expect is not None:
        expected = [int(x) for x in args.expect.split(",")]
        if finalists != expected:
            raise SystemExit(f"the rule selects {finalists}, not the expected {expected}; stopping")
    configs = [f"trial_{n:03d}" for n in finalists] + ["DEFAULT"]
    runs_dir = out.parent / "runs"
    by_trial = {t["trial"]: t for t in trials}
    document = {
        "written_utc": utc(), "contract": "13.20 (config/experiment_contract.md v0.12)",
        "campaign": str(CAMPAIGN.relative_to(REPO_ROOT)), "frozen_code_fingerprint_digest": frozen_digest(),
        "seed": SEED, "sim_time": SIM_TIME, "nudge_times": list(NUDGE_TIMES),
        "perturbation_rule": f"at the first step at or after t, the lowest-id normal_* vehicle moving faster than "
                             f"{MIN_SPEED_MS} m/s is slowed to {SLOW_FACTOR} x its speed over {SLOW_DURATION_S} s, once",
        "score": "mean J over the 4 nudge runs; unperturbed J reported only",
        "tie_rule": "within 1e-6: paired wins among the tied, then worst-case J, then DEFAULT, then lowest trial number",
        "configurations": [{"label": c,
                            "trial": None if c == "DEFAULT" else int(c.split("_")[1]),
                            "tuning_J": None if c == "DEFAULT" else by_trial[int(c.split("_")[1])]["value"],
                            "weights": configuration_weights(c)} for c in configs],
        "runs": [{"id": f"{c}_nudge_{int(t)}", "configuration": c, "nudge_time": t,
                  "out": str((runs_dir / f"{c}_nudge_{int(t)}").relative_to(REPO_ROOT))}
                 for c in configs for t in NUDGE_TIMES],
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(document, indent=1), encoding="utf-8")
    print(f"plan written: {out} - finalists {finalists} + DEFAULT, {len(document['runs'])} runs")
    return 0


# ---------------------------------------------------------------- campaign: the 24 runs, resumable, never overwriting

def run_state(run_dir: Path) -> str:
    if (run_dir / "run.json").is_file():
        return "done"
    return "interrupted" if run_dir.exists() and any(run_dir.iterdir()) else "todo"


def run_validity(run_dir: Path, nudge_time: float) -> List[str]:
    problems = []
    meta = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    trial = json.loads((run_dir / "trial" / "trial.json").read_text(encoding="utf-8"))
    if meta.get("rejected"):
        problems.append(f"rejected: {meta['rejected']}")
    if trial.get("exceptions_logged"):
        problems.append(f"{len(trial['exceptions_logged'])} exception(s) logged")
    if tracebacks(run_dir / "run.out", run_dir / "run.err", run_dir / "trial" / "errors.log"):
        problems.append("traceback")
    if meta.get("code_fingerprint_digest") != frozen_digest():
        problems.append("code fingerprint differs from the frozen record")
    if meta.get("sim_time") != SIM_TIME or meta.get("seed") != SEED or meta.get("tool_check_only"):
        problems.append(f"not a scored {SIM_TIME} s run at seed {SEED}")
    nudge = meta.get("nudge")
    if not nudge:
        problems.append("no nudge applied")
    elif nudge["applied_at"] != nudge_time or nudge["declared_time"] != nudge_time:
        problems.append(f"nudge applied at {nudge['applied_at']}, declared {nudge_time}")
    if not (trial.get("objective") or {}).get("J"):
        problems.append("no J")
    return problems


def campaign(args) -> int:
    plan_path = Path(args.plan)
    document = json.loads(plan_path.read_text(encoding="utf-8"))
    events = plan_path.parent / "campaign_events.jsonl"

    def log(**event):
        with events.open("a", encoding="utf-8") as f:
            f.write(json.dumps({"utc": utc(), **event}) + "\n")

    queue, running, stopped = [], {}, None
    for spec in document["runs"]:
        run_dir = REPO_ROOT / spec["out"]
        state = run_state(run_dir)
        if state == "done":
            continue
        if state == "interrupted":
            # A run with no run.json produced no result (power cut, kill). It is deterministic, so the identical
            # specification is run again; the partial folder is kept as evidence. This is not a replacement run.
            k = 1
            while run_dir.with_name(f"{run_dir.name}_interrupted_{k}").exists():
                k += 1
            run_dir.rename(run_dir.with_name(f"{run_dir.name}_interrupted_{k}"))
            log(event="interrupted_run_set_aside", run=spec["id"], kept_as=f"{run_dir.name}_interrupted_{k}")
        queue.append(spec)
    log(event="campaign_started", todo=[s["id"] for s in queue], parallel=args.parallel)
    env = dict(os.environ, PYTHONHASHSEED="0")
    while (queue and stopped is None) or running:
        while queue and stopped is None and len(running) < args.parallel:
            spec = queue.pop(0)
            run_dir = REPO_ROOT / spec["out"]
            run_dir.mkdir(parents=True, exist_ok=True)
            out, err = (run_dir / "run.out").open("w", encoding="utf-8"), (run_dir / "run.err").open("w", encoding="utf-8")
            proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "run", "--config",
                                     spec["configuration"], "--nudge-time", str(spec["nudge_time"]), "--out",
                                     str(run_dir)], cwd=REPO_ROOT, env=env, stdout=out, stderr=err)
            running[spec["id"]] = (proc, spec, out, err)
            log(event="run_started", run=spec["id"])
            print(f"[robustness] started {spec['id']}", flush=True)
        time.sleep(5)
        for run_id, (proc, spec, out, err) in list(running.items()):
            if proc.poll() is None:
                continue
            out.close(), err.close()
            del running[run_id]
            run_dir = REPO_ROOT / spec["out"]
            problems = ["the run produced no run.json"] if run_state(run_dir) != "done" else \
                run_validity(run_dir, spec["nudge_time"])
            log(event="run_finished", run=run_id, exit=proc.returncode, problems=problems)
            print(f"[robustness] finished {run_id}: {'valid' if not problems else problems}", flush=True)
            if problems:
                stopped = f"{run_id}: {problems}"
    log(event="campaign_stopped" if stopped else "campaign_idle", reason=stopped)
    print(f"[robustness] {'STOPPED for diagnosis: ' + stopped if stopped else 'no runs left to start'}")
    return 1 if stopped else 0


# ---------------------------------------------------------------- analyse

def first_differing_step(a: Path, b: Path) -> Optional[float]:
    x, y = records(a, "step", ("duration",)), records(b, "step", ("duration",))
    for p, q in zip(x, y):
        if p != q:
            return float(p[0]["time"])
    return None


def analyse(args) -> int:
    plan_path = Path(args.plan)
    document = json.loads(plan_path.read_text(encoding="utf-8"))
    rows, problems = [], {}
    for spec in document["runs"]:
        run_dir = REPO_ROOT / spec["out"]
        if run_state(run_dir) != "done":
            problems[spec["id"]] = ["not run"]
            continue
        issues = run_validity(run_dir, spec["nudge_time"])
        if issues:
            problems[spec["id"]] = issues
        meta = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
        trial = json.loads((run_dir / "trial" / "trial.json").read_text(encoding="utf-8"))
        m, o = trial["metrics"], trial["objective"] or {}
        rows.append({"run": spec["id"], "configuration": spec["configuration"], "nudge_time": spec["nudge_time"],
                     "J": o.get("J"), "terms": o.get("terms"), "relative": o.get("relative"),
                     "finished": m["finished"], "scheduled": m["vehicles_scheduled"], "teleports": m["teleports"],
                     "responses": {k: {"value_s": v["value_s"], "status": v["status"]} for k, v in m["responses"].items()},
                     "nudge": meta["nudge"],
                     "first_differing_step_vs_unperturbed": first_differing_step(
                         run_dir / "trial" / "summary.xml", unperturbed_dir(spec["configuration"]) / "summary.xml")})
    unperturbed = {}
    for c in document["configurations"]:
        t = json.loads((unperturbed_dir(c["label"]) / "trial.json").read_text(encoding="utf-8"))
        unperturbed[c["label"]] = {"J": t["objective"]["J"], "finished": t["metrics"]["finished"],
                                   "teleports": t["metrics"]["teleports"],
                                   "responses": {k: v["value_s"] for k, v in t["metrics"]["responses"].items()},
                                   "source": str(unperturbed_dir(c["label"]).relative_to(REPO_ROOT))}
    report = {"plan": str(plan_path), "valid": not problems, "problems": problems, "runs": rows,
              "unperturbed_reported_not_scored": unperturbed,
              "label": "single-seed robustness check (contract 13.20), not a thesis performance claim"}
    if not problems:
        nudge_j = {c["label"]: {r["nudge_time"]: r["J"] for r in rows if r["configuration"] == c["label"]}
                   for c in document["configurations"]}
        trial_number = {c["label"]: c["trial"] for c in document["configurations"]}
        ranking = rank_configurations(nudge_j, trial_number)
        default = nudge_j["DEFAULT"]
        for entry in ranking:
            js = nudge_j[entry["configuration"]]
            entry.update({"nudge_J": js, "sd": (sum((j - entry["score"]) ** 2 for j in js.values()) / (len(js) - 1)) ** 0.5,
                          "worst_J": max(js.values()),
                          "paired_difference_vs_DEFAULT": {t: js[t] - default[t] for t in js},
                          "unperturbed_J": unperturbed[entry["configuration"]]["J"]})
        report.update({"ranking": ranking, "winner": ranking[0]["configuration"]})
    out = Path(args.out) if args.out else plan_path.parent / "robustness_report.json"
    if out.exists():
        raise SystemExit(f"{out} exists; evidence is never overwritten")
    out.write_text(json.dumps(report, indent=1), encoding="utf-8")
    print(f"report: {out}; valid: {report['valid']}; winner: {report.get('winner')}; problems: {problems or 'none'}")
    return 0 if report["valid"] else 1


def main() -> int:
    os.chdir(REPO_ROOT)                                  # the tuner reads simulation.sumocfg relative to the root
    sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="one 3,600 s run of a configuration, with or without its nudge")
    r.add_argument("--config", required=True, help="trial_NNN (from the tuning campaign) or DEFAULT")
    g = r.add_mutually_exclusive_group(required=True)
    g.add_argument("--nudge-time", type=float)
    g.add_argument("--no-nudge", action="store_true")
    r.add_argument("--out", required=True)
    r.add_argument("--sim-time", type=int, default=SIM_TIME, help="shorter only for a tool check, with --unscored")
    r.add_argument("--unscored", action="store_true", help="tool check only: no reference, no J; never one of the 24")
    c = sub.add_parser("check-reproduction", help="a no-nudge run against the original trial, bit for bit")
    c.add_argument("run")
    c.add_argument("original")
    p = sub.add_parser("plan", help="write the fixed 24-run plan from the finished campaign")
    p.add_argument("--out", required=True)
    p.add_argument("--expect", help="comma-separated trial numbers the rule must select, e.g. 10,16,11,17,12")
    k = sub.add_parser("campaign", help="run the planned runs not yet run")
    k.add_argument("--plan", required=True)
    k.add_argument("--parallel", type=int, default=3)
    a = sub.add_parser("analyse", help="validity, score, ranking and paired differences, per contract 13.20")
    a.add_argument("--plan", required=True)
    a.add_argument("--out")
    args = parser.parse_args()
    return {"run": run, "check-reproduction": check_reproduction, "plan": plan, "campaign": campaign,
            "analyse": analyse}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
