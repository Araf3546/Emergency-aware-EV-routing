#!/usr/bin/env python3
"""Held-out-seed evaluation of the final configuration T16, with T17 as a sensitivity arm (2026-09-16).

Diagnostic only. It never changes production code, routing, weights, the objective, the tuning campaign or the
finished robustness check, and it writes only into the folders it is given.

This tool implements the user's pre-registration of 2026-09-16 literally:

* **T16 is the final configuration, fixed before any held-out run** by the robustness procedure of contract 13.20.
  T17 is a sensitivity arm only. **Nothing here selects between them**: there is no winner, no ranking and no
  switching rule, whatever the results show.
* **Provenance gate first.** T16 and T17 are re-run at the tuning seed 20260721 from their provenance-bearing
  weight files, and each must reproduce its campaign trial. `campaign` refuses to start until the gate has passed.
* **Five held-out seeds** (20260722-26, contract 13.15), each with its own no-rerouting baseline reference built by
  the tuner's own builder. A baseline is never reused across seeds, and the tuning seed is never a held-out seed.
* **Primary outcome: emergency response against each seed's own baseline.** J and its parts are reported in full,
  but the primary thesis comparison is baseline against T16, not T16 against T17.

Weights are read only through the production loader `p3_hybrid_router.load_weights_file`, so a file that does not
carry the campaign's provenance cannot be evaluated at all. Every run re-checks the frozen code: the fingerprint
must equal `results/final_code_fingerprint_2026-09-15/` in every entry except that seed's own frozen demand file.

    python scripts/diagnostics/heldout_evaluation.py gate --out results/heldout_eval_2026-09-16/provenance_gate
    python scripts/diagnostics/heldout_evaluation.py plan --out results/heldout_eval_2026-09-16/plan.json
    python scripts/diagnostics/heldout_evaluation.py campaign --plan results/heldout_eval_2026-09-16/plan.json --parallel 3
    python scripts/diagnostics/heldout_evaluation.py analyse --plan results/heldout_eval_2026-09-16/plan.json

Sign conventions, used everywhere and never mixed:

* **baseline - T16** (primary): positive means T16 is faster or lower, negative means slower or higher, zero a tie.
* **T17 - T16** (secondary): negative means T17 is lower, positive means T16 is lower, zero a tie.

Censoring: an incident's normalized response is the treatment's seconds divided by the same incident's seconds in
that seed's own baseline, and it is defined only when both measurements are valid. Otherwise the ratio is
UNDEFINED: it is never substituted, never treated as zero, and it is excluded from the mean, which reports how many
incidents it used and why any was excluded. The frozen objective is untouched by this reporting rule and keeps its
own censoring treatment inside J.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CAMPAIGN = REPO_ROOT / "results/e3_tuning_campaign_2026-09-15"
FROZEN_RECORD = REPO_ROOT / "results/final_code_fingerprint_2026-09-15/campaign_settings.json"
CANDIDATE_DIR = REPO_ROOT / "results/final_weight_candidates_2026-09-16"
OUT_ROOT = "results/heldout_eval_2026-09-16"
FINAL_CONFIGURATION = "trial_016"          # fixed before any held-out run; never revisited here
SENSITIVITY_CONFIGURATION = "trial_017"
CONFIGURATIONS = (FINAL_CONFIGURATION, SENSITIVITY_CONFIGURATION)
INCIDENTS = ("404", "401", "405")
TUNING_SEED = 20260721
SIM_TIME = 3600
WEIGHT_NAMES = ("pheromone", "heuristic", "protocol", "bco", "velocity")
# metrics/tuning_objective.py records a measured response as "valid" (arrived inside the incident window) or
# "valid_post_window" (arrived after it). Anything else - teleported, not reached - is censored, not measured.
VALID_RESPONSE_STATUSES = ("valid", "valid_post_window")
RESPONSE_OBJECTIVE_WEIGHT = 0.30
DECOMPOSITION_TOLERANCE = 1e-9
# Two-sided 95% Student-t multipliers by degrees of freedom, for the small paired samples this design produces.
T_95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262}


def utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def tracebacks(*paths: Path) -> int:
    return sum(p.read_text(encoding="utf-8", errors="replace").count("Traceback") for p in paths if p.is_file())


def frozen_files() -> Dict[str, str]:
    return json.loads(FROZEN_RECORD.read_text(encoding="utf-8"))["settings"]["code_fingerprint"]


def frozen_digest() -> str:
    return json.loads(FROZEN_RECORD.read_text(encoding="utf-8"))["code_fingerprint_digest"]


def is_seed_demand(name: str) -> bool:
    """A frozen ordinary-demand file. Each evaluation seed has its own, so this entry is expected to differ."""
    return name == "demand/porto.rou.xml" or name.startswith("demand/seeds/porto.rou.")


def code_differences(files: Dict[str, str], frozen: Dict[str, str]) -> List[str]:
    """What differs from the frozen record, ignoring the seed's own frozen demand file (pure, tested)."""
    problems = []
    for name in sorted(set(frozen) | set(files)):
        if is_seed_demand(name):
            continue
        if name not in files:
            problems.append(f"{name}: missing from this run's fingerprint")
        elif name not in frozen:
            problems.append(f"{name}: not in the frozen record")
        elif files[name] != frozen[name]:
            problems.append(f"{name}: changed since the freeze")
    return problems


def seeds() -> List[int]:
    sys.path[:0] = [str(REPO_ROOT)]
    from evaluation_seeds import evaluation_seeds
    return [int(s) for s in evaluation_seeds()]


def reference_path(seed: int) -> Path:
    return REPO_ROOT / "results/tuning_reference" / f"seed_{int(seed)}_{SIM_TIME}s" / "reference.json"


def candidate_file(label: str) -> Path:
    if label not in CONFIGURATIONS:
        raise SystemExit(f"unknown configuration {label}; this evaluation carries exactly {CONFIGURATIONS}")
    return CANDIDATE_DIR / f"{label}_weights.json"


def candidate_weights(label: str) -> Dict[str, float]:
    """The weight set, read by the production loader, so campaign provenance is enforced here too."""
    sys.path[:0] = [str(REPO_ROOT)]
    from p3_hybrid_router import load_weights_file
    return load_weights_file(candidate_file(label))


def campaign_trial_dir(label: str) -> Path:
    return CAMPAIGN / "trials" / f"trial_{int(label.split('_')[1]):03d}"


def campaign_trial_weights(label: str) -> Dict[str, float]:
    trial = json.loads((campaign_trial_dir(label) / "trial.json").read_text(encoding="utf-8"))
    return {name: float(trial["weights_requested"][name]) for name in WEIGHT_NAMES}


# ---------------------------------------------------------------- the reporting rules (pure, tested)

def normalized_response(value_s: Optional[float], status: Optional[str],
                        baseline_s: Optional[float], baseline_status: Optional[str]) -> Tuple[Optional[float], str]:
    """One incident's normalized response, or UNDEFINED with the reason. Never substitutes a value."""
    if baseline_s is None or baseline_status not in VALID_RESPONSE_STATUSES:
        return None, f"baseline response censored or invalid (status {baseline_status})"
    if value_s is None or status not in VALID_RESPONSE_STATUSES:
        return None, f"run response censored or invalid (status {status})"
    if float(baseline_s) == 0.0:
        return None, "baseline response is zero"
    return float(value_s) / float(baseline_s), "defined"


def mean_normalized_response(ratios: Dict[str, Optional[float]]) -> dict:
    """The arithmetic mean over the incidents whose ratio is defined, with the count and the exclusions."""
    eligible = {k: v for k, v in ratios.items() if v is not None}
    excluded = sorted(k for k, v in ratios.items() if v is None)
    return {"mean": (sum(eligible.values()) / len(eligible)) if eligible else None,
            "eligible_incidents": sorted(eligible), "eligible_count": len(eligible),
            "excluded_incidents": excluded}


def worst_incident(ratios: Dict[str, Optional[float]]) -> Optional[str]:
    """The incident with the largest defined normalized response (the worst relative to its own baseline)."""
    eligible = {k: v for k, v in ratios.items() if v is not None}
    return max(eligible, key=eligible.get) if eligible else None


def paired_statistics(observations: Sequence[Optional[float]]) -> dict:
    """Mean, SD and a 95% interval of the paired observations that are defined (pure, tested).

    The interval is descriptive: with five deterministic seeds it is not a significance test, and this tool never
    reports whether it excludes zero as though that settled anything.
    """
    values = [float(v) for v in observations if v is not None]
    n = len(values)
    result = {"observations": list(observations), "n_defined": n, "mean": None, "sd": None,
              "ci95_halfwidth": None, "ci95": None}
    if n == 0:
        return result
    result["mean"] = sum(values) / n
    if n < 2:
        return result
    sd = (sum((v - result["mean"]) ** 2 for v in values) / (n - 1)) ** 0.5
    result["sd"] = sd
    t = T_95.get(n - 1)
    if t is not None:
        half = t * sd / math.sqrt(n)
        result["ci95_halfwidth"] = half
        result["ci95"] = [result["mean"] - half, result["mean"] + half]
        result["t_multiplier"] = t
        result["degrees_of_freedom"] = n - 1
    return result


def quadrant(delta_j: Optional[float], delta_response: Optional[float]) -> dict:
    """The descriptive quadrant of one seed, under the T17 - T16 convention. A tie is reported, never forced."""
    if delta_j is None or delta_response is None:
        return {"quadrant": None, "description": "not classifiable: a difference is undefined"}
    if delta_j == 0 or delta_response == 0:
        dimensions = [("J tied" if delta_j == 0 else ("T17 lower J" if delta_j < 0 else "T17 higher J")),
                      ("response tied" if delta_response == 0 else
                       ("T17 lower response" if delta_response < 0 else "T17 higher response"))]
        return {"quadrant": None, "description": "; ".join(dimensions) + " (tie, not forced into a quadrant)"}
    if delta_j < 0:
        return ({"quadrant": "A", "description": "T17 lower J and lower response"} if delta_response < 0 else
                {"quadrant": "B", "description": "T17 lower J but higher response"})
    return ({"quadrant": "C", "description": "T17 higher J but lower response"} if delta_response < 0 else
            {"quadrant": "D", "description": "T17 higher J and higher response"})


def decomposition(terms_a: Dict[str, float], terms_b: Dict[str, float], delta_j: float) -> dict:
    """The change in J against the sum of its five weighted component changes (the objective is additive)."""
    per_term = {k: terms_a[k] - terms_b[k] for k in terms_b}
    total = sum(per_term.values())
    return {"per_term": per_term, "sum_of_terms": total, "delta_J": delta_j, "residual": delta_j - total,
            "additive_within_tolerance": abs(delta_j - total) < DECOMPOSITION_TOLERANCE}


# ---------------------------------------------------------------- one baseline, one run

def _refuse_changed_code(seed: int, tuner) -> Dict[str, str]:
    files = tuner.evaluation_fingerprint(seed, Path("simulation.sumocfg"))
    problems = code_differences(files, frozen_files())
    if problems:
        raise SystemExit("the frozen code has changed; refusing to run:\n  " + "\n  ".join(problems))
    return files


def build_reference(args) -> int:
    sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
    import tune_e3_weights as tuner

    seed, out = int(args.seed), Path(args.out)
    target = reference_path(seed)
    if target.exists():
        raise SystemExit(f"{target} exists; a baseline is evidence and is never rebuilt over")
    _refuse_changed_code(seed, tuner)
    started, clock = utc(), time.time()
    reference = tuner.build_reference(Path("simulation.sumocfg"), seed, SIM_TIME, target.parent)
    out.mkdir(parents=True, exist_ok=True)
    m = reference["metrics"]
    (out / "run.json").write_text(json.dumps(
        {"kind": "held-out baseline reference build", "seed": seed, "sim_time": SIM_TIME,
         "reference": target.relative_to(REPO_ROOT).as_posix(), "reference_sha256": tuner._sha256(target),
         "frozen_demand": reference["frozen_demand"], "metrics": m,
         "started_utc": started, "finished_utc": utc(), "wall_seconds": round(time.time() - clock, 1)},
        indent=1), encoding="utf-8")
    print(json.dumps({"seed": seed, "finished": m["finished"], "scheduled": m["vehicles_scheduled"],
                      "teleports": m["teleports"],
                      "responses": {k: (v["value_s"], v["status"]) for k, v in m["responses"].items()}}))
    return 0


def run(args) -> int:
    sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
    import tune_e3_weights as tuner
    from p3_hybrid_router import TUNER_EVALUATOR

    seed, label, out = int(args.seed), args.config, Path(args.out)
    trial_dir = out / "trial"
    if (out / "run.json").exists() or (trial_dir.exists() and any(trial_dir.iterdir())):
        raise SystemExit(f"{out} already holds a run; evidence is never overwritten")
    files = _refuse_changed_code(seed, tuner)
    reference_file = reference_path(seed)
    if not reference_file.is_file():
        raise SystemExit(f"no baseline reference for seed {seed} at {reference_file}; build it first")
    weights = candidate_weights(label)                      # the production loader: provenance enforced
    if weights != campaign_trial_weights(label):
        raise SystemExit(f"{candidate_file(label)} does not hold the campaign weights of {label}")
    reference = tuner.load_reference(reference_file, seed, SIM_TIME)
    out.mkdir(parents=True, exist_ok=True)
    started, clock = utc(), time.time()
    rejected = None
    try:
        tuner.evaluate_weights(weights, Path("simulation.sumocfg"), SIM_TIME, seed, trial_dir=trial_dir,
                               reference=reference)
    except tuner.TrialRejected as exc:
        rejected = str(exc)
    (out / "run.json").write_text(json.dumps(
        {"completed": True, "configuration": label, "seed": seed, "sim_time": SIM_TIME, "weights": weights,
         "weights_file": candidate_file(label).relative_to(REPO_ROOT).as_posix(),
         "weights_file_sha256": tuner._sha256(candidate_file(label)),
         "reference": reference_file.relative_to(REPO_ROOT).as_posix(),
         "reference_sha256": tuner._sha256(reference_file), "rejected": rejected,
         "evaluator": TUNER_EVALUATOR, "code_fingerprint_digest": tuner._digest(files),
         "frozen_digest_tuning_seed": frozen_digest(), "code_unchanged_except_seed_demand": True,
         "pythonhashseed": os.environ.get("PYTHONHASHSEED", "unset"),
         "started_utc": started, "finished_utc": utc(), "wall_seconds": round(time.time() - clock, 1)},
        indent=1), encoding="utf-8")
    print(json.dumps({"configuration": label, "seed": seed, "rejected": rejected}))
    return 0 if rejected is None else 1


# ---------------------------------------------------------------- the provenance gate

# Exactly what the comparison covers. Nothing outside this list is claimed to have been verified.
TRIAL_KEYS = ("evaluator", "road_permissions", "ambient_road_classes", "seed", "frozen_demand", "sim_time",
              "weights_requested", "weights_used", "exceptions_logged", "coordinator", "learning", "route_stats",
              "metrics", "objective", "finished_equals_summary_arrived", "no_tripinfo_record_outside_population",
              "pairing")
COMPARED_FIELDS = list(TRIAL_KEYS) + ["J", "every tripinfo record", "every summary step (excluding SUMO's "
                                      "wall-clock duration attribute)", "every SUMO warning and error line",
                                      "tracebacks in the run and in the SUMO log"]


def _records(path: Path, tag: str, ignore=()):
    import xml.etree.ElementTree as ET
    return [({k: v for k, v in e.attrib.items() if k not in ignore}, [(c.tag, dict(c.attrib)) for c in e])
            for e in ET.parse(path).getroot().iter(tag)]


def _sumo_lines(path: Path) -> list:
    return [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.startswith(("Warning", "Error"))]


def compare_with_campaign_trial(run_dir: Path, label: str, verbose: bool = True) -> List[str]:
    """One tuning-seed run against its campaign trial. Returns the list of fields that differ."""
    original = campaign_trial_dir(label)
    new_trial = run_dir / "trial"
    failures: List[str] = []

    def same(what, x, y):
        ok = x == y
        if not ok:
            failures.append(what)
        if verbose:
            print(f"  {'same     ' if ok else 'DIFFERENT'} {what}"
                  + ("" if ok else f": {str(x)[:110]} | {str(y)[:110]}"))

    meta = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    same("the run was not rejected", None, meta["rejected"])
    same("the run used the tuning seed", TUNING_SEED, meta["seed"])
    same("the run was the contracted 3,600 s", SIM_TIME, meta["sim_time"])
    same("the weights from the provenance file equal the campaign trial's",
         meta["weights"], campaign_trial_weights(label))
    same("the code fingerprint equals the frozen record", frozen_digest(), meta["code_fingerprint_digest"])
    a = json.loads((new_trial / "trial.json").read_text(encoding="utf-8"))
    b = json.loads((original / "trial.json").read_text(encoding="utf-8"))
    for key in TRIAL_KEYS:
        same(f"trial.json {key}", a.get(key), b.get(key))
    same("J", (a.get("objective") or {}).get("J"), (b.get("objective") or {}).get("J"))
    for name, tag, ignore in (("tripinfo.xml", "tripinfo", ()), ("summary.xml", "step", ("duration",))):
        x, y = _records(new_trial / name, tag, ignore), _records(original / name, tag, ignore)
        same(f"{name} records ({len(x)} / {len(y)})", x, y)
    wx, wy = _sumo_lines(new_trial / "errors.log"), _sumo_lines(original / "errors.log")
    same(f"SUMO warning and error lines ({len(wx)} / {len(wy)})", wx, wy)
    same("tracebacks in the run", 0, tracebacks(run_dir / "run.out", run_dir / "run.err", new_trial / "errors.log"))
    return failures


def check_reproduction(args) -> int:
    failures = compare_with_campaign_trial(Path(args.run), args.config)
    print("REPRODUCED" if not failures else "NOT REPRODUCED: " + ", ".join(failures))
    return 0 if not failures else 1


def gate(args) -> int:
    """Both configurations re-run at the tuning seed from their weight files, then compared with their trials."""
    out = Path(args.out).resolve()
    result_file = out / "gate_result.json"
    if result_file.exists():
        raise SystemExit(f"{result_file} exists; the gate's result is evidence and is written once")
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONHASHSEED="0")
    processes, finished = {}, {}
    for label in CONFIGURATIONS:
        run_dir = out / f"reproduction_{label}"
        state = run_state(run_dir)
        if state == "done":
            # This reproduction already produced its result. It is deterministic, so re-running it would only
            # reproduce the same numbers; the finished run is compared as it stands.
            finished[label] = run_dir
            print(f"[gate] {label} already has a finished run; reusing it", flush=True)
            continue
        if state == "interrupted":
            # No run.json means the run produced no result (a kill, a power cut). The partial folder is kept as
            # evidence and the identical specification runs again. This is not a replacement run.
            k = 1
            while run_dir.with_name(f"{run_dir.name}_interrupted_{k}").exists():
                k += 1
            run_dir.rename(run_dir.with_name(f"{run_dir.name}_interrupted_{k}"))
            print(f"[gate] {label} was interrupted and produced no result; kept as "
                  f"{run_dir.name}_interrupted_{k} and run again", flush=True)
        run_dir.mkdir(parents=True, exist_ok=True)
        handles = ((run_dir / "run.out").open("w", encoding="utf-8"),
                   (run_dir / "run.err").open("w", encoding="utf-8"))
        processes[label] = (subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "run", "--config", label,
             "--seed", str(TUNING_SEED), "--out", str(run_dir)],
            cwd=REPO_ROOT, env=env, stdout=handles[0], stderr=handles[1]), handles, run_dir)
        print(f"[gate] started {label} at the tuning seed", flush=True)
    outcome = {}
    for label, run_dir in finished.items():
        processes.setdefault(label, (None, (), run_dir))
    for label, (proc, handles, run_dir) in processes.items():
        code = proc.wait() if proc is not None else 0
        for handle in handles:
            handle.close()
        if not (run_dir / "run.json").is_file():
            outcome[label] = {"passed": False, "failures": ["the run produced no run.json"], "exit": code,
                              "run": run_dir.resolve().relative_to(REPO_ROOT).as_posix()}
            continue
        print(f"[gate] comparing {label} with its campaign trial", flush=True)
        failures = compare_with_campaign_trial(run_dir, label)
        outcome[label] = {"passed": not failures, "failures": failures, "exit": code,
                          "run": run_dir.resolve().relative_to(REPO_ROOT).as_posix(),
                          "campaign_trial": campaign_trial_dir(label).relative_to(REPO_ROOT).as_posix()}
    passed = all(v["passed"] for v in outcome.values())
    result_file.write_text(json.dumps(
        {"kind": "provenance reproduction gate", "utc": utc(), "seed": TUNING_SEED, "passed": passed,
         "compared_fields": COMPARED_FIELDS, "configurations": outcome}, indent=1), encoding="utf-8")
    print(f"[gate] {'PASSED' if passed else 'FAILED'}; written {result_file}")
    return 0 if passed else 1


def gate_passed() -> bool:
    result = REPO_ROOT / OUT_ROOT / "provenance_gate" / "gate_result.json"
    return result.is_file() and json.loads(result.read_text(encoding="utf-8")).get("passed") is True


# ---------------------------------------------------------------- plan

def evaluation_matrix(seed_list: List[int]) -> List[dict]:
    """Each seed's baseline first, then both configurations at that seed (pure, tested)."""
    specs = [{"id": f"baseline_seed_{s}", "kind": "baseline", "seed": s, "out": f"{OUT_ROOT}/baselines/seed_{s}"}
             for s in seed_list]
    specs += [{"id": f"{c}_seed_{s}", "kind": "evaluation", "configuration": c, "seed": s,
               "out": f"{OUT_ROOT}/runs/{c}_seed_{s}"} for s in seed_list for c in CONFIGURATIONS]
    return specs


def report_specification() -> dict:
    """Frozen with the plan, before any run: exactly what will be reported and how."""
    return {
        "primary_outcome": ("emergency response of T16 against that seed's own no-rerouting baseline reference, "
                            "per incident 404 / 401 / 405 and as the mean normalized response"),
        "per_run_layers": {
            "raw": ["travel_s", "time_loss_s", "response seconds and status per incident", "finished",
                    "vehicles_scheduled", "completion_share", "incomplete_share", "teleports"],
            "baseline_relative": ["travel", "response", "time_loss", "incomplete", "teleports",
                                  "normalized response per incident and its mean"],
            "weighted_terms": ["travel", "response", "time_loss", "incomplete", "teleports", "J"],
        },
        "weighted_terms_are_not_raw_units": True,
        "normalized_response": ("incident seconds / the same incident's seconds in that seed's own baseline, when "
                                "both are valid; the objective's response contribution is "
                                f"{RESPONSE_OBJECTIVE_WEIGHT} x mean normalized response, so the normalized "
                                "response is not itself the weighted term"),
        "censoring_rule": ("if either the baseline or the run measurement of an incident is censored or invalid, "
                           "that incident's ratio is UNDEFINED: no ratio, no substitution, not zero; raw values, "
                           "statuses and the reason are reported and the incident is excluded from the mean, whose "
                           "eligible count is always reported. J keeps the frozen objective's own censoring"),
        "valid_response_statuses": list(VALID_RESPONSE_STATUSES),
        "worst_incident": "the incident with the largest defined normalized ratio, with its seconds and status",
        "sign_conventions": {
            "primary_baseline_minus_T16": "positive = T16 faster or lower; negative = T16 slower or higher; 0 = tie",
            "secondary_T17_minus_T16": "negative = T17 lower; positive = T16 lower; 0 = tie",
        },
        "aggregate": ["the per-seed observations, always listed", "mean", "SD",
                      "95% interval (Student-t, descriptive only, never presented as a significance threshold)"],
        "secondary_comparison": ["delta J", "delta mean normalized response", "delta per incident",
                                 "descriptive quadrant, with ties reported as ties",
                                 "additive check: delta J = the sum of the five weighted term differences"],
        "not_reported": ["baseline J against T16 J, because the baseline's J is 1.0 by construction",
                         "any invented trade-off score, ratio or composite",
                         "any winner, ranking or selection between T16 and T17"],
        "interpretation_order": [
            "does T16 improve emergency response against each seed's own baseline",
            "the incident-specific and aggregate response differences",
            "completion, teleports, travel time and time loss",
            "J", "the five component contributions to J",
            "how similar or different T17 is from T16",
            "the limitations of a five-seed deterministic held-out design"],
    }


def plan(args) -> int:
    out = Path(args.out).resolve()
    if out.exists():
        raise SystemExit(f"{out} exists; a pre-registration is fixed once written")
    seed_list = seeds()
    if len(seed_list) != 5 or TUNING_SEED in seed_list:
        raise SystemExit(f"expected the 5 contracted evaluation seeds without the tuning seed, found {seed_list}")
    runs = evaluation_matrix(seed_list)
    document = {
        "kind": "held-out-seed evaluation: pre-registration, fixed before any run",
        "written_utc": utc(),
        "contract": "13.15 (evaluation seeds), 13.17 (objective), 13.20 (how T16 and T17 were chosen)",
        "final_configuration": FINAL_CONFIGURATION,
        "sensitivity_configuration": SENSITIVITY_CONFIGURATION,
        "selection_statement": ("T16 was selected before any held-out run by the pre-registered robustness "
                               "procedure. The held-out results select nothing: under no outcome is the final "
                               "configuration replaced, re-tuned, switched or supplemented, and this tool "
                               "computes no winner"),
        "seeds": seed_list,
        "tuning_seed_excluded": TUNING_SEED,
        "sim_time": SIM_TIME,
        "emergency_layer": "on",
        "configurations": [{"label": c, "role": "final" if c == FINAL_CONFIGURATION else "sensitivity",
                            "weights_file": candidate_file(c).relative_to(REPO_ROOT).as_posix(),
                            "weights": candidate_weights(c), "source_trial": int(c.split("_")[1]),
                            "campaign_trial_record": campaign_trial_dir(c).relative_to(REPO_ROOT).as_posix(),
                            "tuning_seed_J": json.loads((campaign_trial_dir(c) / "trial.json")
                                                        .read_text(encoding="utf-8"))["objective"]["J"]}
                           for c in CONFIGURATIONS],
        "provenance_gate": {
            "required_before_any_held_out_run": True,
            "runs": [{"configuration": c, "seed": TUNING_SEED,
                      "out": f"{OUT_ROOT}/provenance_gate/reproduction_{c}",
                      "must_match": campaign_trial_dir(c).relative_to(REPO_ROOT).as_posix()}
                     for c in CONFIGURATIONS],
            "compared_fields": COMPARED_FIELDS,
            "on_failure": "stop completely; no held-out run is started",
        },
        "baseline_handling": ("each seed's own no-rerouting baseline reference, built once by the tuner's own "
                             "--build-reference at 3,600 s under the same road-permission rule; never reused "
                             "across seeds, and the tuning seed's canonical baseline is never reused here"),
        "frozen_code_digest_tuning_seed": frozen_digest(),
        "code_check_per_run": ("the evaluation fingerprint must equal the frozen record in every entry except that "
                              "seed's own frozen demand file"),
        "report_specification": report_specification(),
        "runs": runs,
        "expected_run_count": {"provenance_gate": len(CONFIGURATIONS), "baselines": len(seed_list),
                               "evaluation": len(seed_list) * len(CONFIGURATIONS),
                               "total": len(CONFIGURATIONS) + len(runs)},
        "invalid_run_rule": ["any traceback, logged exception, rejected run, changed fingerprint, wrong baseline or "
                             "other predefined invalid condition stops the campaign",
                             "no invalid run is replaced with an extra seed",
                             "no runs are added to compensate",
                             "the stage is reported as incomplete until the issue is resolved without changing "
                             "this frozen methodology"],
        "frozen_after_first_run": ["this plan", "the metric list", "the report specification"],
        "artifacts": {"plan": out.relative_to(REPO_ROOT).as_posix(),
                      "gate_result": f"{OUT_ROOT}/provenance_gate/gate_result.json",
                      "per_run_results": f"{OUT_ROOT}/(baselines|runs)/<id>/run.json and trial/",
                      "final_report": f"{OUT_ROOT}/heldout_report.json",
                      "events": f"{OUT_ROOT}/campaign_events.jsonl",
                      "final_report_written_once": True},
        "authorized_limitations": [
            "DEFAULT is not run on the held-out seeds, so no held-out claim can be made that the tuned weights beat "
            "untuned E3; the held-out claims are T16 against no rerouting, and T17 against T16 as sensitivity",
            "five deterministic seeds, one run per configuration per seed; no repeats and no perturbations",
            "the same three incidents at the same times in every run"],
        "methodology_freeze": ["no new objective weights", "no new tuning", "no changed reroute trigger",
                               "no changed yielding mechanism", "no changed emergency protocol",
                               "no added DEFAULT arm", "no new seeds", "no added repeats",
                               "no post-hoc metric definitions"],
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(document, indent=1), encoding="utf-8")
    print(f"plan: {out}; {len(runs)} runs after a {len(CONFIGURATIONS)}-run gate; final configuration "
          f"{FINAL_CONFIGURATION}, sensitivity {SENSITIVITY_CONFIGURATION}; no selection rule")
    return 0


# ---------------------------------------------------------------- campaign

def run_state(run_dir: Path) -> str:
    if (run_dir / "run.json").is_file():
        return "done"
    return "interrupted" if run_dir.exists() and any(run_dir.iterdir()) else "todo"


def run_validity(spec: dict, run_dir: Path) -> List[str]:
    problems = []
    meta = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    if tracebacks(run_dir / "run.out", run_dir / "run.err"):
        problems.append("traceback")
    if meta.get("seed") != spec["seed"] or meta.get("sim_time") != SIM_TIME:
        problems.append(f"not a {SIM_TIME} s run at seed {spec['seed']}")
    if spec["kind"] == "baseline":
        if not reference_path(spec["seed"]).is_file():
            problems.append("no reference written")
        return problems
    trial = json.loads((run_dir / "trial" / "trial.json").read_text(encoding="utf-8"))
    if meta.get("rejected"):
        problems.append(f"rejected: {meta['rejected']}")
    if trial.get("exceptions_logged"):
        problems.append(f"{len(trial['exceptions_logged'])} exception(s) logged")
    if tracebacks(run_dir / "trial" / "errors.log"):
        problems.append("traceback in the SUMO log")
    if meta.get("code_fingerprint_digest") is None or not meta.get("code_unchanged_except_seed_demand"):
        problems.append("the frozen code check is missing")
    if meta.get("weights") != campaign_trial_weights(spec["configuration"]):
        problems.append("the weights are not the campaign trial's")
    if meta.get("reference") != reference_path(spec["seed"]).relative_to(REPO_ROOT).as_posix():
        problems.append("scored against another seed's baseline")
    if not (trial.get("objective") or {}).get("J"):
        problems.append("no J")
    return problems


def campaign(args) -> int:
    plan_path = Path(args.plan)
    document = json.loads(plan_path.read_text(encoding="utf-8"))
    if not gate_passed():
        raise SystemExit("the provenance gate has not passed; run `gate` first and do not start held-out runs")
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
            # No run.json means the run produced no result (power cut, kill). It is deterministic, so the identical
            # specification runs again and the partial folder is kept as evidence. This is not a replacement run.
            k = 1
            while run_dir.with_name(f"{run_dir.name}_interrupted_{k}").exists():
                k += 1
            run_dir.rename(run_dir.with_name(f"{run_dir.name}_interrupted_{k}"))
            log(event="interrupted_run_set_aside", run=spec["id"], kept_as=f"{run_dir.name}_interrupted_{k}")
        queue.append(spec)
    log(event="campaign_started", todo=[s["id"] for s in queue], parallel=args.parallel)
    env = dict(os.environ, PYTHONHASHSEED="0")
    while (queue and stopped is None) or running:
        launched = False
        for spec in list(queue):
            if stopped is not None or len(running) >= args.parallel:
                break
            if spec["kind"] == "evaluation" and not reference_path(spec["seed"]).is_file():
                continue                            # its baseline is still building; the plan schedules it first
            queue.remove(spec)
            run_dir = REPO_ROOT / spec["out"]
            run_dir.mkdir(parents=True, exist_ok=True)
            out = (run_dir / "run.out").open("w", encoding="utf-8")
            err = (run_dir / "run.err").open("w", encoding="utf-8")
            command = [sys.executable, str(Path(__file__).resolve()),
                       "build-reference" if spec["kind"] == "baseline" else "run",
                       "--seed", str(spec["seed"]), "--out", str(run_dir)]
            if spec["kind"] == "evaluation":
                command[3:3] = ["--config", spec["configuration"]]
            running[spec["id"]] = (subprocess.Popen(command, cwd=REPO_ROOT, env=env, stdout=out, stderr=err),
                                   spec, out, err)
            log(event="run_started", run=spec["id"])
            print(f"[heldout] started {spec['id']}", flush=True)
            launched = True
        if not launched and not running and queue:
            stopped = "a baseline is missing for the remaining runs"
            break
        time.sleep(5)
        for run_id, (proc, spec, out, err) in list(running.items()):
            if proc.poll() is None:
                continue
            out.close(), err.close()
            del running[run_id]
            run_dir = REPO_ROOT / spec["out"]
            problems = ["the run produced no run.json"] if run_state(run_dir) != "done" \
                else run_validity(spec, run_dir)
            log(event="run_finished", run=run_id, exit=proc.returncode, problems=problems)
            print(f"[heldout] finished {run_id}: {'valid' if not problems else problems}", flush=True)
            if problems:
                stopped = f"{run_id}: {problems}"
    log(event="campaign_stopped" if stopped else "campaign_idle", reason=stopped)
    print(f"[heldout] {'STOPPED for diagnosis: ' + stopped if stopped else 'no runs left to start'}")
    return 1 if stopped else 0


# ---------------------------------------------------------------- analyse

def _run_row(run_dir: Path, spec: dict) -> dict:
    meta = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    trial = json.loads((run_dir / "trial" / "trial.json").read_text(encoding="utf-8"))
    m, o = trial["metrics"], trial["objective"] or {}
    return {
        "run": spec["id"], "configuration": spec["configuration"], "seed": spec["seed"],
        "raw": {"travel_s": m["travel_s"], "time_loss_s": m["time_loss_s"], "finished": m["finished"],
                "vehicles_scheduled": m["vehicles_scheduled"],
                "completion_share": m["finished"] / m["vehicles_scheduled"],
                "incomplete_share": m["incomplete_share"], "teleports": m["teleports"],
                "teleports_per_vehicle": m["teleports_per_vehicle"],
                "responses": {k: {"value_s": v["value_s"], "status": v["status"]}
                              for k, v in m["responses"].items()}},
        "baseline_relative": o.get("relative"),
        "weighted_terms": o.get("terms"), "J": o.get("J"),
        "provenance": {"weights_file": meta.get("weights_file"),
                       "weights_file_sha256": meta.get("weights_file_sha256"),
                       "reference": meta.get("reference"), "reference_sha256": meta.get("reference_sha256"),
                       "code_fingerprint_digest": meta.get("code_fingerprint_digest"),
                       "evaluator": meta.get("evaluator"), "wall_seconds": meta.get("wall_seconds")},
    }


def _normalized_block(responses: dict, baseline_responses: dict) -> dict:
    ratios, reasons = {}, {}
    for incident in INCIDENTS:
        run_value = responses.get(incident) or {}
        base_value = baseline_responses.get(incident) or {}
        ratio, reason = normalized_response(run_value.get("value_s"), run_value.get("status"),
                                            base_value.get("value_s"), base_value.get("status"))
        ratios[incident], reasons[incident] = ratio, reason
    block = mean_normalized_response(ratios)
    block.update({"per_incident": ratios, "reasons": reasons, "worst_incident": worst_incident(ratios)})
    worst = block["worst_incident"]
    if worst:
        block["worst_incident_detail"] = {"incident": worst, "normalized": ratios[worst],
                                          "seconds": responses[worst]["value_s"],
                                          "status": responses[worst]["status"]}
    return block


def analyse(args) -> int:
    document = json.loads(Path(args.plan).read_text(encoding="utf-8"))
    final, sensitivity = document["final_configuration"], document["sensitivity_configuration"]
    rows, baselines, problems = {}, {}, {}
    for spec in document["runs"]:
        run_dir = REPO_ROOT / spec["out"]
        if run_state(run_dir) != "done":
            problems[spec["id"]] = ["not run"]
            continue
        issues = run_validity(spec, run_dir)
        if issues:
            problems[spec["id"]] = issues
        if spec["kind"] == "baseline":
            reference = json.loads(reference_path(spec["seed"]).read_text(encoding="utf-8"))
            m = reference["metrics"]
            baselines[spec["seed"]] = {
                "reference": reference_path(spec["seed"]).relative_to(REPO_ROOT).as_posix(),
                "raw": {"travel_s": m["travel_s"], "time_loss_s": m["time_loss_s"], "finished": m["finished"],
                        "vehicles_scheduled": m["vehicles_scheduled"],
                        "completion_share": m["finished"] / m["vehicles_scheduled"],
                        "incomplete_share": m["incomplete_share"], "teleports": m["teleports"],
                        "responses": {k: {"value_s": v["value_s"], "status": v["status"]}
                                      for k, v in m["responses"].items()}}}
            continue
        rows[(spec["configuration"], spec["seed"])] = _run_row(run_dir, spec)

    report = {
        "plan": str(args.plan), "written_utc": utc(), "valid": not problems, "problems": problems,
        "final_configuration": final, "sensitivity_configuration": sensitivity,
        "selection_statement": document["selection_statement"],
        "sign_conventions": document["report_specification"]["sign_conventions"],
        "censoring_rule": document["report_specification"]["censoring_rule"],
        "authorized_limitations": document["authorized_limitations"],
        "label": ("held-out evaluation of the pre-selected final configuration, with a sensitivity arm; "
                  "no winner is computed and no selection is made from these results"),
    }
    out = Path(args.out) if args.out else Path(args.plan).parent / "heldout_report.json"
    if out.exists():
        raise SystemExit(f"{out} exists; the final report is written once and never overwritten")
    if problems:
        report["per_seed"] = {}
        out.write_text(json.dumps(report, indent=1), encoding="utf-8")
        print(f"report: {out}; valid: False; the stage is INCOMPLETE; problems: {problems}")
        return 1

    per_seed = {}
    primary = {incident: [] for incident in INCIDENTS}
    primary["mean_normalized_response"] = []
    secondary = {"J": [], "normalized_response": [], **{incident: [] for incident in INCIDENTS}}
    for seed in document["seeds"]:
        base = baselines[seed]["raw"]["responses"]
        block = {"baseline": baselines[seed]}
        for label in (final, sensitivity):
            row = rows[(label, seed)]
            row["normalized_response"] = _normalized_block(row["raw"]["responses"], base)
            block[label] = row
        final_row = block[final]
        differences = {}
        for incident in INCIDENTS:
            b = base.get(incident, {})
            t = final_row["raw"]["responses"].get(incident, {})
            defined = (b.get("status") in VALID_RESPONSE_STATUSES and t.get("status") in VALID_RESPONSE_STATUSES)
            value = (b["value_s"] - t["value_s"]) if defined else None
            differences[incident] = {"baseline_s": b.get("value_s"), "baseline_status": b.get("status"),
                                     "final_s": t.get("value_s"), "final_status": t.get("status"),
                                     "baseline_minus_final_s": value,
                                     "normalized_final": final_row["normalized_response"]["per_incident"][incident]}
            primary[incident].append(value)
        primary["mean_normalized_response"].append(final_row["normalized_response"]["mean"])
        block["primary_baseline_minus_final"] = differences

        other, chosen = block[sensitivity], block[final]
        delta_j = other["J"] - chosen["J"]
        delta_norm = (None if other["normalized_response"]["mean"] is None
                      or chosen["normalized_response"]["mean"] is None
                      else other["normalized_response"]["mean"] - chosen["normalized_response"]["mean"])
        per_incident = {}
        for incident in INCIDENTS:
            x = other["raw"]["responses"].get(incident, {})
            y = chosen["raw"]["responses"].get(incident, {})
            defined = (x.get("status") in VALID_RESPONSE_STATUSES and y.get("status") in VALID_RESPONSE_STATUSES)
            per_incident[incident] = (x["value_s"] - y["value_s"]) if defined else None
            secondary[incident].append(per_incident[incident])
        secondary["J"].append(delta_j)
        secondary["normalized_response"].append(delta_norm)
        block["secondary_sensitivity_minus_final"] = {
            "delta_J": delta_j, "delta_mean_normalized_response": delta_norm,
            "delta_per_incident_s": per_incident, "classification": quadrant(delta_j, delta_norm),
            "additive_decomposition": decomposition(other["weighted_terms"], chosen["weighted_terms"], delta_j)}
        per_seed[str(seed)] = block

    report["per_seed"] = per_seed
    report["aggregate"] = {
        "primary_baseline_minus_final": {
            **{incident: paired_statistics(primary[incident]) for incident in INCIDENTS},
            "mean_normalized_response_of_final": paired_statistics(primary["mean_normalized_response"])},
        "secondary_sensitivity_minus_final": {
            "delta_J": paired_statistics(secondary["J"]),
            "delta_mean_normalized_response": paired_statistics(secondary["normalized_response"]),
            **{incident: paired_statistics(secondary[incident]) for incident in INCIDENTS}},
        "mean_J": {label: sum(rows[(label, s)]["J"] for s in document["seeds"]) / len(document["seeds"])
                   for label in (final, sensitivity)},
        "quadrants": {str(s): per_seed[str(s)]["secondary_sensitivity_minus_final"]["classification"]["quadrant"]
                      for s in document["seeds"]},
        "interval_note": ("the 95% intervals are descriptive summaries of five deterministic paired observations; "
                          "they are not significance tests and no threshold is implied"),
    }
    out.write_text(json.dumps(report, indent=1), encoding="utf-8")
    print(f"report: {out}; valid: True; final configuration {final} (fixed in advance; nothing selected here)")
    return 0


def main() -> int:
    os.chdir(REPO_ROOT)                                  # the tuner reads simulation.sumocfg relative to the root
    sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    b = sub.add_parser("build-reference", help="the no-rerouting baseline reference of one seed")
    b.add_argument("--seed", type=int, required=True)
    b.add_argument("--out", required=True)
    r = sub.add_parser("run", help="one 3,600 s run of one configuration at one seed")
    r.add_argument("--config", required=True, choices=CONFIGURATIONS)
    r.add_argument("--seed", type=int, required=True)
    r.add_argument("--out", required=True)
    g = sub.add_parser("gate", help="the provenance gate: both configurations reproduced at the tuning seed")
    g.add_argument("--out", default=f"{OUT_ROOT}/provenance_gate")
    c = sub.add_parser("check-reproduction", help="one tuning-seed run against its campaign trial")
    c.add_argument("run")
    c.add_argument("--config", required=True, choices=CONFIGURATIONS)
    p = sub.add_parser("plan", help="write the fixed pre-registration")
    p.add_argument("--out", required=True)
    k = sub.add_parser("campaign", help="run the planned runs not yet run (refuses until the gate has passed)")
    k.add_argument("--plan", required=True)
    k.add_argument("--parallel", type=int, default=3)
    a = sub.add_parser("analyse", help="the frozen report specification, applied once")
    a.add_argument("--plan", required=True)
    a.add_argument("--out")
    args = parser.parse_args()
    return {"build-reference": build_reference, "run": run, "gate": gate, "check-reproduction": check_reproduction,
            "plan": plan, "campaign": campaign, "analyse": analyse}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
