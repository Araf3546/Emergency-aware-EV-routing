#!/usr/bin/env python3
"""Tuner smoke test: does a tuning trial evaluate the algorithm that is reported?

Not tuning. Nothing here searches for weights or writes a weight set anyone should use.

    # 1. identity: the tuner's evaluator with DEFAULT_WEIGHTS must reproduce a saved production
    #    P3 run (same seed, duration, weights) trip record for trip record
    python scripts/diagnostics/tuner_smoke.py identity --out DIR \
        --reference results/pd1_verification_2026-09-13/p3on_s21_1800
    #    (the pre-PD-1 run results/e3_component_validation_2026-09-12 is no longer a valid
    #    reference: PD-1 shifts the random stream after t = 947 - see that folder's README.
    #    PD-1 question 2, later on 2026-09-13, also postdates p3on_s21_1800: ambient vehicles no
    #    longer drive through closures, so first make a new reference run on current code with
    #    `pd1_verify.py --run`)

    # 2. isolation: re-run one trial of a finished study in a fresh process; it must be identical
    #    to what the study recorded, so nothing leaked into it from the trials before it
    python scripts/diagnostics/tuner_smoke.py repeat --study STUDY_DIR --trial 1 --out DIR

    # 3. loading: the study's best_e3_weights.json loads, and the runtime uses it when configured
    python scripts/diagnostics/tuner_smoke.py load --study STUDY_DIR

    # 4. the objective (D1-D8): every trial's J recomputed from its files, counters cross-checked
    python scripts/diagnostics/tuner_smoke.py objective --study STUDY_DIR

Every subcommand prints PASS/FAIL lines and writes a JSON verdict next to its outputs.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)


def tripinfo(path: Path) -> dict:
    """Finished trips only: tuner trials also write unfinished vehicles (arrival -1) since 2026-09-13."""
    return {e.get("id"): dict(e.attrib) for e in ET.parse(path).getroot().iter("tripinfo")
            if float(e.get("arrival", -1)) >= 0}


def compare_tripinfo(a: Path, b: Path) -> dict:
    ta, tb = tripinfo(a), tripinfo(b)
    common = set(ta) & set(tb)
    differing = sorted(k for k in common if ta[k] != tb[k])
    return {
        "trips_a": len(ta), "trips_b": len(tb),
        "only_in_a": len(set(ta) - set(tb)), "only_in_b": len(set(tb) - set(ta)),
        "records_differing": len(differing), "first_differing": differing[:5],
        "identical": len(ta) == len(tb) and not differing and set(ta) == set(tb),
    }


def report(name: str, checks: dict, out: Path) -> int:
    ok = all(v for v in checks.values() if isinstance(v, bool))
    for key, value in checks.items():
        if isinstance(value, bool):
            print(f"{'PASS' if value else 'FAIL'}  {key}")
    print(f"{name}: {'PASS' if ok else 'FAIL'}")
    (out / f"verdict_{name}.json").write_text(json.dumps({"pass": ok, **checks}, indent=1,
                                                         default=str), encoding="utf-8")
    return 0 if ok else 1


def identity(args) -> int:
    from p3_hybrid_router import DEFAULT_WEIGHTS
    from scripts.tune_e3_weights import evaluate_weights

    out, ref = Path(args.out), Path(args.reference)
    meta = json.loads((ref / "meta.json").read_text(encoding="utf-8"))
    result = evaluate_weights(dict(DEFAULT_WEIGHTS), Path("simulation.sumocfg"),
                              int(meta["duration_s"]), int(meta["seed"]), trial_dir=out)
    (out / "metrics.json").write_text(json.dumps(result.to_dict(), indent=1), encoding="utf-8")
    trips = compare_tripinfo(out / "tripinfo.xml", ref / "tripinfo.xml")
    ours, theirs = result.checks["coordinator"], meta["counters"]
    counters = {k: (ours[k], theirs[k2]) for k, k2 in (
        ("reroute_decisions", "reroute_decisions"), ("reroute_executed", "reroute_executed"),
        ("candidate_edges_scored", "candidate_edges_scored"), ("yield_actions", "yield_actions"),
        ("emergency_reroutes", "emergency_reroutes"))}
    return report("identity", {
        "reference": str(ref),
        "same seed, duration and weights as the reference":
            meta["weights"] == dict(DEFAULT_WEIGHTS) and result.checks["seed"] == meta["seed"],
        "tripinfo identical record for record": trips["identical"],
        "tripinfo": trips,
        "coordinator counters equal the reference": all(a == b for a, b in counters.values()),
        "counters (tuner, reference)": counters,
        "pheromone learned (table non-empty)": result.checks["learning"]["pheromone_table_entries"] > 0,
        "learning": result.checks["learning"],
        "no exceptions logged": not result.checks["exceptions_logged"],
        "0 speed-factor differences": result.checks["pairing"]["speed_factor_differences"] == 0,
        "pairing": result.checks["pairing"],
    }, out)


def repeat(args) -> int:
    from scripts.tune_e3_weights import evaluate_weights

    study, out = Path(args.study), Path(args.out)
    recorded_dir = study / "trials" / f"trial_{args.trial:03d}"
    recorded = json.loads((recorded_dir / "trial.json").read_text(encoding="utf-8"))
    from scripts.tune_e3_weights import load_reference
    summary = json.loads((study / "best_e3_weights.json").read_text(encoding="utf-8"))
    reference = load_reference(Path(summary["reference"]), int(recorded["seed"]), int(recorded["sim_time"]))
    result = evaluate_weights(recorded["weights_requested"], Path("simulation.sumocfg"),
                              int(recorded["sim_time"]), int(recorded["seed"]), trial_dir=out,
                              reference=reference)
    trials = list(__import__("csv").DictReader((study / "e3_trials.csv").open(encoding="utf-8")))
    row = next(r for r in trials if int(r["trial"]) == args.trial)
    trips = compare_tripinfo(out / "tripinfo.xml", recorded_dir / "tripinfo.xml")
    return report("repeat", {
        "trial": args.trial,
        "tripinfo identical to the study's trial": trips["identical"],
        "tripinfo": trips,
        "objective identical": repr(result.objective) == repr(float(row["value"])),
        "objective (fresh process, study)": (result.objective, float(row["value"])),
        "coordinator counters identical": result.checks["coordinator"] == recorded["coordinator"],
        "no exceptions logged": not result.checks["exceptions_logged"],
    }, out)


def load(args) -> int:
    """The loading path, and the 2026-09-13 guard that keeps throwaway weight sets out of it.

    A smoke or validation study's file is inspected with require_campaign=False and must be
    REFUSED by the runtime. A copy reshaped as the real campaign would write it (tuning seed,
    3,600 s) must load and be reported as the run's weights source.
    """
    import tempfile
    from algorithms.e3_core import E3Core
    from p3_hybrid_router import DEFAULT_WEIGHTS, load_weights_file

    study = Path(args.study)
    path = (study / "best_e3_weights.json").resolve()
    summary = json.loads(path.read_text(encoding="utf-8"))
    loaded = load_weights_file(path, require_campaign=False)
    default_core = E3Core(emergency_layer=True, reset_state=True)
    refused = False
    os.environ["P3_E3_WEIGHTS"] = str(path)
    try:
        E3Core(emergency_layer=True, reset_state=True)
    except ValueError:
        refused = True
    finally:
        del os.environ["P3_E3_WEIGHTS"]
    campaign_copy = Path(tempfile.mkdtemp(prefix="campaign_shape_")) / "best_e3_weights.json"
    campaign_copy.write_text(json.dumps(dict(summary, seed=20260721, sim_time=3600, campaign_complete=True,
                                             trials_evaluated=25)), encoding="utf-8")
    os.environ["P3_E3_WEIGHTS"] = str(campaign_copy)
    try:
        configured_core = E3Core(emergency_layer=True, reset_state=True)
    except ValueError:
        configured_core = None     # e.g. a pre-2026-09-13 study: scored with another objective
    finally:
        del os.environ["P3_E3_WEIGHTS"]
    return report("load", {
        "file": str(path),
        "the tuner's file parses (inspection only)": loaded == summary["best_weights"],
        "unconfigured runtime keeps DEFAULT_WEIGHTS": default_core.weights_source == "DEFAULT_WEIGHTS"
                                                      and default_core.router.weights == dict(DEFAULT_WEIGHTS),
        "runtime REFUSES this short study's weights": refused,
        "a campaign-shaped file loads and is reported as the source":
            configured_core is not None and configured_core.weights_source == str(campaign_copy)
            and configured_core.router.weights == loaded,
        "weights": loaded,
    }, study)


def objective(args) -> int:
    """For every trial: recompute J from its SUMO files and the reference, and check the counters."""
    from metrics.tuning_objective import OBJECTIVE_WEIGHTS, objective as score, run_metrics

    study = Path(args.study)
    summary = json.loads((study / "best_e3_weights.json").read_text(encoding="utf-8"))
    reference = json.loads(Path(summary["reference"]).read_text(encoding="utf-8"))
    findings, ok = {}, True
    for trial_json in sorted((study / "trials").glob("trial_*/trial.json")):
        recorded = json.loads(trial_json.read_text(encoding="utf-8"))
        m = recorded["metrics"]
        again = score(m, reference["metrics"])
        by_hand = sum(OBJECTIVE_WEIGHTS[k] * again["relative"][k] for k in OBJECTIVE_WEIGHTS)
        trips = ET.parse(trial_json.parent / "tripinfo.xml").getroot().findall("tripinfo")
        finished = sum(1 for t in trips if float(t.get("arrival")) >= 0)
        row = {
            "J recorded": recorded["objective"]["J"], "J recomputed": again["J"],
            "J = sum of weight x relative term": by_hand,
            "finished (metrics, tripinfo, summary)": (m["finished"], finished, m["summary_arrived"]),
            "waiting reported s (not in J)": m["reported_waiting_s_completed"],
            "responses": {e: (v["value_s"], v["status"]) for e, v in m["responses"].items()},
        }
        ok &= (abs(row["J recorded"] - again["J"]) < 1e-12 and abs(by_hand - again["J"]) < 1e-12
               and m["finished"] == finished == m["summary_arrived"])
        findings[trial_json.parent.name] = row
    print(json.dumps(findings, indent=1, default=str))
    return report("objective", {"every J recomputes exactly and every count agrees": ok,
                                "per trial": findings}, study)


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("identity"); p.add_argument("--out", required=True); p.add_argument("--reference", required=True)
    p = sub.add_parser("repeat"); p.add_argument("--study", required=True); p.add_argument("--trial", type=int, required=True); p.add_argument("--out", required=True)
    p = sub.add_parser("load"); p.add_argument("--study", required=True)
    p = sub.add_parser("objective"); p.add_argument("--study", required=True)
    args = parser.parse_args()
    return {"identity": identity, "repeat": repeat, "load": load, "objective": objective}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
