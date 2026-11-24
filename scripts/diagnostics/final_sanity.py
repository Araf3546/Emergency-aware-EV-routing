#!/usr/bin/env python3
"""Pre-campaign sanity checks (2026-09-13). Cheap: the longest run is a 300 s tuner trial.

    python scripts/diagnostics/final_sanity.py --out results/final_sanity_2026-09-13

Checks, each written to verdict.json as PASS/FAIL:
1. a plain `sumo -c simulation.sumocfg` run that names no outputs writes no file anywhere in the
   project (the default outputs used to land in results/baseline_smoke_test/);
2. a real tuner trial (300 s, default weights, tuning seed) writes its files only into its own
   folder, logs no exception, drives the frozen population (0 speed-factor differences), and its
   tripinfo and summary agree on who finished;
3. no weight file is configured, the runtime uses DEFAULT_WEIGHTS, and every weight file under
   results/_invalid/ is refused;
4. the tuning seed maps to its own frozen demand file (hash matches its manifest) and the campaign
   reference matches that seed, duration, demand and incident file.

Since PD-1 question 2 (2026-09-13, later) it also checks that the trial ran with the ambient
emergency types as `authority` and that the campaign reference was built under the same rule.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
os.chdir(REPO_ROOT)


def snapshot(skip: Path) -> dict:
    """Every file in the project (except caches and `skip`) with its size and mtime."""
    out = {}
    for path in REPO_ROOT.rglob("*"):
        if not path.is_file() or "__pycache__" in path.parts or ".pytest_cache" in path.parts:
            continue
        if skip in path.parents:
            continue
        stat = path.stat()
        out[str(path.relative_to(REPO_ROOT))] = (stat.st_size, stat.st_mtime_ns)
    return out


def changed(before: dict, after: dict) -> list:
    return sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    out = Path(parser.parse_args().out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    verdict = {}

    # 1. a run that names no outputs writes nothing
    before = snapshot(out)
    run = subprocess.run(["sumo", "-c", "simulation.sumocfg", "--end", "5", "--no-step-log", "true"],
                         capture_output=True, text=True)
    after = snapshot(out)
    verdict["plain sumocfg run exits 0"] = run.returncode == 0
    verdict["plain sumocfg run writes no file"] = not changed(before, after)
    verdict["files changed by the plain run"] = changed(before, after)
    verdict["results/baseline_smoke_test absent"] = not (REPO_ROOT / "results" / "baseline_smoke_test").exists()

    # 2. a real tuner trial writes only into its own folder
    from p3_hybrid_router import DEFAULT_WEIGHTS
    from scripts.tune_e3_weights import evaluate_weights
    trial_dir = out / "trial_300s"
    before = snapshot(out)
    result = evaluate_weights(dict(DEFAULT_WEIGHTS), Path("simulation.sumocfg"), 300, 20260721,
                              trial_dir=trial_dir)
    after = snapshot(out)
    checks = result.checks
    verdict["trial writes nothing outside its folder"] = not changed(before, after)
    verdict["files changed outside the trial folder"] = changed(before, after)
    verdict["trial folder contents"] = sorted(p.name for p in trial_dir.iterdir())
    verdict["trial folder has tripinfo, summary, logs and trial.json"] = {
        "tripinfo.xml", "summary.xml", "errors.log", "messages.log", "trial.json"} <= set(verdict["trial folder contents"])
    verdict["trial logged no exception"] = not checks["exceptions_logged"]
    verdict["trial: 0 speed-factor differences"] = checks["pairing"]["speed_factor_differences"] == 0
    verdict["trial pairing"] = checks["pairing"]
    verdict["trial: tripinfo and summary agree on who finished"] = checks["finished_equals_summary_arrived"]
    verdict["trial ran DEFAULT_WEIGHTS (no weight file configured)"] = (
        checks["weights_source_overridden"] == "DEFAULT_WEIGHTS")
    verdict["trial frozen demand"] = checks["frozen_demand"]
    verdict["trial used the tuning seed's frozen file"] = checks["frozen_demand"].endswith("porto.rou.20260721.xml")
    # PD-1 question 2 (2026-09-13, later): the ambient emergency types ran as `authority`.
    from emergency_manager import ROAD_PERMISSION_POLICY
    verdict["trial ran under the PD-1 question 2 road permissions (ambient types authority)"] = (
        checks["road_permissions"] == ROAD_PERMISSION_POLICY
        and set(checks["ambient_road_classes"].values()) == {"authority"}
        and len(checks["ambient_road_classes"]) == 4)

    # 3. weights
    from algorithms.e3_core import E3Core, configured_weights_file
    from p3_hybrid_router import load_weights_file
    verdict["no weight file configured"] = configured_weights_file() is None and not os.environ.get("P3_E3_WEIGHTS")
    verdict["runtime source is DEFAULT_WEIGHTS"] = E3Core(emergency_layer=True).weights_source == "DEFAULT_WEIGHTS"
    refused, accepted = [], []
    for path in sorted((REPO_ROOT / "results" / "_invalid").rglob("best_e3_weights.json")):
        try:
            load_weights_file(path)
            accepted.append(str(path.relative_to(REPO_ROOT)))
        except ValueError:
            refused.append(str(path.relative_to(REPO_ROOT)))
    verdict["every quarantined weight file is refused"] = bool(refused) and not accepted
    verdict["quarantined weight files refused"] = refused
    for study in ("results/tuner_smoke_2026-09-12/cli_2x700", "results/tuner_objective_validation_2026-09-13/cli_2x1800"):
        try:
            load_weights_file(REPO_ROOT / study / "best_e3_weights.json")
            verdict[f"throwaway study refused: {study}"] = False
        except ValueError:
            verdict[f"throwaway study refused: {study}"] = True

    # 4. seed -> frozen demand -> reference
    from evaluation_seeds import checked_frozen_demand, frozen_demand_paths, tuning_seed
    from scripts.tune_e3_weights import contract_tuning_duration, default_reference_path, load_reference
    seed = tuning_seed()
    route_file, manifest_path = frozen_demand_paths(seed)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verdict["tuning seed is 20260721"] = seed == 20260721
    verdict["tuning seed maps to demand/seeds/porto.rou.20260721.xml"] = route_file.name == "porto.rou.20260721.xml"
    verdict["frozen file passes evaluation_seeds' manifest check"] = checked_frozen_demand(seed) == route_file
    verdict["manifest names the tuning seed"] = int(manifest.get("sumo_seed", -1)) == seed
    verdict["contract tuning duration is 3600"] = contract_tuning_duration() == 3600
    reference_path = default_reference_path(seed, 3600)
    load_reference(reference_path, seed, 3600)           # raises on any mismatch
    verdict["campaign reference matches seed, duration, demand, incidents and road permissions"] = True
    verdict["campaign reference"] = str(reference_path.relative_to(REPO_ROOT))

    ok = all(v for v in verdict.values() if isinstance(v, bool))
    verdict = {"pass": ok, **verdict}
    (out / "verdict.json").write_text(json.dumps(verdict, indent=1, default=str), encoding="utf-8")
    for key, value in verdict.items():
        if isinstance(value, bool):
            print(f"{'PASS' if value else 'FAIL'}  {key}")
    print("final sanity:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
